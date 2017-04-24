#include "dinasour/position_joint_group_controller.h"

namespace dinasour
{

PositionJointGroupController::PositionJointGroupController()
{
        Loop_Count = 0;
}
PositionJointGroupController::~PositionJointGroupController()
{

}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: initialize joints from robot_description
**************************************************************************/
bool PositionJointGroupController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
                ROS_ERROR("Failed to parse urdf file");
                return false;
        }
        n_joints_ = 0;
        while(true)
        {
                std::string joint_name;
                std::string param_name = std::string("joint_" + to_string(n_joints_));
                if (n.getParam(param_name.c_str(), joint_name))
                {
                        joint_names_.push_back(joint_name);
                }
                else
                {
                        break;
                }
                n_joints_++;
        }
        for(std::vector<std::string>::iterator iter = joint_names_.begin(); iter!=joint_names_.end(); iter++)
        {
                // Get joint handle from hardware interface
                joints_.push_back(robot->getHandle(*iter));
        }

        joint_state_publisher_.reset( new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(n, "/dragon/joint_state", 1));

        for(int i=0; i<Joint_Num; i++)
        {
                commands.push_back(0);
        }

        if(n_joints_ == Joint_Num)
        {
                std::cout<<"System Init Succeed!"<<std::endl;
        }
        return true;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: set model initial state
**************************************************************************/
void PositionJointGroupController::starting(const ros::Time& time)
{
        for(unsigned int i=0; i<n_joints_; i++)
        {
                commands[i]=joints_[i].getPosition();
        }
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void PositionJointGroupController::update(const ros::Time& time, const ros::Duration& period)
{
        switch (Time_Order)
        {
        case 0: //init height
                pose_init();
                break;
        case 1://assign next footholder value
                assign_next_foot();
                break;
        case 2://adjust CoG
                cog_adj();
                break;
        case 3: //swing leg
                swing_control();
                break;
        default: break;
        }

        Loop_Count++;

        commands = vec_assign(Angle_ptr);

        for(int i=0; i<n_joints_; i++)
        {
                joints_[i].setCommand(commands[i]);
        }

        if(joint_state_publisher_ && joint_state_publisher_->trylock())
        {
                joint_state_publisher_->msg_.data.clear();
                for(auto i:commands)
                {
                        joint_state_publisher_->msg_.data.push_back(i);
                }
                joint_state_publisher_->unlockAndPublish();
        }
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating leg position while swing, design swing foot trajectory,now using ellipse, need to work on in the future
   Input: start point,and desired point,and sample time order(1~)
   Output: innerheart position
**************************************************************************/
_Position PositionJointGroupController::get_swing_pos(_Position Start_point, _Position End_point, unsigned int Loop)
{
        unsigned int last_order = Loop - 1;
        if(Loop==0)
        {
                last_order = 0;
        }

        int sgn = Sgn(End_point.x - Start_point.x);
        _Position swing_foot = {0,0,0};
        double theta = Loop * PI / Swing_Num;
        double b = Swing_Height;
        double a = fabs((End_point.x - Start_point.x) / 2.0);

        swing_foot.x = a - sgn * a * cos(theta);
        swing_foot.z = b * sin(theta);

        sgn = Sgn(End_point.y - Start_point.y);
        a = fabs((End_point.y - Start_point.y) / 2.0);
        swing_foot.y = a - sgn * a * cos(theta);

        theta = last_order * PI / Swing_Num;
        sgn = Sgn(End_point.x - Start_point.x);
        a = fabs((End_point.x - Start_point.x) / 2.0);
        swing_foot.x = swing_foot.x - a + sgn * a * cos(theta);
        swing_foot.z = swing_foot.z - b * sin(theta);
        sgn = Sgn(End_point.y - Start_point.y);
        a = fabs((End_point.y - Start_point.y) / 2.0);
        swing_foot.y = swing_foot.y - a + sgn * a * cos(theta);
        return swing_foot;
        // std::cout<<"Adj_Vector:"<<swing_foot.x<<"  "<<swing_foot.y<<"  "<<swing_foot.z<<std::endl;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after knowing the exact distance to move CoG, design proper trajectory
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime position
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Pos(t):6 * XD * t^5 / Stance_Num^5 - 15 * XD * t^4 / Stance_Num^4 + 10 * XD * t^3 / Stance_Num^3
   Y-axis:
   Pos(t):6 * YD * t^5 / Stance_Num^5 - 15 * YD * t^4 / Stance_Num^4 + 10 * YD * t^3 / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position PositionJointGroupController::get_stance_position(_Position Adj_vec, unsigned int Loop)
{
        if(Loop>Stance_Num)
        {
                ROS_ERROR("Time Order wrong while stance");
        }

        unsigned int last_order = Loop - 1;
        if(Loop==0)
        {
                last_order = 0;
        }

        _Position stance_adj = {0,0,0};
        stance_adj.x = 6 * Adj_vec.x * pow(Loop,5) / pow(Stance_Num,5)
                       - 15 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,4)
                       + 10 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,3);
        stance_adj.y = 6 * Adj_vec.y * pow(Loop,5) / pow(Stance_Num,5)
                       - 15 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,4)
                       + 10 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,3);

        stance_adj.x = stance_adj.x - (6 * Adj_vec.x * pow(last_order,5) / pow(Stance_Num,5)
                                       - 15 * Adj_vec.x * pow(last_order,4) / pow(Stance_Num,4)
                                       + 10 * Adj_vec.x * pow(last_order,3) / pow(Stance_Num,3));
        stance_adj.y = stance_adj.y - (6 * Adj_vec.y * pow(last_order,5) / pow(Stance_Num,5)
                                       - 15 * Adj_vec.y * pow(last_order,4) / pow(Stance_Num,4)
                                       + 10 * Adj_vec.y * pow(last_order,3) / pow(Stance_Num,3));
        return stance_adj;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: initialize body pose at first time
   Input: position and Angle_ptr pointer
   Output: none
**************************************************************************/
void PositionJointGroupController::swing_control()
{
        _Position s = {0,0,0};

        if(Loop_Count < Swing_Num)
        {
                s = get_swing_pos(Pos_start, Desired_Foot_Pos, Loop_Count);
                switch(Leg_Order)
                {
                case 1: //swing left front leg
                        Pos_ptr->lf = struct_assign(Pos_ptr->lf.x + s.x,Pos_ptr->lf.y + s.y,Pos_ptr->lf.z + s.z);
                        break;
                case 2: //swing right front leg
                        Pos_ptr->rf = struct_assign(Pos_ptr->rf.x + s.x,Pos_ptr->rf.y + s.y,Pos_ptr->rf.z + s.z);
                        break;
                case 3: //swing left back leg
                        Pos_ptr->lb = struct_assign(Pos_ptr->lb.x + s.x,Pos_ptr->lb.y + s.y,Pos_ptr->lb.z + s.z);
                        break;
                case 4: //swing right back leg
                        Pos_ptr->rb = struct_assign(Pos_ptr->rb.x + s.x,Pos_ptr->rb.y + s.y,Pos_ptr->rb.z + s.z);
                        break;
                default: break;
                }
                reverse_kinematics();
        }
        else
        {
                Loop_Count = 0;
                Time_Order = 1;
                Leg_Order = ((Leg_Order+2)>4) ? (5-Leg_Order) : (Leg_Order+2);
        }
}
void PositionJointGroupController::cog_pos_assign(_Position Adj)
{
        Pos_ptr->lf = struct_assign(Pos_ptr->lf.x - Adj.x,
                                    Pos_ptr->lf.y - Adj.y, Pos_ptr->lf.z);
        Pos_ptr->rf = struct_assign(Pos_ptr->rf.x - Adj.x,
                                    Pos_ptr->rf.y - Adj.y, Pos_ptr->rf.z);
        Pos_ptr->lb = struct_assign(Pos_ptr->lb.x - Adj.x,
                                    Pos_ptr->lb.y - Adj.y, Pos_ptr->lb.z);
        Pos_ptr->rb = struct_assign(Pos_ptr->rb.x - Adj.x,
                                    Pos_ptr->rb.y - Adj.y, Pos_ptr->rb.z);
}
void PositionJointGroupController::cog_adj()
{
        _Position adj = {0,0,0};
        switch(Leg_Order)
        {
        case 1://adjust CoG to right side
                if(Loop_Count <= Stance_Num)
                {
                        if(Loop_Count==1)
                        {
                                Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos,Leg_Order); //1 means left
                        }
                        adj = get_stance_position(Cog_adj, Loop_Count);
                        cog_pos_assign(adj);
                        reverse_kinematics();
                }
                else
                {
                        Loop_Count = 0;
                        Time_Order = 3;
                        Leg_Order = 1;
                        Pos_start = struct_copy(Pos_ptr->lf);
                }
                break;
        case 2://adjust CoG to left side
                if(Loop_Count <= Stance_Num)
                {
                        if(Loop_Count==1)
                        {
                                Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos,Leg_Order); //1 means left
                        }
                        adj = get_stance_position(Cog_adj, Loop_Count);
                        cog_pos_assign(adj);
                        reverse_kinematics();
                }
                else
                {
                        Loop_Count = 0;
                        Time_Order = 3;
                        Leg_Order = 2;
                        Pos_start = struct_copy(Pos_ptr->rf);
                }
                break;
        case 3:
                Pos_start = struct_copy(Pos_ptr->lb);
                Time_Order = 3;
                break;
        case 4:
                Pos_start = struct_copy(Pos_ptr->rb);
                Time_Order = 3;
                break;
        }
}
void PositionJointGroupController::assign_next_foot()
{
        switch(Leg_Order)//choose swing foot
        {
        case 1:
                Desired_Foot_Pos = struct_assign(Body_L - 10, Body_W, -Height);
                break;
        case 2:
                Desired_Foot_Pos = struct_assign(Body_L - 10, -Body_W, -Height);
                break;
        case 3:
                Desired_Foot_Pos = struct_assign(-Body_L - 15, Body_W, -Height);
                break;
        case 4:
                Desired_Foot_Pos = struct_assign(-Body_L - 15, -Body_W, -Height);
                break;
        default: break;
        }

        Loop_Count = 0;
        Time_Order = 2;
}
void PositionJointGroupController::pose_init()
{
        float adj = fabs(L0 + L1 + L2 - Height);//TODO

        if(Loop_Count>Init_Num)//TODO
        {
                Loop_Count = 0;
                Time_Order = 1;
                return;
        }

        Pos_ptr->lb.z = Pos_ptr->rf.z = Pos_ptr->rb.z = Pos_ptr->lf.z = -(L0 + L1 + L2) + get_adj_pos(adj, Loop_Count, Init_Num);

        reverse_kinematics();
}
float PositionJointGroupController::get_adj_pos(float Adj, int t, int T)
{
        float pos = 6 * Adj * pow(t,5) / pow(T,5) - 15 * Adj * pow(t,4) / pow(T,4)
                    + 10 * Adj * pow(t,3) / pow(T,3);
        return pos;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: calculating forward kinematics for quadruped robot(3 DOF) using D-H methods
   Formula:
   Px = Xrf + L1 * S1 + L2 * S12;
   Py = Yrf + L0 * S0 + L1 * S0 * C1 + L2 * S0 * C12;
   Pz = Zrf - Lo * C0 - L1 * C0 * C1 - L2 * C0 * C12;
**************************************************************************/
_Position PositionJointGroupController::cal_formula(_Angle_Leg A,int L,int W)
{
        _Position p;
        p.x = L + L1 * sin(A.hip) + L2 * sin(A.hip + A.knee);
        p.y = W + L0 * sin(A.pitch) + L1 * sin(A.pitch) * cos(Angle_ptr->lf.hip)
              + L2 * sin(A.pitch) * cos(Angle_ptr->lf.hip + A.knee);
        p.z = -L0 * cos(A.pitch) - L1 * cos(A.pitch) * cos(A.hip)
              - L2 * cos(A.pitch) * cos(A.hip + A.knee);
        return p;
}
void PositionJointGroupController::forward_kinematics()
{
        Pos_ptr->lf = cal_formula(Angle_ptr->lf,Body_L,Body_W);
        Pos_ptr->rf = cal_formula(Angle_ptr->rf,Body_L,-Body_W);
        Pos_ptr->lb = cal_formula(Angle_ptr->lb,-Body_L,Body_W);
        Pos_ptr->rb = cal_formula(Angle_ptr->rb,-Body_L,-Body_W);
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: calculating reverse kinematics for quadruped robot(3 DOF)
   Formula:
   Theta_0 = atan((Yrf - Py) / (Pz - Zrf));
   Theta_1 = 2 * atan((Epsilon + sqrt(Epsilon^2 - Phi * (L2 * S2 - Delte))) / Phi);
   Theta_2 = sgn * acos((Delte^2 + Epsilon^2 - L1^2 - L2^2) / 2 / L1 / L2);
   Meantime, Delte = Px - Xrf; Phi = Delte + L2 * S2; Epsilon = L0 + Pz * C0 - Zrf * C0 - Py * S0 + Yrf * S0;
**************************************************************************/
_Angle_Leg PositionJointGroupController::cal_kinematics(_Position P,int L,int W,int Sgn)
{
        _Angle_Leg A;
        double Delte = P.x - L;
        A.pitch = atan((W - P.y) / (P.z ));
        double Epsilon = L0 + P.z * cos(A.pitch) - P.y * sin(A.pitch) + W * sin(A.pitch);
        A.knee = Sgn * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2)) / 2.0 / L1 / L2);
        double Phi = Delte + L2 * sin(A.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        A.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(A.knee) - Delte))) / Phi);
        return A;
}
void PositionJointGroupController::reverse_kinematics()
{
        //front
        Angle_ptr->lf = cal_kinematics(Pos_ptr->lf, Body_L, Body_W,-1);
        Angle_ptr->rf = cal_kinematics(Pos_ptr->rf, Body_L,-Body_W,-1);
        Angle_ptr->lb = cal_kinematics(Pos_ptr->lb,-Body_L, Body_W,1);
        Angle_ptr->rb = cal_kinematics(Pos_ptr->rb,-Body_L,-Body_W,1);
        // return;
        // double Delte = Pos_ptr->lf.x - Body_L;
        // Angle_ptr->lf.pitch = atan((Body_W - Pos_ptr->lf.y) / (Pos_ptr->lf.z ));
        // double Epsilon = L0 + Pos_ptr->lf.z * cos(Angle_ptr->lf.pitch)
        //                  - Pos_ptr->lf.y * sin(Angle_ptr->lf.pitch) + Body_W * sin(Angle_ptr->lf.pitch);
        // Angle_ptr->lf.knee = -1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2))
        //                                / 2.0 / L1 / L2);
        // double Phi = Delte + L2 * sin(Angle_ptr->lf.knee);
        // if(Phi == 0) {Phi = Phi + 0.000001;}
        // Angle_ptr->lf.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(Angle_ptr->lf.knee) - Delte))) / Phi);
        //
        //
        // Delte = Pos_ptr->rf.x - Body_L;
        // Angle_ptr->rf.pitch = atan((-Body_W - Pos_ptr->rf.y) / (Pos_ptr->rf.z));
        // Epsilon = L0 + Pos_ptr->rf.z * cos(Angle_ptr->rf.pitch)
        //           - Pos_ptr->rf.y * sin(Angle_ptr->rf.pitch) + -Body_W * sin(Angle_ptr->rf.pitch);
        // Angle_ptr->rf.knee = -1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2))
        //                                / 2.0 / L1 / L2);
        // Phi = Delte + L2 * sin(Angle_ptr->rf.knee);
        // if(Phi == 0) {Phi = Phi + 0.000001;}
        // Angle_ptr->rf.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(Angle_ptr->rf.knee) - Delte))) / Phi);
        //
        // //back
        // Delte = Pos_ptr->lb.x - -Body_L;
        // Angle_ptr->lb.pitch = atan((Body_W - Pos_ptr->lb.y) / (Pos_ptr->lb.z));
        //
        // Epsilon = L0 + Pos_ptr->lb.z * cos(Angle_ptr->lb.pitch)
        //           - Pos_ptr->lb.y * sin(Angle_ptr->lb.pitch) + Body_W * sin(Angle_ptr->lb.pitch);
        // Angle_ptr->lb.knee = 1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2))
        //                               / 2.0 / L1 / L2);
        // Phi = Delte + L2 * sin(Angle_ptr->lb.knee);
        // if(Phi == 0) {Phi = Phi + 0.000001;}
        // Angle_ptr->lb.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(Angle_ptr->lb.knee) - Delte))) / Phi);
        //
        //
        // Delte = Pos_ptr->rb.x - -Body_L;
        // Angle_ptr->rb.pitch = atan((-Body_W - Pos_ptr->rb.y) / Pos_ptr->rb.z );
        // Epsilon = L0 + Pos_ptr->rb.z * cos(Angle_ptr->rb.pitch)
        //           - Pos_ptr->rb.y * sin(Angle_ptr->rb.pitch) + -Body_W * sin(Angle_ptr->rb.pitch);
        // Angle_ptr->rb.knee = 1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2))
        //                               / 2.0 / L1 / L2);
        // Phi = Delte + L2 * sin(Angle_ptr->rb.knee);
        // if(Phi == 0) {Phi = Phi + 0.000001;}
        // Angle_ptr->rb.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(Angle_ptr->rb.knee) - Delte))) / Phi);

}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating CoG position and CoG adjust vector,using next foothold,while 1 means to right side
   Input: four feet(end effector) position(x,y,z),Swing leg order and next Swing leg position
   Output: CoG adjust vector
**************************************************************************/
_Position PositionJointGroupController::get_CoG_adj_vec(_Position Next_Foothold, unsigned int Swing_Order)
{
        _Position point = {0,0,0};
        switch (Swing_Order)
        {
        case 1:
                point = get_cross_point(Next_Foothold, Pos_ptr->rb, Pos_ptr->rf, Pos_ptr->lb);
                point = get_innerheart(point, Pos_ptr->rf, Pos_ptr->rb);
                break;
        case 2:
                point = get_cross_point(Next_Foothold, Pos_ptr->lb, Pos_ptr->lf, Pos_ptr->rb);
                point = get_innerheart(point, Pos_ptr->lf, Pos_ptr->lb);
                break;
        default: break;
        }

        point.z = 0;         //Default CoG={0，0，-1 * Height}
        return point;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating line crosspoint position
   Input: four point position
   Output: crosspoint position
**************************************************************************/
_Position PositionJointGroupController::get_cross_point(_Position A_start, _Position A_end, _Position B_start, _Position B_end)
{
        int flag_one, flag_two;
        double k1, k2, b1, b2;
        _Position cross_point = {0,0,0};
        //line 1
        if (A_start.x == A_end.x)
                flag_one = 1;
        else if (A_start.y == A_end.y)
                flag_one = 2;
        else
        {
                k1 = (A_start.y - A_end.y )/(A_start.x - A_end.x);
                b1 = A_start.y - k1 * A_start.x;
                flag_one = 3;
        }
        //line 2
        if (B_start.x == B_end.x)
                flag_two = 1;
        else if (B_start.y == B_end.y)
                flag_two = 2;
        else
        {
                k2 = (B_start.y - B_end.y )/(B_start.x - B_end.x);
                b2 = B_start.y - k2 * B_start.x;
                flag_two = 3;
        }
        switch (flag_one)
        {
        case 1:
                switch (flag_two)
                {
                case 1:
                        std::cout<<"Error! foot piont cross error in get_cross_cog_margin"<<std::endl;
                        break;
                case 2:
                        cross_point.x = A_start.x;
                        cross_point.y = B_start.y;
                        break;
                case 3:
                        cross_point.x = A_start.x;
                        cross_point.y = k2 * A_start.x + b2;
                        break;
                default: break;
                }
                break;
        case 2:
                switch (flag_two)
                {
                case 1:
                        cross_point.x = A_start.y;
                        cross_point.y = B_start.x;
                        break;
                case 2:
                        std::cout<<"Error! foot piont cross error in get_cross_cog_margin"<<std::endl;
                        break;
                case 3:
                        cross_point.x = (A_start.y - b2) / k2;
                        cross_point.y = A_start.y;
                        break;
                default: break;
                }
                break;
        case 3:
                switch (flag_two)
                {
                case 1:
                        cross_point.x = B_start.x;
                        cross_point.y = k1 * B_start.x + b1;
                        break;
                case 2:
                        cross_point.x = (B_start.y - b1) / k1;
                        cross_point.y = B_start.y;
                        break;
                case 3:
                        if (k1==k2)
                                std::cout<<"Error! foot piont cross error in get_cross_cog_margin"<<std::endl;
                        else
                        {
                                cross_point.x = (b1-b2)/(k2-k1);
                                cross_point.y = k1 * (b1-b2)/(k2-k1) + b1;
                        }
                        break;
                }
        default: break;
        }
        cross_point.z = -1 * Height;
        return cross_point;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating triangle innerheart
   Input: three point position
   Output: innerheart position
**************************************************************************/
_Position PositionJointGroupController::get_innerheart(_Position A, _Position B, _Position C)
{
        double a = 0, b = 0, c = 0;
        _Position heart = {0,0,0};

        a = sqrt(pow(B.x - C.x, 2) + pow(B.y - C.y, 2));
        b = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2));
        c = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));

        heart.x = (a * A.x + b * B.x + c * C.x ) / (a + b + c);
        heart.y = (a * A.y + b * B.y + c * C.y ) / (a + b + c);
        heart.z = -1 * Height;
        return heart;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after design the exact trajectory, providing velocity while move the body
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime velocity
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Vel(t):30 * XD * t^4 / Stance_Num^5 - 60 * XD * t^3 / Stance_Num^4 + 30 * XD * t^2 / Stance_Num^3
   Y-axis:
   Vel(t):30 * YD * t^4 / Stance_Num^5 - 60 * YD * t^3 / Stance_Num^4 + 30 * YD * t^2 / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position PositionJointGroupController::get_stance_velocity(_Position Adj_vec, unsigned int Loop)
{
        if(Loop>Stance_Num)
        {
                ROS_ERROR("Time Order wrong while stance");
        }

        _Position stance_vel = {0,0,0};
        stance_vel.x = 30 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,5) - 60 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,4)
                       + 30 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,3);
        stance_vel.y = 30 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,5) - 60 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,4)
                       + 30 * Adj_vec.y * pow(Loop,2) / pow(Stance_Num,3);
        return stance_vel;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after design the exact velocity, providing accelation while move the body
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime accelation
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Acc(t):120 * XD * t^3 / Stance_Num^5 - 180 * XD * t^2 / Stance_Num^4 + 60 * XD * t / Stance_Num^3
   Y-axis:
   Acc(t):120 * YD * t^3 / Stance_Num^5 - 180 * YD * t^2 / Stance_Num^4 + 60 * YD * t / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position PositionJointGroupController::get_stance_acceration(_Position Adj_vec, unsigned int Loop)
{
        if(Loop>Stance_Num)
        {
                ROS_ERROR("Time Order wrong while stance");
        }

        _Position stance_acc = {0,0,0};
        stance_acc.x = 120 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,5) - 180 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,4)
                       + 60 * Adj_vec.x * Loop / pow(Stance_Num,3);
        stance_acc.y = 120 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,5) - 180 * Adj_vec.y * pow(Loop,2) / pow(Stance_Num,4)
                       + 60 * Adj_vec.y * Loop / pow(Stance_Num,3);
        return stance_acc;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: copy struct data to another one
   Input: struct data
   Output: copy data
**************************************************************************/
_Position PositionJointGroupController::struct_copy(_Position A)
{
        _Position copy_struct = {0,0,0};
        copy_struct.x = A.x;
        copy_struct.y = A.y;
        copy_struct.z = A.z;
        return copy_struct;
}
_Position PositionJointGroupController::struct_assign(double x, double y, double z)
{
        _Position copy;
        copy.x = x;
        copy.y = y;
        copy.z = z;
        return copy;
}
int PositionJointGroupController::Sgn(double a)
{
        if (a<0)
                return -1;
        else return 1;
}
std::vector<double> PositionJointGroupController::vec_assign(Angle_Ptr Angle)
{
        std::vector<double> vec;
        vec.clear();
        vec.push_back(Angle->lf.pitch);
        vec.push_back(Angle->rf.pitch);
        vec.push_back(Angle->lb.pitch);
        vec.push_back(Angle->rb.pitch);
        vec.push_back(Angle->lf.hip);
        vec.push_back(Angle->rf.hip);
        vec.push_back(Angle->lb.hip);
        vec.push_back(Angle->rb.hip);
        vec.push_back(Angle->lf.knee);
        vec.push_back(Angle->rf.knee);
        vec.push_back(Angle->lb.knee);
        vec.push_back(Angle->rb.knee);
        return vec;
}


}
PLUGINLIB_DECLARE_CLASS(dinasour, PositionJointGroupController,
                        dinasour::PositionJointGroupController,
                        controller_interface::ControllerBase)
// PLUGINLIB_EXPORT_CLASS(dragon_control::ForwardJointGroupCommandController,controller_interface::ControllerBase)
