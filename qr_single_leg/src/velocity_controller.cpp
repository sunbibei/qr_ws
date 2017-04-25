
#include "dragon_single_leg/velocity_controller.h"

namespace dragon_single_leg
{

VelocityController::VelocityController()
{
        Loop_Count = 0;
        Phase = 0;
}
VelocityController::~VelocityController()
{

}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: initialize joints from robot_description
**************************************************************************/
bool VelocityController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
        Pos.x = 0;
        Pos.y = 0;
        Pos.z = 0;
        Angle.pitch = 0;
        Angle.hip = 0;
        Angle.knee = 0;
        commands.push_back(0);
        commands.push_back(0);
        commands.push_back(0);
        commands.push_back(0);
        commands.push_back(0);
        commands.push_back(0);

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

        if(n_joints_== 3)
        {
                std::cout<<"System Init Succeed!"<<std::endl;
        }
        return true;
}
void VelocityController::starting(const ros::Time& time)
{
        // Start controller with current joint positions
        for(unsigned int i=0; i<n_joints_; i++)
        {
                commands[i]=joints_[i].getPosition();
        }
}

void VelocityController::update(const ros::Time& time, const ros::Duration& period)
{
        _Position Start_point = {10,10,-Height};
        _Position End_point = {-10,-10,-Height};

        switch(Phase)// 0:初始化 1:上下移动 2:swing
        {
        case 0://到达指定高度
                reset(Loop_Count);
                break;
        case 1://up and down
                updown();
                break;
        case 2://swing and stance
                sas(Start_point, End_point);
                break;

        default: break;
        }

        /****************************************/
        commands[0] = Angle.pitch;
        commands[1] = Angle.hip;
        commands[2] = Angle.knee;
        for(unsigned int i=0; i<n_joints_; i++)
        {
                joints_[i].setCommand(commands[i]);//set position
                // joints_[i].setCommand(commands[i+3]); //set velocity
        }

        if(joint_state_publisher_ && joint_state_publisher_->trylock())
        {
                joint_state_publisher_->msg_.data.clear();

                for(unsigned int i=0; i<n_joints_; i++)
                {
                        joint_state_publisher_->msg_.data.push_back(commands[i]);
                }
                joint_state_publisher_->unlockAndPublish();
        }
        Loop_Count++;
}

void VelocityController::reset(int t)
{
        float adj = fabs(l0 + l1 + l2 - Height);//TODO
        int T = 1000;

        if(fabs(Pos.z + Height)<0.001 || t>T)//TODO
        {
                Phase = 1;
                Loop_Count = 0;
                std::cout<<"reset done!"<<std::endl;
                return;
        }
        /****************************************/
        // Angle.pitch = joints_[0].getPosition();//TODO--> may need trans
        // Angle.hip   = joints_[1].getPosition();
        // Angle.knee  = joints_[2].getPosition();
        // forward_kinematics();
        Pos.z = -(l0 + l1 + l2) + get_adj_pos(adj, t, T);
        reverse_kinematics();
}

void VelocityController::updown()
{
        if(Loop_Count<1000)
        {
                forward_kinematics();
                Pos.z = Pos.z - 0.005;
                reverse_kinematics();
        }
        else if(Loop_Count<2000)
        {
                forward_kinematics();
                Pos.z = Pos.z + 0.005;
                reverse_kinematics();
        }
        else
        {
                Loop_Count = 0;
                Phase = 1;
                std::cout<<"up and down done!"<<std::endl;
        }
}

_Position VelocityController::get_eclipse_pos(_Position Start_point, _Position End_point,int Loop)
{
        _Position swing_foot = {0,0,0};
        int sgn = Sgn( End_point.x - Start_point.x);
        float angle = PI - PI * Loop/Swing_Num;
        float centre = (Start_point.x + End_point.x)/2;
        float a =  fabs(Start_point.x - End_point.x)/2;
        float b = Swing_Height;
        swing_foot.x = sgn*(a + a*cos(angle));
        swing_foot.z = b*sin(angle);

        sgn = Sgn( End_point.y - Start_point.y);
        a =  fabs(Start_point.y - End_point.y)/2;

        swing_foot.y = sgn*(a + a*cos(angle));

        return swing_foot;
}

_Position VelocityController::get_stance_position(_Position Start_point,_Position End_point,int Loop)
{
        _Position Adj_vec;
        Adj_vec.x = End_point.x - Start_point.x;
        Adj_vec.y = End_point.y - Start_point.y;

        _Position stance_adj = {0,0,0};
        stance_adj.x = 6 * Adj_vec.x * pow(Loop,5) / pow(Stance_Num,5)
                       - 15 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,4)
                       + 10 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,3);
        stance_adj.y = 6 * Adj_vec.y * pow(Loop,5) / pow(Stance_Num,5)
                       - 15 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,4)
                       + 10 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,3);

        stance_adj.z = -Height;
        return stance_adj;
}

void VelocityController::sas(_Position Start_point, _Position End_point)
{
        if(Loop_Count<1000)//swing
        {
                _Position adj = get_eclipse_pos(Start_point, End_point, Loop_Count);
                Pos.x = Start_point.x + adj.x;
                Pos.y = Start_point.y + adj.y;
                Pos.z = Start_point.z + adj.z;
                reverse_kinematics();
        }
        else if(Loop_Count<2000)//stance
        {
                _Position adj = get_stance_position(End_point, Start_point, Loop_Count-1000);
                Pos.x = End_point.x + adj.x;
                Pos.y = End_point.y + adj.y;
                reverse_kinematics();
        }
        else
        {
                Loop_Count = 0;
                Phase = 2;
                std::cout<<"swing done!"<<std::endl;
        }
}

float VelocityController::get_adj_pos(float Adj, int t, int T)
{
        float pos = 6 * Adj * pow(t,5) / pow(T,5) - 15 * Adj * pow(t,4) / pow(T,4)
                    + 10 * Adj * pow(t,3) / pow(T,3);
        return pos;
}

float VelocityController::get_adj_vel(float Adj, int t, int T)
{
        float vel = 30 * Adj * pow(t,4) / pow(T,5) - 60 * Adj * pow(t,3) / pow(T,4)
                    + 30 * Adj * pow(t,2) / pow(T,3);
        return vel;
}

float VelocityController::get_adj_acc(float Adj, int t, int T)
{
        float acc = 120 * Adj * pow(t,3) / pow(T,5) - 180 * Adj * pow(t,2) / pow(T,4)
                    + 60 * Adj * t / pow(T,3);
        return acc;
}

void VelocityController::forward_kinematics()
{
        Pos.x = l1 * sin(Angle.hip) + l2 * sin(Angle.hip + Angle.knee);
        Pos.y = l0 * sin(Angle.pitch) + l1 * sin(Angle.pitch) * cos(Angle.hip) + l2 * sin(Angle.pitch) * cos(Angle.hip + Angle.knee);
        Pos.z = -l0 * cos(Angle.pitch) - l1 * cos(Angle.pitch) * cos(Angle.hip) - l2 * cos(Angle.pitch) * cos(Angle.hip + Angle.knee);
}
void VelocityController::reverse_kinematics()
{
        Angle.pitch = atan(-Pos.y / Pos.z);

        double Epsilon = l0 + Pos.z * cos(Angle.pitch) - Pos.y * sin(Angle.pitch);
        Angle.knee = -1 * acos((pow(Pos.x,2) + pow(Epsilon,2) - pow(l1,2) - pow(l2,2)) / 2.0 / l1 / l2);

        double Phi = Pos.x + l2 * sin(Angle.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        Angle.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (l2 * sin(Angle.knee) - Pos.x))) / Phi);
}

}
PLUGINLIB_DECLARE_CLASS(dragon_single_leg, VelocityController,
                        dragon_single_leg::VelocityController,
                        controller_interface::ControllerBase)
// PLUGINLIB_EXPORT_CLASS(dragon_control::ForwardJointGroupCommandController,controller_interface::ControllerBase)
