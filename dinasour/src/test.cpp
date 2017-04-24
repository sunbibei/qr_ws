#include "test.h"
#include <iostream>
#include <math.h>
#include <ros/ros.h>
void forward_kinematics();
void reverse_kinematics();
int Sgn(double a);
_Position get_stance_position(_Position Adj_vec, unsigned int Time_order);
_Position get_cross_point(_Position A_start, _Position A_end, _Position B_start, _Position B_end);
_Position get_innerheart(_Position A, _Position B, _Position C);
_Position get_CoG_adj_vec(_Position Next_Foothold,unsigned int Swing_Order);
_Position get_swing_pos(_Position Start_point, _Position End_point, unsigned int Time_order);

Angle Angle_Group = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
Position Foot_Position_Group = {{Body_Par[0][0]+10,Body_Par[0][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]+5},
                                {Body_Par[1][0],Body_Par[1][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]},
                                {Body_Par[2][0],Body_Par[2][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]},
                                {Body_Par[3][0],Body_Par[3][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]}};
Angle_Ptr Angle_ptr = &Angle_Group;
Position_Ptr Foot_pos_ptr = &Foot_Position_Group;
unsigned int Loop = 0;

_Position struct_assign(double x, double y, double z)
{
        _Position copy;
        copy.x = x;
        copy.y = y;
        copy.z = z;
        return copy;
}
using namespace std;
int main()
{

        std::cout<<Foot_pos_ptr->lf_foot.x<<"  "<<Foot_pos_ptr->lf_foot.y<<"  "<<Foot_pos_ptr->lf_foot.z<<" angle:"<<Angle_ptr->lf_angle.pitch<<"  "<<Angle_ptr->lf_angle.hip<<"  "<<Angle_ptr->lf_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->rf_foot.x<<"  "<<Foot_pos_ptr->rf_foot.y<<"  "<<Foot_pos_ptr->rf_foot.z<<" angle:"<<Angle_ptr->rf_angle.pitch<<"  "<<Angle_ptr->rf_angle.hip<<"  "<<Angle_ptr->rf_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->lb_foot.x<<"  "<<Foot_pos_ptr->lb_foot.y<<"  "<<Foot_pos_ptr->lb_foot.z<<" angle:"<<Angle_ptr->lb_angle.pitch<<"  "<<Angle_ptr->lb_angle.hip<<"  "<<Angle_ptr->lb_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->rb_foot.x<<"  "<<Foot_pos_ptr->rb_foot.y<<"  "<<Foot_pos_ptr->rb_foot.z<<" angle:"<<Angle_ptr->rb_angle.pitch<<"  "<<Angle_ptr->rb_angle.hip<<"  "<<Angle_ptr->rb_angle.knee<<std::endl;

        reverse_kinematics();
        std::cout<<Foot_pos_ptr->lf_foot.x<<"  "<<Foot_pos_ptr->lf_foot.y<<"  "<<Foot_pos_ptr->lf_foot.z<<" angle:"<<Angle_ptr->lf_angle.pitch<<"  "<<Angle_ptr->lf_angle.hip<<"  "<<Angle_ptr->lf_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->rf_foot.x<<"  "<<Foot_pos_ptr->rf_foot.y<<"  "<<Foot_pos_ptr->rf_foot.z<<" angle:"<<Angle_ptr->rf_angle.pitch<<"  "<<Angle_ptr->rf_angle.hip<<"  "<<Angle_ptr->rf_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->lb_foot.x<<"  "<<Foot_pos_ptr->lb_foot.y<<"  "<<Foot_pos_ptr->lb_foot.z<<" angle:"<<Angle_ptr->lb_angle.pitch<<"  "<<Angle_ptr->lb_angle.hip<<"  "<<Angle_ptr->lb_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->rb_foot.x<<"  "<<Foot_pos_ptr->rb_foot.y<<"  "<<Foot_pos_ptr->rb_foot.z<<" angle:"<<Angle_ptr->rb_angle.pitch<<"  "<<Angle_ptr->rb_angle.hip<<"  "<<Angle_ptr->rb_angle.knee<<std::endl;


        forward_kinematics();
        std::cout<<Foot_pos_ptr->lf_foot.x<<"  "<<Foot_pos_ptr->lf_foot.y<<"  "<<Foot_pos_ptr->lf_foot.z<<" angle:"<<Angle_ptr->lf_angle.pitch<<"  "<<Angle_ptr->lf_angle.hip<<"  "<<Angle_ptr->lf_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->rf_foot.x<<"  "<<Foot_pos_ptr->rf_foot.y<<"  "<<Foot_pos_ptr->rf_foot.z<<" angle:"<<Angle_ptr->rf_angle.pitch<<"  "<<Angle_ptr->rf_angle.hip<<"  "<<Angle_ptr->rf_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->lb_foot.x<<"  "<<Foot_pos_ptr->lb_foot.y<<"  "<<Foot_pos_ptr->lb_foot.z<<" angle:"<<Angle_ptr->lb_angle.pitch<<"  "<<Angle_ptr->lb_angle.hip<<"  "<<Angle_ptr->lb_angle.knee<<std::endl;
        std::cout<<Foot_pos_ptr->rb_foot.x<<"  "<<Foot_pos_ptr->rb_foot.y<<"  "<<Foot_pos_ptr->rb_foot.z<<" angle:"<<Angle_ptr->rb_angle.pitch<<"  "<<Angle_ptr->rb_angle.hip<<"  "<<Angle_ptr->rb_angle.knee<<std::endl;


        // _Position adj_vec;
        // _Position Next_Foothold = {Body_Par[0][0],Body_Par[0][1],-HEIGHT};
        //
        // Next_Foothold.x = Body_Par[0][0] + Foot_Steps / 4;
        //
        // adj_vec = get_CoG_adj_vec(Next_Foothold,1);
        //
        // std::cout<<"CoG Adj vector："<<adj_vec.x<<" "<<adj_vec.y<<std::endl;
        // _Position A={0,0,0};
        // _Position B ={1,-1,0};
        // _Position C={0,0,0};
        // _Position D ={-1,-1,0};
        // _Position cross_point;
        // cross_point = get_cross_point(A,B,C,D);
        // std::cout<<"cross_point: "<<cross_point.x<<" "<<cross_point.y<<std::endl;
        // Foot_pos_ptr->lf_foot = struct_assign(22.8072, 19.0043, -50);
        // cout<<Foot_pos_ptr->lf_foot.x<<endl;
        // _Position Start_point = {40.372, 13.5077,-49.8205};
        // _Position End_point = {11.8534,13.5,-50};
        // _Position adj;
        // while(Loop<Swing_Num)
        // {
        //         adj = get_swing_pos(Start_point, End_point,Loop);
        //         cout<<"Loop:"<<Loop<<",Adj:"<<adj.x<<","<<adj.y<<","<<adj.z<<endl;
        //         Loop++;
        // }

        return 0;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating leg position while swing, design swing foot trajectory,now using ellipse, need to work on in the future
   Input: start point,and desired point,and sample time order(1~)
   Output: innerheart position
**************************************************************************/
_Position get_swing_pos(_Position Start_point, _Position End_point, unsigned int Loop)
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

        // theta = last_order * PI / Swing_Num;
        // sgn = Sgn(End_point.x - Start_point.x);
        // a = fabs((End_point.x - Start_point.x) / 2.0);
        // swing_foot.x = swing_foot.x - a + sgn * a * cos(theta);
        // swing_foot.z = swing_foot.z - b * sin(theta);
        // sgn = Sgn(End_point.y - Start_point.y);
        // a = fabs((End_point.y - Start_point.y) / 2.0);
        // swing_foot.y = swing_foot.y - a + sgn * a * cos(theta);
        return swing_foot;
        // std::cout<<"Adj_Vector:"<<swing_foot.x<<"  "<<swing_foot.y<<"  "<<swing_foot.z<<std::endl;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating CoG position and CoG adjust vector,using next foothold,while 1 means to right side
   Input: four feet(end effector) position(x,y,z),Swing leg order and next Swing leg position
   Output: CoG adjust vector
**************************************************************************/
int Sgn(double a)
{
        if (a<0)
                return -1;
        else return 1;
}
_Position get_CoG_adj_vec(_Position Next_Foothold,unsigned int Swing_Order)
{
        _Position point = {0,0,0};

        switch (Swing_Order)
        {
        case 1:
                point = get_cross_point(Next_Foothold, Foot_pos_ptr->rb_foot, Foot_pos_ptr->rf_foot, Foot_pos_ptr->lb_foot);
                // std::cout<<"Cross_Point："<<point.x<<" "<<point.y<<std::endl;
                point = get_innerheart(point, Foot_pos_ptr->rf_foot, Foot_pos_ptr->rb_foot);
                // std::cout<<"innerheart："<<point.x<<" "<<point.y<<std::endl;
                break;
        case 2:
                point = get_cross_point(Next_Foothold, Foot_pos_ptr->lb_foot, Foot_pos_ptr->lf_foot, Foot_pos_ptr->rb_foot);
                point = get_innerheart(point, Foot_pos_ptr->lf_foot, Foot_pos_ptr->lb_foot);
                break;
        default: break;
        }

        point.z = 0; //Default CoG={0，0，-1 * HEIGHT}
        return point;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating line crosspoint position
   Input: four point position
   Output: crosspoint position
**************************************************************************/
_Position get_cross_point(_Position A_start, _Position A_end, _Position B_start, _Position B_end)
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
        // std::cout<<"k: "<<k1<<" "<<k2<<std::endl;
        // std::cout<<"flag: "<<flag_one<<" "<<flag_two<<std::endl;
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
                                cross_point.x =  (b1-b2)/(k2-k1);
                                cross_point.y = k1 * (b1-b2)/(k2-k1) + b1;
                        }
                        break;
                }
        default: break;
        }
        cross_point.z = -HEIGHT;
        return cross_point;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating triangle innerheart
   Input: three point position
   Output: innerheart position
**************************************************************************/
_Position get_innerheart(_Position A, _Position B, _Position C)
{
        double a = 0, b = 0, c = 0;
        _Position heart = {0,0,0};
        std::cout<<A.x<<" "<<A.y<<std::endl;
        std::cout<<B.x<<" "<<B.y<<std::endl;
        std::cout<<C.x<<" "<<C.y<<std::endl;

        a = sqrt(pow(B.x - C.x, 2) + pow(B.y - C.y, 2));
        b = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2));
        c = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));

        heart.x = (a * A.x + b * B.x + c * C.x ) / (a + b + c);
        heart.y = (a * A.y + b * B.y + c * C.y ) / (a + b + c);
        heart.z = -1 * HEIGHT;
        std::cout<<heart.x<<" "<<heart.y<<std::endl;
        return heart;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after knowing the exact distance to move CoG, design proper trajectory
   Input: CoG adjust Vector and Time_order(1~)
   Output: realtime position
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
    X-axis:
      Pos(t):6 * XD * t^5 / Stance_Num^5 - 15 * XD * t^4 / Stance_Num^4 + 10 * XD * t^3 / Stance_Num^3
    Y-axis:
      Pos(t):6 * YD * t^5 / Stance_Num^5 - 15 * YD * t^4 / Stance_Num^4 + 10 * YD * t^3 / Stance_Num^3
    Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position get_stance_position(_Position Adj_vec, unsigned int Time_order)
{
        if(Time_order>Stance_Num)
        {

        }

        unsigned int last_order = Time_order - 1;
        if(Time_order==0)
        {
                last_order = 0;
        }

        _Position stance_adj = {0,0,0};
        stance_adj.x = 6 * Adj_vec.x * pow(Time_order,5) / pow(Stance_Num,5) - 15 * Adj_vec.x * pow(Time_order,4) / pow(Stance_Num,4)
                       + 10 * Adj_vec.x * pow(Time_order,3) / pow(Stance_Num,3);
        stance_adj.y = 6 * Adj_vec.y * pow(Time_order,5) / pow(Stance_Num,5) - 15 * Adj_vec.y * pow(Time_order,4) / pow(Stance_Num,4)
                       + 10 * Adj_vec.y * pow(Time_order,3) / pow(Stance_Num,3);

        stance_adj.x = stance_adj.x - (6 * Adj_vec.x * pow(last_order,5) / pow(Stance_Num,5) - 15 * Adj_vec.x * pow(last_order,4) / pow(Stance_Num,4)
                                       + 10 * Adj_vec.x * pow(last_order,3) / pow(Stance_Num,3));
        stance_adj.y = stance_adj.y - (6 * Adj_vec.y * pow(last_order,5) / pow(Stance_Num,5) - 15 * Adj_vec.y * pow(last_order,4) / pow(Stance_Num,4)
                                       + 10 * Adj_vec.y * pow(last_order,3) / pow(Stance_Num,3));

        return stance_adj;

}
void forward_kinematics()
{
        Foot_pos_ptr->lf_foot.x = Body_Par[0][0] + F_Leg_Length[1] * sin(Angle_ptr->lf_angle.hip)
                                  + F_Leg_Length[2] * sin(Angle_ptr->lf_angle.hip + Angle_ptr->lf_angle.knee);
        Foot_pos_ptr->lf_foot.y = Body_Par[0][1] + F_Leg_Length[0] * sin(Angle_ptr->lf_angle.pitch)
                                  + F_Leg_Length[1] * sin(Angle_ptr->lf_angle.pitch) * cos(Angle_ptr->lf_angle.hip)
                                  + F_Leg_Length[2] * sin(Angle_ptr->lf_angle.pitch) * cos(Angle_ptr->lf_angle.hip
                                                                                           + Angle_ptr->lf_angle.knee);
        Foot_pos_ptr->lf_foot.z = Body_Par[0][2] - F_Leg_Length[0] * cos(Angle_ptr->lf_angle.pitch)
                                  - F_Leg_Length[1] * cos(Angle_ptr->lf_angle.pitch) * cos(Angle_ptr->lf_angle.hip)
                                  - F_Leg_Length[2] * cos(Angle_ptr->lf_angle.pitch) * cos(Angle_ptr->lf_angle.hip
                                                                                           + Angle_ptr->lf_angle.knee);


        Foot_pos_ptr->rf_foot.x = Body_Par[1][0] + F_Leg_Length[1] * sin(Angle_ptr->rf_angle.hip)
                                  + F_Leg_Length[2] * sin(Angle_ptr->rf_angle.hip + Angle_ptr->rf_angle.knee);
        Foot_pos_ptr->rf_foot.y = Body_Par[1][1] + F_Leg_Length[0] * sin(Angle_ptr->rf_angle.pitch)
                                  + F_Leg_Length[1] * sin(Angle_ptr->rf_angle.pitch) * cos(Angle_ptr->rf_angle.hip)
                                  + F_Leg_Length[2] * sin(Angle_ptr->rf_angle.pitch) * cos(Angle_ptr->rf_angle.hip
                                                                                           + Angle_ptr->rf_angle.knee);
        Foot_pos_ptr->rf_foot.z = Body_Par[1][2] - F_Leg_Length[0] * cos(Angle_ptr->rf_angle.pitch)
                                  - F_Leg_Length[1] * cos(Angle_ptr->rf_angle.pitch) * cos(Angle_ptr->rf_angle.hip)
                                  - F_Leg_Length[2] * cos(Angle_ptr->rf_angle.pitch) * cos(Angle_ptr->rf_angle.hip
                                                                                           + Angle_ptr->rf_angle.knee);


        Foot_pos_ptr->lb_foot.x = Body_Par[2][0] + B_Leg_Length[1] * sin(Angle_ptr->lb_angle.hip)
                                  + B_Leg_Length[2] * sin(Angle_ptr->lb_angle.hip + Angle_ptr->lb_angle.knee);
        Foot_pos_ptr->lb_foot.y = Body_Par[2][1] + B_Leg_Length[0] * sin(Angle_ptr->lb_angle.pitch)
                                  + B_Leg_Length[1] * sin(Angle_ptr->lb_angle.pitch) * cos(Angle_ptr->lb_angle.hip)
                                  + B_Leg_Length[2] * sin(Angle_ptr->lb_angle.pitch) * cos(Angle_ptr->lb_angle.hip
                                                                                           + Angle_ptr->lb_angle.knee);
        Foot_pos_ptr->lb_foot.z = Body_Par[2][2] - B_Leg_Length[0] * cos(Angle_ptr->lb_angle.pitch)
                                  - B_Leg_Length[1] * cos(Angle_ptr->lb_angle.pitch) * cos(Angle_ptr->lb_angle.hip)
                                  - B_Leg_Length[2] * cos(Angle_ptr->lb_angle.pitch) * cos(Angle_ptr->lb_angle.hip
                                                                                           + Angle_ptr->lb_angle.knee);


        Foot_pos_ptr->rb_foot.x = Body_Par[3][0] + B_Leg_Length[1] * sin(Angle_ptr->rb_angle.hip)
                                  + B_Leg_Length[2] * sin(Angle_ptr->rb_angle.hip + Angle_ptr->rb_angle.knee);
        Foot_pos_ptr->rb_foot.y = Body_Par[3][1] + B_Leg_Length[0] * sin(Angle_ptr->rb_angle.pitch)
                                  + B_Leg_Length[1] * sin(Angle_ptr->rb_angle.pitch) * cos(Angle_ptr->rb_angle.hip)
                                  + B_Leg_Length[2] * sin(Angle_ptr->rb_angle.pitch) * cos(Angle_ptr->rb_angle.hip + Angle_ptr->rb_angle.knee);
        Foot_pos_ptr->rb_foot.z = Body_Par[3][2] - B_Leg_Length[0] * cos(Angle_ptr->rb_angle.pitch)
                                  - B_Leg_Length[1] * cos(Angle_ptr->rb_angle.pitch) * cos(Angle_ptr->rb_angle.hip)
                                  - B_Leg_Length[2] * cos(Angle_ptr->rb_angle.pitch) * cos(Angle_ptr->rb_angle.hip + Angle_ptr->rb_angle.knee);

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
void reverse_kinematics()
{
        //front
        double Delte = Foot_pos_ptr->lf_foot.x - Body_Par[0][0];
        Angle_ptr->lf_angle.pitch = atan((Body_Par[0][1] - Foot_pos_ptr->lf_foot.y) / (Foot_pos_ptr->lf_foot.z - Body_Par[0][2]));
        double Epsilon = F_Leg_Length[0] + Foot_pos_ptr->lf_foot.z * cos(Angle_ptr->lf_angle.pitch) - Body_Par[0][2] * cos(Angle_ptr->lf_angle.pitch)
                         - Foot_pos_ptr->lf_foot.y * sin(Angle_ptr->lf_angle.pitch) + Body_Par[0][1] * sin(Angle_ptr->lf_angle.pitch);
        Angle_ptr->lf_angle.knee = -1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(F_Leg_Length[1],2) - pow(F_Leg_Length[2],2))
                                             / 2.0 / F_Leg_Length[1] / F_Leg_Length[2]);
        double Phi = Delte + F_Leg_Length[2] * sin(Angle_ptr->lf_angle.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        Angle_ptr->lf_angle.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (F_Leg_Length[2] * sin(Angle_ptr->lf_angle.knee) - Delte))) / Phi);


        Delte = Foot_pos_ptr->rf_foot.x - Body_Par[1][0];
        Angle_ptr->rf_angle.pitch = atan((Body_Par[1][1] - Foot_pos_ptr->rf_foot.y) / (Foot_pos_ptr->rf_foot.z - Body_Par[1][2]));
        Epsilon = F_Leg_Length[0] + Foot_pos_ptr->rf_foot.z * cos(Angle_ptr->rf_angle.pitch) - Body_Par[1][2] * cos(Angle_ptr->rf_angle.pitch)
                  - Foot_pos_ptr->rf_foot.y * sin(Angle_ptr->rf_angle.pitch) + Body_Par[1][1] * sin(Angle_ptr->rf_angle.pitch);
        Angle_ptr->rf_angle.knee = -1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(F_Leg_Length[1],2) - pow(F_Leg_Length[2],2))
                                             / 2.0 / F_Leg_Length[1] / F_Leg_Length[2]);
        Phi = Delte + F_Leg_Length[2] * sin(Angle_ptr->rf_angle.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        Angle_ptr->rf_angle.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (F_Leg_Length[2] * sin(Angle_ptr->rf_angle.knee) - Delte))) / Phi);

        //back
        Delte = Foot_pos_ptr->lb_foot.x - Body_Par[2][0];
        Angle_ptr->lb_angle.pitch = atan((Body_Par[2][1] - Foot_pos_ptr->lb_foot.y) / (Foot_pos_ptr->lb_foot.z - Body_Par[2][2]));

        Epsilon = B_Leg_Length[0] + Foot_pos_ptr->lb_foot.z * cos(Angle_ptr->lb_angle.pitch) - Body_Par[2][2] * cos(Angle_ptr->lb_angle.pitch)
                  - Foot_pos_ptr->lb_foot.y * sin(Angle_ptr->lb_angle.pitch) + Body_Par[2][1] * sin(Angle_ptr->lb_angle.pitch);
        Angle_ptr->lb_angle.knee = 1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(B_Leg_Length[1],2) - pow(B_Leg_Length[2],2))
                                            / 2.0 / B_Leg_Length[1] / B_Leg_Length[2]);
        Phi = Delte + B_Leg_Length[2] * sin(Angle_ptr->lb_angle.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        Angle_ptr->lb_angle.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (B_Leg_Length[2] * sin(Angle_ptr->lb_angle.knee) - Delte))) / Phi);


        Delte = Foot_pos_ptr->rb_foot.x - Body_Par[3][0];
        Angle_ptr->rb_angle.pitch = atan((Body_Par[3][1] - Foot_pos_ptr->rb_foot.y) / (Foot_pos_ptr->rb_foot.z - Body_Par[3][2]));
        Epsilon = B_Leg_Length[0] + Foot_pos_ptr->rb_foot.z * cos(Angle_ptr->rb_angle.pitch) - Body_Par[3][2] * cos(Angle_ptr->rb_angle.pitch)
                  - Foot_pos_ptr->rb_foot.y * sin(Angle_ptr->rb_angle.pitch) + Body_Par[3][1] * sin(Angle_ptr->rb_angle.pitch);
        Angle_ptr->rb_angle.knee = 1 * acos((pow(Delte,2) + pow(Epsilon,2) - pow(B_Leg_Length[1],2) - pow(B_Leg_Length[2],2))
                                            / 2.0 / B_Leg_Length[1] / B_Leg_Length[2]);
        Phi = Delte + B_Leg_Length[2] * sin(Angle_ptr->rb_angle.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        Angle_ptr->rb_angle.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (B_Leg_Length[2] * sin(Angle_ptr->rb_angle.knee) - Delte))) / Phi);

}
