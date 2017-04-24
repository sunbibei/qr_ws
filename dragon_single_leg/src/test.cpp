#include "test.h"
#include <iostream>
#include <math.h>


void forward_kinematics();
void reverse_kinematics();
void print();
using namespace std;

_Position Pos = {0,0,0};
_Angle Angle = {0,0,0};
int Swing_Num = 10;
int Stance_Num = 10;
int Swing_Height = 10;
int Height = 55;
int Sgn(int a)
{
        return (a>0 ? 1 : -1);
}
_Position get_eclipse_pos(_Position Start_point, _Position End_point,int Loop)
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
_Position get_stance_position(_Position Start_point,_Position End_point,  int Loop)
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
float get_adj_pos(float Adj, int t, int T)
{
        float pos = 6 * Adj * pow(t,5) / pow(T,5) - 15 * Adj * pow(t,4) / pow(T,4)
                    + 10 * Adj * pow(t,3) / pow(T,3);
        return pos;
}
int main()
{
        // int T=1000;
        // int t = 0;
        // while(1)
        // {
        //         float adj = fabs(l0 + l1 + l2 - Height);//TODO
        //         int T = 1000;
        //
        //         if(fabs(Pos.z + Height)<0.001 || t>T)//TODO
        //         {
        //                 std::cout<<"reset done!"<<std::endl;
        //                 break;
        //         }
        /****************************************/
        // Angle.pitch = joints_[0].getPosition();//TODO--> may need trans
        // Angle.hip   = joints_[1].getPosition();
        // Angle.knee  = joints_[2].getPosition();
        _Position Start_point = {0,0,-Height};
        _Position End_point = {10,0,-Height};
        _Position adj;
        for(int i=0; i<10; i++)
        {
                adj = get_stance_position(Start_point, End_point,  i+1);
                // forward_kinematics();
                // cout<<get_adj_pos(adj, t, T)<<endl;
                Pos.x = Start_point.x + adj.x;
                Pos.y = Start_point.y + adj.y;
                Pos.z =  adj.z;
                reverse_kinematics();
                print();
        }


        return 0;
}



void forward_kinematics()
{
        Pos.x = l1 * sin(Angle.hip) + l2 * sin(Angle.hip + Angle.knee);
        Pos.y = l0 * sin(Angle.pitch) + l1 * sin(Angle.pitch) * cos(Angle.hip) + l2 * sin(Angle.pitch) * cos(Angle.hip + Angle.knee);
        Pos.z = -l0 * cos(Angle.pitch) - l1 * cos(Angle.pitch) * cos(Angle.hip) - l2 * cos(Angle.pitch) * cos(Angle.hip + Angle.knee);
}

void reverse_kinematics()
{
        Angle.pitch = atan(-Pos.y / Pos.z);

        double Epsilon = l0 + Pos.z * cos(Angle.pitch) - Pos.y * sin(Angle.pitch);
        Angle.knee = -1 * acos((pow(Pos.x,2) + pow(Epsilon,2) - pow(l1,2) - pow(l2,2)) / 2.0 / l1 / l2);

        double Phi = Pos.x + l2 * sin(Angle.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        Angle.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (l2 * sin(Angle.knee) - Pos.x))) / Phi);
}
void print()
{
        cout<<"Angle(pitch,hip,knee):"<<Angle.hip<<endl;
        // cout<<"Position(x,y,z):"<<Pos.x<<","<<Pos.y<<","<<Pos.z<<endl;
}
