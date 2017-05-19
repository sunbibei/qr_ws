#pragma once
#include <string>
#include <iostream>

const  int Joint_Num = 12;
const float PI = 3.14159265;
const int Height = 35;

const  int Foot_Steps = 10;
const  int Swing_Height = 4;

const  int Init_Num = 200;
const  int Swing_Num  = 100;
const  int Stance_Num = 100;
const  int Update_Rate = 1000;

const double L0 = 4;
const double L1 = 27.3;
const double L2 = 22.5;

const double Body_L = 27.65;
const double Body_W = 16.8;
const double Body_D = 10;

inline int Sgn(int a)
{
        return (a>0 ? 1 : -1);
}
template < typename T > std::string to_string( const T& n )
{
        std::ostringstream stm;
        stm << n;
        return stm.str();
}
struct Commands
{
        double position_; // Last commanded position
        double velocity_; // Last commanded velocity
        bool has_velocity_; // false if no velocity command has been specified
};

struct _Position
{
        double x;
        double y;
        double z;
};
struct _Angle_Leg
{
        double pitch;
        double hip;
        double knee;
};

typedef struct Position
{
        _Position lf;
        _Position rf;
        _Position lb;
        _Position rb;

}Position,*Position_Ptr;
typedef struct Angle
{
        _Angle_Leg lf;
        _Angle_Leg rf;
        _Angle_Leg lb;
        _Angle_Leg rb;
}Angle,*Angle_Ptr;
