#pragma once
#include <string>
#include <iostream>

const unsigned int Joint_Num = 12;
const float PI = 3.14159265;
const unsigned int Height = 50;

const unsigned int Foot_Steps = 10;
const unsigned int Swing_Height = 5;

const unsigned int Init_Num = 1000;
const unsigned int Swing_Num  = 1000;
const unsigned int Stance_Num = 1000;
const unsigned int Update_Rate = 1000;

const double L0 = 4;
const double L1 = 27.3;
const double L2 = 22.5;


const double Body_L = 20;
const double Body_W = 13.5;


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
