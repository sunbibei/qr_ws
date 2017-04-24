#pragma once
#include <string>
#include <iostream>

const unsigned int LEG_NUM 4
const unsigned int DOF 3
const unsigned int SPACE 3

const unsigned float PI 3.14159265
const unsigned int HEIGHT 50
const unsigned int Velocity 5 // cm/s
const unsigned int Update_Rate 1000
const unsigned int Foot_Steps 35
const unsigned int Swing_Height 10
const unsigned int Body_Acc_Max 5
const unsigned int Body_vel_Max 5

const unsigned int Swing_Num  = Update_Rate * Foot_Steps / Velocity /2;
const unsigned int Stance_Num = Update_Rate * Foot_Steps / Velocity /2;

const double F_Leg_Length[] = {0, 27.5, 23};
const double B_Leg_Length[] = {0, 29, 28};
const double Body_Par[4][3] = {{21.8534, 13.5, -6.5}, {21.8534, -13.5, -6.5}, {-30.6466, 16.5, 0}, {-30.6466, -16.5, 0}};

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
        _Position lf_foot;
        _Position rf_foot;
        _Position lb_foot;
        _Position rb_foot;

}Position,*Position_Ptr;
typedef struct Angle
{
        _Angle_Leg lf_angle;
        _Angle_Leg rf_angle;
        _Angle_Leg lb_angle;
        _Angle_Leg rb_angle;
}Angle,*Angle_Ptr;
