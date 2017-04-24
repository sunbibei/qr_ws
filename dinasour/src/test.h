#pragma once
#include <string>
#include <iostream>

#define LEG_NUM 4
#define DOF 3
#define SPACE 3
#define Min_Angle_Error 0.05
#define Min_Pos_Error 0.05
#define PI 3.14159265
#define HEIGHT 50
#define Velocity 50 // cm/s
#define Update_Rate 1000
#define Foot_Steps 35
#define Swing_Height 10
#define Body_Acc_Max 5
#define Body_vel_Max 5

const unsigned int Swing_Num  = Update_Rate * Foot_Steps / Velocity /2;
const unsigned int Stance_Num = Update_Rate * Foot_Steps / Velocity /2;

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

const double F_Leg_Length[] = {0, 27.5, 23};
const double B_Leg_Length[] = {0, 29, 28};
const double Body_Par[4][3] = {{21.8534, 13.5, -6.5}, {21.8534, -13.5, -6.5}, {-30.6466, 16.5, 0}, {-30.6466, -16.5, 0}};
