#pragma once
#include <string>
#include <iostream>
#define DOF 3
#define SPACE 3

#define PI 3.14159265
#define l0 5.0
#define l1 30.0
#define l2 30.0

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
struct _Angle
{
        double pitch;
        double hip;
        double knee;
};
