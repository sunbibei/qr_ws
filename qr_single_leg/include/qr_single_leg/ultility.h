#pragma once
#include <string>
#include <iostream>
#define DOF 3
#define SPACE 3

#define PI 3.14159265
#define l0 4
#define l1 27.3
#define l2 22.5
#define Height 40
#define Swing_Num 1000
#define Stance_Num 1000
#define Reset_Num 1000
#define Swing_Height 5

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
struct _Angle
{
        double pitch;
        double hip;
        double knee;
};
struct _Velocity
{
        double pitch;
        double hip;
        double knee;
};
