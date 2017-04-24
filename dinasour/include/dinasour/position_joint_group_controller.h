#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/node_handle.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <urdf/model.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include "dinasour/ultility.h"

namespace dinasour
{
        class PositionJointGroupController: public controller_interface::Controller<hardware_interface::PositionJointInterface>
        {
public:
                PositionJointGroupController();
                ~PositionJointGroupController();

                bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
                void starting(const ros::Time& time);
                void update(const ros::Time& time, const ros::Duration& period);

                unsigned int n_joints_;
                std::vector<double> commands;
                std_msgs::Float64MultiArray msg;
                std::vector< std::string > joint_names_;
                std::vector< hardware_interface::JointHandle > joints_;
                realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;

private:
                int Loop_Count;
                unsigned int Time_Order = 0;
                unsigned int Leg_Order = 1;
                Angle Angle_Group = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
                Position Foot_Position_Group = {{Body_Par[0][0],Body_Par[0][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]},
                                                {Body_Par[1][0],Body_Par[1][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]},
                                                {Body_Par[2][0],Body_Par[2][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]},
                                                {Body_Par[3][0],Body_Par[3][1],-B_Leg_Length[0] - B_Leg_Length[1] - B_Leg_Length[2]}};

                Angle_Ptr Angle_ptr = &Angle_Group;
                Position_Ptr Foot_pos_ptr = &Foot_Position_Group;
                _Position Desired_Foot_Pos = {0,0,0};
                _Position Pos_start,Cog_adj;
                ros::Subscriber sub_command_;
                void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
                void forward_kinematics();
                void reverse_kinematics();
                void pose_init();
                void ISM(unsigned int Loop_count);
                int Sgn(double a);
                std::vector<double> vec_assign(Angle_Ptr Angle);


                _Position get_stance_position(_Position Adj_vec, unsigned int Loop);
                _Position get_stance_velocity(_Position Adj_vec, unsigned int Loop);
                _Position get_stance_acceration(_Position Adj_vec, unsigned int Loop);
                _Position get_swing_pos(_Position Start_point, _Position End_point, unsigned int Loop);
                _Position get_CoG_adj_vec(_Position Next_Foothold, unsigned int Swing_Order);
                _Position get_cross_point(_Position A_start, _Position A_end, _Position B_start, _Position B_end);
                _Position get_innerheart(_Position A, _Position B, _Position C);
                _Position struct_copy(_Position A);
                _Position struct_assign(double x, double y, double z);
        };

}
