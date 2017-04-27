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
#include "qr_single_leg/ultility.h"

namespace qr_single_leg
{
        class PositionController: public controller_interface::Controller<hardware_interface::PositionJointInterface>
        {
public:
                PositionController();
                ~PositionController();

                bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
                void starting(const ros::Time& time);
                void update(const ros::Time& time, const ros::Duration& period);

                unsigned int n_joints_;
                std::vector<double> commands;
                std_msgs::Float64MultiArray msg;
                std::vector< std::string > joint_names_;
                std::vector< hardware_interface::JointHandle > joints_;
                boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;

private:
                int Loop_Count;
                int Phase;
                _Position Pos;
                _Angle Angle;

                void forward_kinematics();
                void reverse_kinematics();
                void reset();
                void reset(_Position Start_point, _Position End_point);
                void updown(_Position Start_point, _Position End_point);
                void sas(_Position Start_point, _Position End_point);
                float get_adj_pos(float Adj, int t, int T);
                float get_adj_vel(float Adj, int t, int T);
                float get_adj_acc(float Adj, int t, int T);
                void loop_control();
                _Position get_eclipse_pos(_Position Start_point, _Position End_point,int Loop);
                _Position get_stance_position(_Position Start_point, _Position End_point,int Loop);

        };

}
