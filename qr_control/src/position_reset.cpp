#include "qr_control/position_reset.h"

namespace qr_control {

PositionResetController::PositionResetController() {
}

PositionResetController::~PositionResetController() {

}

/**************************************************************************
   Description: initialize joints from robot_description
**************************************************************************/
bool PositionResetController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {
    int n_joints = 0;
    while(true) {
        std::string joint_name;
        std::string param_name = std::string("joint_" + std::to_string(n_joints));
        if (ros::param::get(param_name.c_str(), joint_name)) {
            std::cout << "Get Joint Name: " << joint_name;
            joint_handles_.push_back(robot->getHandle(joint_name));
        } else {
            break;
        }
        ++n_joints;
    }

    return true;
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: set model initial state
**************************************************************************/
void PositionResetController::starting(const ros::Time& time) {

}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void PositionResetController::update(const ros::Time&, const ros::Duration&) {
    for(auto iter = joint_handles_.begin();
        iter != joint_handles_.end();
        ++iter) {
        // if (joint_handles_.getPosition() != 0) {

        // }
        std::cout << iter->getPosition();
    }
    std::cout << std::endl;

}


} /* end for namespace qr_control */

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(qr_control, PositionResetController,
                        qr_control::PositionResetController,
                        controller_interface::ControllerBase)