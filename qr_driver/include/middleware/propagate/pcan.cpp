/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include "pcan.h"

namespace middleware {

PcanChannel::PcanChannel(const std::string& name)
  :Propagate(name)
{ }

PcanChannel::~PcanChannel() { }

// 完成PCAN的初始化
// 以及act_state_map_, act_cmd_map_, enc_state_map_三个MAP的从cmd_map_和state_map_中初始化
bool PcanChannel::init() {
  return true;
}

void PcanChannel::stop() {
  return;
}

// 完成数据的读写. (下面全是测试代码)
bool PcanChannel::write(const std::vector<std::string>& names) {
  if (names.empty()) return true;

  // LOG_INFO << "PCAN write: ";
  for (const std::string& name : names) {
    auto itr = cmd_composite_.find(name);
    if (cmd_composite_.end() != itr) {
      Motor::CmdTypeSp cmd = boost::dynamic_pointer_cast<Motor::CmdType>(itr->second);
      LOG_INFO << "command: " << cmd->command_
          << " mode: " << cmd->mode_;
      if (0 == name.compare("hip")) {
        // std::string enc_name = "hip_motor";
        // auto itr_state = state_composite_.find(enc_name);
        auto itr_state = state_composite_.find(name);
        Encoder::StateTypeSp act_state
          = boost::dynamic_pointer_cast<Encoder::StateType>(itr_state->second);

        double current_pos = cmd->command_;
        auto current_time = std::chrono::high_resolution_clock::now();
        act_state->vel_ = (current_pos - act_state->pos_)
            / std::chrono::duration_cast<std::chrono::duration<double>>(
                current_time - act_state->previous_time_).count();
        act_state->pos_ = current_pos;
        act_state->previous_time_ = current_time;

      } else if (0 == name.compare("knee")) {
        // std::string enc_name = "knee_motor";
        // auto itr_state = state_composite_.find(enc_name);
        auto itr_state = state_composite_.find(name);
        Encoder::StateTypeSp act_state
          = boost::dynamic_pointer_cast<Encoder::StateType>(itr_state->second);

        double current_pos = cmd->command_;
        auto current_time = std::chrono::high_resolution_clock::now();
        act_state->vel_ = (current_pos - act_state->pos_)
            / std::chrono::duration_cast<std::chrono::duration<double>>(
                current_time - act_state->previous_time_).count();
        act_state->pos_ = current_pos;
        act_state->previous_time_ = current_time;
      } else {
        ; // Nothing to de here
      }
    } else {
      LOG_WARNING << "Could not found the " << name << " command handle";
    }
  }
  return true;
}

// (下面全是测试代码)
bool PcanChannel::read() {
  // 从PCAN中获取到的数据对应到具体的状态name
  // 也可以从PCAN的数据中， 明确到底是什么类型的State
  // 转化为对应类型的State, 在进行赋值
  // LOG_INFO << "PCAN read: ";
  std::string name = "knee";
  auto itr = state_composite_.find(name);
  if (state_composite_.end() == itr) {
    LOG_WARNING << "Could not found the " << name << " state handle: ";
  } else {
    Encoder::StateTypeSp act_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
    double current_pos = act_state->pos_ + 0.00001;
    auto current_time = std::chrono::high_resolution_clock::now();
    act_state->vel_ = (current_pos - act_state->pos_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - act_state->previous_time_).count();
    act_state->pos_ = current_pos;
    act_state->previous_time_ = current_time;
  }

  name = "hip";
  itr = state_composite_.find(name);
  if (state_composite_.end() == itr) {
    ;//LOG_WARNING << "Could not found the " << name << " state handle: ";
  } else {
    Encoder::StateTypeSp act_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
    double current_pos = act_state->pos_ + 0.0000001;
    auto current_time = std::chrono::high_resolution_clock::now();
    act_state->vel_ = (current_pos - act_state->pos_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - act_state->previous_time_).count();
    act_state->pos_ = current_pos;
    act_state->previous_time_ = current_time;
  }

  return true;
}

void PcanChannel::check() {
  LOG_WARNING << "================check================";
  LOG_INFO << "NAME: " << name_;
  LOG_WARNING << "-------------------------------------";
  LOG_INFO << "STATE:";
  LOG_INFO << "NAME\tADDR\tCOUNT";
  for (auto& s : state_composite_) {
    LOG_INFO << s.first << "\t" << s.second.get()
        << "\t" << s.second.use_count();
  }
  LOG_WARNING << "-------------------------------------";
  LOG_INFO << "COMMAND:";
  LOG_INFO << "NAME\tADDR\tCOUNT";
  for (auto& c : cmd_composite_) {
    LOG_INFO << c.first << "\t" << c.second.get() << "\t" << c.second.use_count();
  }
  LOG_WARNING << "=====================================";
}

} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Propagate)
