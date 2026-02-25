//
// Created by longyu on 9/21/25.
//

#include "rm_manual/series_legged_manual.h"

namespace rm_manual
{
SeriesLeggedManual::SeriesLeggedManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : BalanceManual(nh, nh_referee)
{
  ros::NodeHandle leg_wheel_chassis_nh(nh, "balance/legged_wheel_chassis");
  legCommandSender_ = new rm_common::LegCommandSender(leg_wheel_chassis_nh);

  double short_leg_len{}, mid_leg_length{}, high_leg_len{};
  leg_wheel_chassis_nh.param("short_leg_length", short_leg_len, 0.2);
  leg_wheel_chassis_nh.param("mid_leg_length", mid_leg_length, 0.32);
  leg_wheel_chassis_nh.param("high_leg_length", high_leg_len, 0.36);
  leg_len_map_.emplace(SHORT, short_leg_len);
  leg_len_map_.emplace(MID, mid_leg_length);
  leg_len_map_.emplace(HIGH, high_leg_len);
  legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
  legCommandSender_->setJump(false);

  b_event_.setEdge(boost::bind(&SeriesLeggedManual::bPress, this), boost::bind(&SeriesLeggedManual::bRelease, this));
  r_event_.setEdge(boost::bind(&SeriesLeggedManual::rPress, this), boost::bind(&SeriesLeggedManual::rRelease, this));
  r_event_.setActiveHigh(boost::bind(&SeriesLeggedManual::rPressing, this));
  ctrl_g_event_.setRising(boost::bind(&SeriesLeggedManual::ctrlGPress, this));
  ctrl_w_event_.setActiveHigh(boost::bind(&SeriesLeggedManual::ctrlWPressing, this));
  std::string unstick_topic, upstair_status_topic, legged_chassis_mode_topic, tof_topic;
  leg_wheel_chassis_nh.param("unstick_topic", unstick_topic,
                             std::string("/controllers/legged_balance_controller/unstick/two_leg_unstick"));
  leg_wheel_chassis_nh.param("upstair_status_topic", upstair_status_topic,
                             std::string("/controllers/chassis_controller/upstair_status"));
  leg_wheel_chassis_nh.param("legged_chassis_mode_topic", legged_chassis_mode_topic,
                             std::string("/controllers/chassis_controller/legged_chassis_mode"));
  leg_wheel_chassis_nh.param("tof_topic", tof_topic, std::string("/tof"));
  unstick_sub_ =
      leg_wheel_chassis_nh.subscribe<std_msgs::Bool>(unstick_topic, 1, &SeriesLeggedManual::unstickCallback, this);
  leg_len_status_sub_ = leg_wheel_chassis_nh.subscribe<rm_msgs::LeggedUpstairStatus>(
      upstair_status_topic, 1, &SeriesLeggedManual::upstairStatusCallback, this);
  legged_chassis_mode_sub_ = leg_wheel_chassis_nh.subscribe<rm_msgs::LeggedChassisMode>(
      legged_chassis_mode_topic, 1, &SeriesLeggedManual::leggedChassisModeCallback, this);
  left_tof_sensor_sub_ = leg_wheel_chassis_nh.subscribe<sensor_msgs::Range>(
      tof_topic + "/left_tof_link", 10, &SeriesLeggedManual::tofSensorMsgCallback, this);
  right_tof_sensor_sub_ = leg_wheel_chassis_nh.subscribe<sensor_msgs::Range>(
      tof_topic + "/right_tof_link", 10, &SeriesLeggedManual::tofSensorMsgCallback, this);
}

void SeriesLeggedManual::sendCommand(const ros::Time& time)
{
  BalanceManual::sendCommand(time);
  //  if (is_gyro_)
  //  {
  //    double current_length = legCommandSender_->getLgeLength();
  //    if (is_increasing_length_)
  //    {
  //      if (current_length < 0.3)
  //      {
  //        double delta = current_length + 0.002;
  //        legCommandSender_->setLgeLength(delta > 0.3 ? 0.3 : delta);
  //      }
  //      else
  //        is_increasing_length_ = false;
  //    }
  //    else
  //    {
  //      if (current_length > 0.18)
  //      {
  //        double delta = current_length - 0.002;
  //        legCommandSender_->setLgeLength(delta < 0.18 ? 0.18 : delta);
  //      }
  //      else
  //        is_increasing_length_ = true;
  //    }
  //  }
  //  else
  //  {
  //    legCommandSender_->setLgeLength(0.18);
  //  }
  legCommandSender_->sendCommand(time);
}

void SeriesLeggedManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  ctrl_g_event_.update(dbus_data->key_ctrl && dbus_data->key_g);
  ctrl_event_.update(dbus_data->key_ctrl);
  ctrl_w_event_.update(dbus_data->key_ctrl && dbus_data->key_w && !dbus_data->key_g);
}

void SeriesLeggedManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  BalanceManual::updateRc(dbus_data);
  if (!is_gyro_)
  {  // Capacitor enter fast charge when chassis stop.
    if (!dbus_data->wheel && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    else if (chassis_power_ < 6.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
  else
  {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}

void SeriesLeggedManual::rightSwitchDownRise()
{
  BalanceManual::rightSwitchDownRise();
}

void SeriesLeggedManual::rightSwitchMidRise()
{
  BalanceManual::rightSwitchMidRise();
}

void SeriesLeggedManual::ctrlZPress()
{
  BalanceManual::ctrlZPress();
  if (!supply_)
  {
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    leg_len_status_ = SHORT;
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
  }
}

void SeriesLeggedManual::shiftRelease()
{
  BalanceManual::shiftRelease();
}

void SeriesLeggedManual::shiftPress()
{
  BalanceManual::shiftPress();
}

void SeriesLeggedManual::rPress()
{
  if (!stretch_)
  {
    upstair_leg_len_fsm_ = 1;
    leg_len_status_ = HIGH;
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
    stretch_ = true;
  }
  else
  {
    upstair_leg_len_fsm_ = 0;
    leg_len_status_ = SHORT;
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
    stretch_ = false;
  }
  //  if (total_tof_len_ < 1.0)
  //  {
  //    legCommandSender_->setJump(true);
  //  }
}

void SeriesLeggedManual::rPressing()
{
  //  if (total_tof_len_ < 1.0)
  //  {
  //    legCommandSender_->setJump(true);
  //  }
}

void SeriesLeggedManual::rRelease()
{
  legCommandSender_->setJump(false);
}

void SeriesLeggedManual::bPress()
{
  ChassisGimbalShooterCoverManual::bPress();
  chassis_cmd_sender_->updateSafetyPower(60);
}

void SeriesLeggedManual::bRelease()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  chassis_cmd_sender_->updateSafetyPower(60);
}

void SeriesLeggedManual::ctrlGPress()
{
  if (!stretch_)
  {
    upstair_leg_len_fsm_ = 1;
    leg_len_status_ = HIGH;
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
    stretch_ = true;
  }
  else
  {
    upstair_leg_len_fsm_ = 0;
    leg_len_status_ = SHORT;
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
    stretch_ = false;
  }
}

void SeriesLeggedManual::ctrlWPressing()
{
  if ((ros::Time::now() - last_upstairs_time_) > ros::Duration(2.0))
  {
    if (total_tof_len_ < 0.4)
    {
      last_upstairs_time_ = ros::Time::now();
      leg_len_status_ = HIGH;
    }
    else
    {
      leg_len_status_ = SHORT;
    }
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
  }
}

void SeriesLeggedManual::unstickCallback(const std_msgs::BoolConstPtr& msg)
{
  auto two_leg_unstick = msg->data;
  if (two_leg_unstick)
  {
    auto delta = legCommandSender_->getLgeLength() + 0.02;
    target_leg_length_ = delta > 0.36 ? 0.36 : delta;
    legCommandSender_->setLgeLength(target_leg_length_);
    stretching_ = true;
  }
  else if (stretching_ && !two_leg_unstick)
  {
    legCommandSender_->setLgeLength(target_leg_length_);
    stretching_ = false;
  }
}

void SeriesLeggedManual::upstairStatusCallback(const rm_msgs::LeggedUpstairStatusConstPtr& msg)
{
  if (msg->upstair_flag)
    --upstair_leg_len_fsm_;
  if (upstair_leg_len_fsm_ == 1)
    leg_len_status_ = HIGH;
  else if (upstair_leg_len_fsm_ <= 0)
  {
    leg_len_status_ = SHORT;
    upstair_leg_len_fsm_ = 0;
  }
  target_leg_length_ = leg_len_map_[leg_len_status_];
  legCommandSender_->setLgeLength(target_leg_length_);
}

void SeriesLeggedManual::leggedChassisModeCallback(const rm_msgs::LeggedChassisModeConstPtr& msg)
{
  static bool trigger_gimbal_normal_flag{ false };
  if (msg->mode != rm_msgs::LeggedChassisMode::NORMAL)
  {
    double roll{}, pitch{}, yaw{};
    try
    {
      quatToRPY(tf_buffer_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    gimbal_cmd_sender_->setGimbalTrajFrameId("base_link");
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    if (msg->mode == rm_msgs::LeggedChassisMode::RECOVERY)
    {
      gimbal_cmd_sender_->setGimbalTraj(yaw, pitch);
    }
    else
    {
      // avoid leg crash gimbal
      gimbal_cmd_sender_->setGimbalTraj(-0.0, pitch);
    }
    trigger_gimbal_normal_flag = true;
  }
  else if (trigger_gimbal_normal_flag)
  {
    trigger_gimbal_normal_flag = false;
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
}

void SeriesLeggedManual::tofSensorMsgCallback(const sensor_msgs::RangeConstPtr& msg)
{
  if (msg->header.frame_id == "left_tof_link")
    left_tof_len_ = msg->range;
  else
    right_tof_len = msg->range;
  total_tof_len_ = (left_tof_len_ + right_tof_len) / 2.0;
}

void SeriesLeggedManual::leftSwitchUpRise()
{
  BalanceManual::leftSwitchUpRise();
  //  legCommandSender_->setJump(true);
  return;
}

void SeriesLeggedManual::leftSwitchMidRise()
{
  BalanceManual::leftSwitchMidRise();
  //  legCommandSender_->setJump(false);
  //  upstair_leg_len_fsm_ = 1;
  //  leg_len_status_ = HIGH;
  //  target_leg_length_ = leg_len_map_[leg_len_status_];
  //  legCommandSender_->setLgeLength(target_leg_length_);
  //  legCommandSender_->setJump(true);
}

void SeriesLeggedManual::leftSwitchDownRise()
{
  BalanceManual::leftSwitchDownRise();
  legCommandSender_->setJump(false);
  upstair_leg_len_fsm_ = 0;
  leg_len_status_ = SHORT;
  target_leg_length_ = leg_len_map_[leg_len_status_];
  legCommandSender_->setLgeLength(target_leg_length_);
}
void SeriesLeggedManual::zPress()
{
  reverse_ = !reverse_;
}

void SeriesLeggedManual::aPress()
{
}

void SeriesLeggedManual::aPressing()
{
}

void SeriesLeggedManual::dPress()
{
}

void SeriesLeggedManual::dPressing()
{
}

}  // namespace rm_manual
