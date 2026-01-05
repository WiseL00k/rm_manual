//
// Created by longyu on 9/21/25.
//

#pragma once

#include "rm_manual/balance_manual.h"
#include <rm_msgs/LeggedChassisMode.h>
#include <sensor_msgs/Range.h>

namespace rm_manual
{
class SeriesLeggedManual : public BalanceManual
{
public:
  SeriesLeggedManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void shiftPress() override;
  void shiftRelease() override;
  void bPress() override;
  void bRelease() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rPress() override;
  void rPressing();
  void rRelease();
  void ctrlWPressing();

  void leftSwitchUpRise() override;

  void leftSwitchMidRise() override;

  void leftSwitchDownRise() override;

  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void ctrlGPress();
  rm_common::LegCommandSender* legCommandSender_{};

private:
  enum Leg_len_status
  {
    SHORT,
    HIGH
  };
  bool stretch_ = false, stretching_ = false, is_increasing_length_ = false;
  Leg_len_status leg_len_status_{ SHORT };
  std::map<Leg_len_status, double> leg_len_map_;
  double target_leg_length_{ 0.22 }, current_leg_length_{};
  ros::Time last_upstairs_time_{};
  double total_tof_len_{}, left_tof_len_{}, right_tof_len{};
  InputEvent ctrl_event_, ctrl_g_event_, ctrl_w_event_;
  ros::Subscriber unstick_sub_, leg_len_status_sub_, legged_chassis_mode_sub_;
  ros::Subscriber left_tof_sensor_sub_, right_tof_sensor_sub_;
  void unstickCallback(const std_msgs::BoolConstPtr& msg);
  void legLenStatusCallback(const std_msgs::BoolConstPtr& msg);
  void leggedChassisModeCallback(const rm_msgs::LeggedChassisModeConstPtr& msg);
  void tofSensorMsgCallback(const sensor_msgs::RangeConstPtr& msg);
};
}  // namespace rm_manual
