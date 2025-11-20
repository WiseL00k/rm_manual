//
// Created by longyu on 9/21/25.
//

//
// Created by kook on 9/28/24.
//

#include "rm_manual/series_legged_manual.h"

namespace rm_manual
{
SeriesLeggedManual::SeriesLeggedManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : BalanceManual(nh, nh_referee)
{
  ros::NodeHandle leg_wheel_chassis_nh(nh, "balance/legged_wheel_chassis");
  legCommandSender_ = new rm_common::LegCommandSender(leg_wheel_chassis_nh);
  legCommandSender_->setLgeLength(0.18);
  legCommandSender_->setJump(false);

  b_event_.setEdge(boost::bind(&SeriesLeggedManual::bPress, this), boost::bind(&SeriesLeggedManual::bRelease, this));
  r_event_.setEdge(boost::bind(&SeriesLeggedManual::rPress, this), boost::bind(&SeriesLeggedManual::rRelease, this));
  ctrl_g_event_.setRising(boost::bind(&SeriesLeggedManual::ctrlGPress, this));

  std::string unstick_topic;
  leg_wheel_chassis_nh.param("unstick_topic", unstick_topic,
                             std::string("/controllers/legged_balance_controller/unstick/two_leg_unstick"));
  unstick_sub_ =
      leg_wheel_chassis_nh.subscribe<std_msgs::Bool>(unstick_topic, 1, &SeriesLeggedManual::unstickCallback, this);
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
    target_leg_length = 0.18;
    legCommandSender_->setLgeLength(0.18);
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
  legCommandSender_->setJump(true);
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
    target_leg_length = 0.36;
    legCommandSender_->setLgeLength(0.36);
    stretch_ = true;
  }
  else
  {
    target_leg_length = 0.18;
    legCommandSender_->setLgeLength(0.18);
    stretch_ = false;
  }
}

void SeriesLeggedManual::unstickCallback(const std_msgs::BoolConstPtr& msg)
{
  auto two_leg_unstick = msg->data;
  if (two_leg_unstick)
  {
    auto delta = legCommandSender_->getLgeLength() + 0.02;
    target_leg_length = delta > 0.36 ? 0.36 : delta;
    legCommandSender_->setLgeLength(target_leg_length);
    stretching_ = true;
  }
  else if (stretching_ && !two_leg_unstick)
  {
    legCommandSender_->setLgeLength(target_leg_length);
    stretching_ = false;
  }
}

void SeriesLeggedManual::leftSwitchUpRise()
{
  legCommandSender_->setJump(false);
  return;
}

void SeriesLeggedManual::leftSwitchMidRise()
{
  legCommandSender_->setJump(false);
  target_leg_length = 0.36;
  legCommandSender_->setLgeLength(0.36);
}

void SeriesLeggedManual::leftSwitchDownRise()
{
  legCommandSender_->setJump(false);
  target_leg_length = 0.20;
  legCommandSender_->setLgeLength(0.20);
}

}  // namespace rm_manual
