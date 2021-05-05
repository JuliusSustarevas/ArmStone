// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <boost/optional.hpp>
#include <boost/circular_buffer.hpp>
#include <numeric>
#include <std_srvs/Trigger.h>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

#include <dynamic_reconfigure/server.h>
#include <vesc_driver/MotorSetupConfig.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

#include <fstream>
#include <iostream>

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);
  void vescFaultCallback(const int& fault_code);

  double radPerSecToErpm(double rad_s);
  double erpmToRadPerSec(double erpm);
  double ticksToRad(double ticks);
  void setDirection();

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  ros::Publisher state_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher battery_pub_;
  ros::Publisher servo_sensor_pub_;
  ros::Subscriber duty_cycle_sub_;
  ros::Subscriber current_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber servo_sub_;
  ros::Subscriber joint_sub_;
  ros::ServiceServer pulse_serv_;
  ros::ServiceServer disable_serv_;
  ros::ServiceServer enable_serv_;
  ros::Timer timer_;
  ros::Timer batt_timer_;

  // Mutexes
  boost::mutex joint_mtx_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING,
    MODE_DISABLED
  } driver_mode_t;

  //TODO: possibly change the order,
  //TOOD: consider changing to DIRECTION_NORMAL/NONE/REVERSED
  //TODO: consider using vesc_driver_cfg enum instead
  // direction of motor rotation
  typedef enum {
    DIRECTION_CLOCKWISE = -1,
    DIRECTION_NONE,
    DIRECTION_COUNTERCLOCKWISE,
  } motor_direction_t;

  std::string joint_name;

  // motor pulse variables
  bool pulse_engaged_ = false;
  float pulse_duration_ = 0.0;
  float pulse_speed_rads_ = 0.0;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc
  motor_direction_t motor_direction_;
  motor_direction_t queued_motor_direction_; //queue motor direction until almost 0 speed achieved
  double last_speed_;
  bool dir_change_requested_;
  bool first_dyn_callback_ = true;
  std::string config_package_;

  // battery measurements
  float last_voltage_ = -1.0;
  float last_current_ = -1.0;
  boost::circular_buffer<double> ang_speed_buffer_;
  int avg_filter_capacity_;

  // motor parameters
  int gear_ratio_;
  int pole_pair_count_;

  // initial state variables
  bool first_value_read_ = true;
  double initial_tachometer_;
  double initial_tachometer_abs_;

  // consts
  const double rad_s_to_rpm_ = 9.54929659642538;
  const double rpm_to_rad_s = 0.104719755;
  // only apply direction change when speed is within lower then this threshold
  const double dir_change_threshod_rad_s = 0.02;

  // ROS callbacks
  void timerCallback(const ros::TimerEvent& event);
  void dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle);
  void currentCallback(const std_msgs::Float64::ConstPtr& current);
  void brakeCallback(const std_msgs::Float64::ConstPtr& brake);
  void speedCallback(const std_msgs::Float64::ConstPtr& speed);
  void positionCallback(const std_msgs::Float64::ConstPtr& position);
  void servoCallback(const std_msgs::Float64::ConstPtr& servo);
  void dynParamCallback(const vesc_driver_cfg::MotorSetupConfig &config, uint32_t level);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState);
  bool pulseCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
  bool disableCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
  bool enableCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
  dynamic_reconfigure::Server<vesc_driver_cfg::MotorSetupConfig> dyn_param_server_;
  dynamic_reconfigure::Server<vesc_driver_cfg::MotorSetupConfig>::CallbackType dyn_param_server_callback_function_;
  void saveToParam(std::string filename, std::string config_package);

  void batteryPubTimerCallback(const ros::TimerEvent& event);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
