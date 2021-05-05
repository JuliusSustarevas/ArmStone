// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <rr_custom_msgs/VescStateStamped.h>

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
  vesc_(std::string(),
        boost::bind(&VescDriver::vescPacketCallback, this, _1),
        boost::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed_rads"),
  position_limit_(private_nh, "position"), servo_limit_(private_nh, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  dir_change_requested_ = false;

  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  }
  catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }

  // create vesc state (telemetry) publisher
  state_pub_ = private_nh.advertise<rr_custom_msgs::VescStateStamped>("sensors/core", 10);
  joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/vesc_driver/joint_state", 10);
  battery_pub_ = nh.advertise<sensor_msgs::BatteryState>("/vesc_driver/battery", 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = private_nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = private_nh.subscribe("motor/duty_cycle", 10,
                                 &VescDriver::dutyCycleCallback, this);
  current_sub_ = private_nh.subscribe("motor/current", 10, &VescDriver::currentCallback, this);
  brake_sub_ = private_nh.subscribe("motor/brake", 10, &VescDriver::brakeCallback, this);
  speed_sub_ = private_nh.subscribe("motor/speed_rads", 10, &VescDriver::speedCallback, this);
  position_sub_ = private_nh.subscribe("motor/position", 10, &VescDriver::positionCallback, this);
  servo_sub_ = private_nh.subscribe("servo/position", 10, &VescDriver::servoCallback, this);
  joint_sub_ = nh.subscribe("/rr_robot/joint_commands", 1, &VescDriver::jointStateCallback, this);
  pulse_serv_ = private_nh.advertiseService("motor/pulse", &VescDriver::pulseCallback, this);
  disable_serv_ = private_nh.advertiseService("disable", &VescDriver::disableCallback, this);
  enable_serv_ = private_nh.advertiseService("enable", &VescDriver::enableCallback, this);

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &VescDriver::timerCallback, this);
  batt_timer_ = nh.createTimer(ros::Duration(1.0), &VescDriver::batteryPubTimerCallback, this);

  private_nh.param<int>("pole_pair_count", pole_pair_count_, 8);
  ROS_INFO_NAMED("vesc_driver", "Pole pair count: %d", pole_pair_count_);
  private_nh.param<int>("gear_ratio", gear_ratio_, 1);
  ROS_INFO_NAMED("vesc_driver", "Gear ratio: %d", gear_ratio_);
  private_nh.param<float>("pulse_duration", pulse_duration_, 0.1);
  ROS_INFO_NAMED("vesc_driver", "Pulse duration: %f", pulse_duration_);
  private_nh.param<float>("pulse_speed_rads", pulse_speed_rads_, 0.6);
  ROS_INFO_NAMED("vesc_driver", "Pulse speed[rad/s]: %f", pulse_speed_rads_);
  private_nh.param("config_package", config_package_, std::string("vesc_driver"));
  private_nh.param<int>("averaging_flter_capacity", avg_filter_capacity_, 8);
  ROS_INFO_NAMED("vesc_driver", "Averaging filter size: %d", avg_filter_capacity_);

  dyn_param_server_callback_function_ = boost::bind(&VescDriver::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);

  motor_direction_ = DIRECTION_NONE;
  last_speed_ = 0; //std::numeric_limits<double>::infinity(); TODO: decide on a safe approach
  ang_speed_buffer_.set_capacity(avg_filter_capacity_);
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds errors
  */
void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
    setDirection();
  }
  else if (driver_mode_ == MODE_DISABLED) {
    ROS_INFO_STREAM_THROTTLE(10, "VESC in disabled mode");
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::batteryPubTimerCallback(const ros::TimerEvent& event)
{
  sensor_msgs::BatteryState::Ptr batt_msg(new sensor_msgs::BatteryState);
  batt_msg->header.stamp = ros::Time::now();
  batt_msg->voltage = last_voltage_;
  batt_msg->current = last_current_;
  battery_pub_.publish(batt_msg);
}

void VescDriver::setDirection()
{
  if (!dir_change_requested_)
  {
    return;
  }

  if(fabs(last_speed_) < dir_change_threshod_rad_s)
  {
    ROS_INFO_NAMED("rr_motor", "Motor direction changed");
    dir_change_requested_ = false;
    motor_direction_ = queued_motor_direction_;
  }
  else
  {
    ROS_WARN_THROTTLE_NAMED(10, "rr_motor","Motor direction will be set when motor speed falls around 0");
  }
}

bool VescDriver::pulseCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    if (fabs(last_speed_) > dir_change_threshod_rad_s)
    {
      // Motor is in motion, don't attempt to drive it
      response.message = "Motor appears to be in motion";
      response.success = false;
      return true;
    }
    pulse_engaged_ = true;
    std_msgs::Float64::Ptr speedCmd(new std_msgs::Float64);
    speedCmd->data = pulse_speed_rads_;
    speedCallback(speedCmd);
    ros::Duration(pulse_duration_).sleep();
    speedCmd->data = 0;
    speedCallback(speedCmd);
    pulse_engaged_ = false;
    response.success = true;
    return true;
}

bool VescDriver::disableCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    if (fabs(last_speed_) > dir_change_threshod_rad_s)
    {
      // Motor is in motion, don't attempt to drive it
      response.message = "Motor appears to be in motion";
      response.success = false;
      return true;
    }
    if (driver_mode_ == MODE_INITIALIZING)
    {
      response.message = "Driver is still initializing";
      response.success = false;
      return true;
    }
    driver_mode_ = MODE_DISABLED;
    response.message = "Motor mode set to disabled";
    response.success = true;
    return true;
}

bool VescDriver::enableCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    if (driver_mode_ == MODE_INITIALIZING)
    {
      response.message = "Driver is still initializing";
      response.success = false;
      return true;
    }
    driver_mode_ = MODE_OPERATING;
    response.message = "Motor mode set to Operating";
    response.success = true;
    return true;
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    if(first_value_read_){
      initial_tachometer_ = values->tachometer();
      initial_tachometer_abs_ = values->tachometer_abs();
      first_value_read_ = false;
    }

    rr_custom_msgs::VescStateStamped::Ptr state_msg(new rr_custom_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed_rads = erpmToRadPerSec(values->rpm()) * motor_direction_; //WARNING: Breaking change compated to VESC driver
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = ticksToRad(values->tachometer() - initial_tachometer_) * motor_direction_;
    state_msg->state.distance_traveled = values->tachometer_abs() - initial_tachometer_abs_;
    state_msg->state.fault_code = values->fault_code();
    vescFaultCallback(values->fault_code());
    state_pub_.publish(state_msg);
    std::vector<double> joint_velocity;
    std::vector<double> joint_position;
    std::vector<std::string> joint_names;
    ang_speed_buffer_.push_back(state_msg->state.speed_rads);
    double sum = std::accumulate(ang_speed_buffer_.begin(), ang_speed_buffer_.end(), 0.0);
    double average_speed = sum/ang_speed_buffer_.size();
    joint_velocity.push_back(average_speed);
    joint_position.push_back(state_msg->state.displacement);
    joint_names.push_back(joint_name);

    sensor_msgs::JointState::Ptr joint_state_msg(new sensor_msgs::JointState);
    joint_state_msg->header.stamp = state_msg->header.stamp;
    joint_state_msg->name = joint_names;
    joint_state_msg->position = joint_position;
    joint_state_msg->velocity = joint_velocity;
    joint_state_pub_.publish(joint_state_msg);

    last_speed_ = state_msg->state.speed_rads;

    last_voltage_ = values->v_in();
    last_current_ = values->current_in();
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

void VescDriver::vescFaultCallback(const int& fault_code)
{
  if(fault_code > 0)
    ROS_ERROR_THROTTLE(1,"Vesc fault code: %d ", fault_code);
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data*motor_direction_));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed WARNING!!!! Breaking change compared to the original VESC driver
 *              Commanded VESC speed in rad/s. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  if (driver_mode_ == MODE_OPERATING) {
    double erpm = radPerSecToErpm(speed->data*motor_direction_);
    vesc_.setSpeed(speed_limit_.clip(erpm));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ == MODE_OPERATING) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  if (driver_mode_ == MODE_OPERATING) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
  }
}

void VescDriver::jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{
  if(pulse_engaged_)
  {
    ROS_ERROR_NAMED("vesc_driver", "Motor pulse in progress, ignoring joint commands.");
    return;
  }

  std_msgs::Float64::Ptr speedCmd(new std_msgs::Float64);

  boost::mutex::scoped_lock lock(joint_mtx_);

  //TODO: Consider performing this check when looping through the jointState.name below
  if (!(std::find(std::begin(jointState->name), std::end(jointState->name), joint_name) != std::end(jointState->name)))
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(7, "vesc_driver", "Name: " << joint_name << " not found in jointState list");
    return;
  }
  if(jointState->velocity.size() != jointState->name.size())
  {
    ROS_ERROR_THROTTLE_NAMED(10, "vesc_driver", "JointState Name-Velocity mismatch");
    return;
  }
  for (int joint_id = 0; joint_id < jointState->name.size(); ++joint_id)
  {
    if(joint_name == jointState->name[joint_id].c_str())
    {
      speedCmd->data = jointState->velocity[joint_id];
      // don't break here - this ensures all the motors will need to loop through
      // the same amount of data - smaller chance they go out of sync
    }
  }
  speedCallback(speedCmd);
}

//TODO: consider taking the joint_name and motor_direction) as arguments
void VescDriver::saveToParam(std::string filename, std::string config_package)
{
  std::string path = ros::package::getPath(config_package) + "/config/motor/" + filename;
  std::ofstream file;
  file.open(path, std::ios::out | std::ios::trunc);

  file << "joint_name: " << joint_name << std::endl;
  file << "motor_direction: " << queued_motor_direction_ << std::endl;
  file.flush();
  file.close();
}

VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min)) {
    if (min_lower && param_min < *min_lower) {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper) {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      lower = param_min;
    }
  }
  else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max)) {
    if (min_lower && param_max < *min_lower) {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper) {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      upper = param_max;
    }
  }
  else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double VescDriver::radPerSecToErpm(double rad_s)
{
  return rad_s*rad_s_to_rpm_*pole_pair_count_*gear_ratio_;
}

double VescDriver::ticksToRad(double ticks)
{
  // pole_pair_count*2*3 - ticks per revolution. Multiplied x2 to get
  // number of poles, and multiplied x3 becuase there are 3 phases
  //TODO: verify
  return ticks*2*M_PI/(pole_pair_count_*2*3*gear_ratio_);
}

double VescDriver::erpmToRadPerSec(double erpm)
{
  return erpm/(pole_pair_count_*gear_ratio_)*rpm_to_rad_s;
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

void VescDriver::dynParamCallback(
    const vesc_driver_cfg::MotorSetupConfig &config, uint32_t level)
{
  dir_change_requested_ = true;
  switch(config.motor_direction)
  {
    case vesc_driver_cfg::MotorSetup_CLOCKWISE:
      queued_motor_direction_ = DIRECTION_CLOCKWISE;
      ROS_INFO("DIRECTION CLOCKWISE");
      break;
    case vesc_driver_cfg::MotorSetup_COUNTERCLOCKWISE:
      queued_motor_direction_ = DIRECTION_COUNTERCLOCKWISE;
      ROS_INFO("DIRECTION COUNTERCLOCKWISE");
      break;
    default:
      queued_motor_direction_ = DIRECTION_NONE;
      ROS_WARN_NAMED("vesc_driver", "Invalid value for motor direction. Setting to None");
  }

  boost::mutex::scoped_lock lock(joint_mtx_);

  joint_name = config.joint_name;
  ROS_INFO_STREAM("Motor joint name: " << joint_name);
  if (!first_dyn_callback_)
  {
    saveToParam("module_settings.yaml", config_package_);
  }
  first_dyn_callback_ = false;
}

} // namespace vesc_driver
