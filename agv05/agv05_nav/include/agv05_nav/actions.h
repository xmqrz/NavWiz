/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAV_ACTIONS_H
#define AGV05_NAV_ACTIONS_H

#include <agv05_nav/action_processor.h>
#include <agv05_nav/utils.h>


namespace agv05
{

typedef std::complex<float> complex;

class ActionManualControl: public ActionProcessor
{
public:
  ActionManualControl(Nav& nav, const Goal& goal);
  void process(float frequency);

protected:
  bool isPausedManualControl()
  {
    return ((laser_sensor_.getArea() == agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL) ||
            (isPaused() && base_.getManualCmdVelTimeout() < 0.5));
  }

protected:
  SignedVelocitySmoother<float> speed_x_, speed_y_, speed_z_;

  float speed_limit_;
  bool force_enable_sensor_;
  uint8_t area_;
};

class ActionStraight: public ActionManualControl
{
public:
  /**
   * Goal params:
   * - line_follow_type
   * - speed
   * - enable_sensor
   * - distance
   * - io_trigger_type
   * - io_trigger_port
   * - io_trigger_pin
   * - error_io_trigger_type
   * - error_io_trigger_port
   * - error_io_trigger_pin
   * - next_motion
   * - next_distance
   */
  ActionStraight(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    state_(RUNNING),
    reaching_(config_.straight_junction_min_speed,
              config_.straight_junction_dec < config_.getProfile().straight_normal_dec ?
              config_.straight_junction_dec : config_.getProfile().straight_normal_dec,  // must be smaller for VelocitySmoother to catch up
              config_.getProfile().straight_jerk_dec),
    line_speed_(goal.speed),
    initial_jsd_(base_.getStraightDistance()), initial_olsd_(base_.getStraightDistance()),
    overshoot_(0.0f),
    out_of_line_filter_(false, config_.straight_out_of_line_distance),
    position_align_filter_(false, config_.straight_alignment_max_time),
    io_trigger_latch_(goal.io_trigger_type, false, config_.io_trigger_debounce),
    error_io_trigger_latch_(goal.error_io_trigger_type, false, config_.io_trigger_debounce),
    io_trigger_(false)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_FORWARD ||
               goal.nav == Goal::NAV_REVERSE ||
               goal.nav == Goal::NAV_FORWARD_LEFT ||
               goal.nav == Goal::NAV_FORWARD_RIGHT ||
               goal.nav == Goal::NAV_REVERSE_LEFT ||
               goal.nav == Goal::NAV_REVERSE_RIGHT ||
               goal.nav == Goal::NAV_BEZIER_FORWARD ||
               goal.nav == Goal::NAV_BEZIER_REVERSE ||
               goal.nav == Goal::NAV_BEZIER_FORWARD_LEFT ||
               goal.nav == Goal::NAV_BEZIER_FORWARD_RIGHT ||
               goal.nav == Goal::NAV_BEZIER_REVERSE_LEFT ||
               goal.nav == Goal::NAV_BEZIER_REVERSE_RIGHT ||
               goal.nav == Goal::NAV_LINE_TUNE_PID);
    ROS_ASSERT(goal.io_trigger_type < io_trigger_latch_.NUM_TRIGGER_TYPES);
    ROS_ASSERT(goal.io_trigger_port <= io_.NUM_PORTS);
    ROS_ASSERT(goal.io_trigger_pin < io_.NUM_PINS);
    ROS_ASSERT(goal.error_io_trigger_type < error_io_trigger_latch_.NUM_TRIGGER_TYPES);
    ROS_ASSERT(goal.error_io_trigger_port <= io_.NUM_PORTS);
    ROS_ASSERT(goal.error_io_trigger_pin < io_.NUM_PINS);

    initialize();
  }

  void initialize();
  void process(float frequency);
  void handleNavControl(const NavControl& nav_control);
  float getOvershoot()
  {
    return overshoot_;
  }

  static void resetSpeed()
  {
    speed_.setSpeed(0.0f);
  }

protected:
  void running(float frequency);
  void onOverJunction(float frequency);
  void stopExit(float frequency);

  virtual void lineFollow(float frequency, float speed, float line_angular_error, float line_heading_error);
  void resetLineFollowVariable();
  void lineForwardSpeedUpdate();

  float computeJunctionSpeed(float junction_distance);
  float computeLinearErrorSpeed(float linear_error);
  void updateLineSensorData(bool init);
  float getStraightDistance0();
  float getLinearError0();
  float getAngularError0();
  float getAngularError0(float linear_error);
  virtual float getHeadingError0();

  UseLineSensor getLineSensor();
  agv05_msgs::LineSensor getLineSensorData0();
  float getTargetJunctionDistance0();

  inline complex poseComplex()
  {
    return complex(pose_.x - line_start_.x, pose_.y - line_start_.y);
  }
  virtual void resetPose()
  {
    base_.getPose(line_start_);
    line_vector_ = std::polar<float>(1.0f, forward_dir_ ? line_start_.theta : line_start_.theta + M_PI);
    line_start_.theta = base_.getRotationalDistance();
  }
  virtual void updatePose()
  {
    base_.getPose(pose_);
  }

protected:
  enum State
  {
    RUNNING,
    BY_DISTANCE,
    ON_JUNCTION,
    OVER_JUNCTION,
    OVER_IO_DETECTED,
    OVER_DISTANCE,
    STOP_EXIT,
    STOP_EXIT_JUNCTION,
    STOP_EXIT_IO_DETECTED,
    STOP_EXIT_ERROR_TRIGGERED,
  } state_;

  bool forward_dir_;
  static VelocitySmoother<float> speed_;  // has to be static to enable forward_flag action
  ReachingKinematic<float> reaching_;
  float junction_stopping_speed_;  // calculated junction stopping speed
  float junction_stopping_distance_;  // calculated junction stopping distance
  float line_speed_;
  complex line_vector_;
  geometry_msgs::Pose2D line_start_;
  geometry_msgs::Pose2D pose_;
  double initial_jsd_;  // junction straight distance
  double initial_olsd_;  // out-of-line straight distance
  float overshoot_;

  // led
  uint8_t led_side_;
  uint8_t next_led_side_;  // aka next_turning

  // line sensor data
  agv05_msgs::LineSensor lsd_;
  agv05_msgs::LineSensor initial_lsd_;  // line sensor data before entering junction
  agv05_msgs::LineSensor inline_lsd_;  // line sensor data before out-of-line

  // out-of-line filter
  DigitalInputFilter<double> out_of_line_filter_;

  // alignment filter
  DigitalInputFilter<float> position_align_filter_;

  // io trigger
  FlipFlop<float> io_trigger_latch_;
  FlipFlop<float> error_io_trigger_latch_;
  bool io_trigger_;

  // data buffer
  float line_follow_error_i_;
  float line_follow_error_d_;
  float line_follow_theta_;
};

class ActionTurn: public ActionManualControl
{
public:
  /**
   * Goal params:
   * - line_follow_type
   * - enable_sensor
   * - rotate_align_sensor
   * - distance
   */
  ActionTurn(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    state_(TURNING),
    line_follow_type_(goal.line_follow_type),
    reaching_(config_.turn_near_block_speed,
              config_.getProfile().turn_normal_dec * 0.5f,  // must be smaller for VelocitySmoother to catch up
              config_.getProfile().turn_jerk_dec),
    initial_rd_(base_.getRotationalDistance()),
    line_found_filter_(false, 0.1f),
    line_align_filter_(false, config_.turn_alignment_center_min_time, config_.turn_alignment_max_time)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_ROTATE_LEFT ||
               goal.nav == Goal::NAV_ROTATE_RIGHT ||
               goal.nav == Goal::NAV_UTURN_LEFT ||
               goal.nav == Goal::NAV_UTURN_RIGHT ||
               goal.nav == Goal::NAV_ROTATE3Q_LEFT ||
               goal.nav == Goal::NAV_ROTATE3Q_RIGHT ||
               goal.nav == Goal::NAV_SEARCH_LINE_LEFT ||
               goal.nav == Goal::NAV_SEARCH_LINE_RIGHT ||
               goal.nav == Goal::NAV_LINE_CALIBRATE);
    ROS_ASSERT(goal.rotate_align_sensor < Goal::ALIGN_SENSOR_MAX_NUMBER);

    initialize();
  }

  void initialize();
  void process(float frequency);

protected:
  void turning(float frequency);
  void searchLine(float frequency);
  void alignLine(float frequency);
  void stopExit(float frequency);

  void turningAlign(float frequency, float line_angular_error, float max_angular_speed);

  UseLineSensor getLineSensor();
  agv05_msgs::LineSensor getLineSensorData0();
  void turning0(float speed);

protected:
  enum State
  {
    TURNING,
    SEARCH_LINE,
    ALIGN_LINE,
    STOP_EXIT,
    STOP_EXIT_NO_LINE,
  } state_;

  bool left_dir_;
  uint8_t line_follow_type_;
  VelocitySmoother<float> speed_;
  ReachingKinematic<float> reaching_;
  double initial_rd_;  // rotational distance
  float turn_distance_;

  DigitalInputFilter<float> line_found_filter_;
  DigitalInputFilter<float> line_align_filter_;
};

class ActionLineCalibrate: public ActionTurn
{
public:
  ActionLineCalibrate(Nav& nav, const Goal& goal) :
    ActionTurn(nav, goal),
    state_(TURN_LEFT_1),
    final_rd_(base_.getRotationalDistance())
  {
    initialize();
  }

  void initialize();
  void process(float frequency);

protected:
  void checkCalibratedData();

  void completeNav(uint8_t result);
  uint8_t checkSafetyStatus(bool enable_sensor, bool resume = false, bool trigger_out_of_line = false, UseLineSensor use_line_sensor = LINE_SENSOR_DISABLE);

protected:
  enum State
  {
    TURN_LEFT_1,
    TURN_RIGHT_2,
    CHECK_CALIBRATED_DATA,
    TURN_LEFT_3,
  } state_;

  double final_rd_;  // rotational distance
  double initial_cal_time_;
};

class ActionTunePID: public ActionStraight
{
public:
  /**
   * Goal params:
   * - distance
   */
  ActionTunePID(Nav& nav, const Goal& goal) :
    ActionStraight(nav, goal_init(nav, goal)),
    v_(0.0f)
  {
    ROS_ASSERT(goal.distance != 0.0);

    initialize();
  }

  void initialize();

protected:
  float computeTu();
  void updatePID();

  void completeNav(uint8_t result);
  uint8_t checkSafetyStatus(bool enable_sensor, bool resume = false, bool trigger_out_of_line = false, UseLineSensor use_line_sensor = LINE_SENSOR_DISABLE);

  void lineFollow(float frequency, float speed, float line_angular_error, float line_heading_error);
  float getHeadingError0();

  void resetPose()
  {
    base_.getPose(line_start_, true);
    line_vector_ = std::polar<float>(1.0f, forward_dir_ ? line_start_.theta : line_start_.theta + M_PI);
  }
  void updatePose()
  {
    base_.getPose(pose_, true);
  }

protected:
  double kp_, ki_, kd_, kh_;
  std::vector<complex> error_;
  float v_;

private:
  // override goal
  Goal& goal_init(Nav& nav, const Goal& goal);
  Goal g_;
};

class ActionSelectNavProfile: public ActionProcessor
{
public:
  ActionSelectNavProfile(Nav& nav, const Goal& goal);
  void process(float frequency) {}
};

class ActionSelectLaserProfile: public ActionProcessor
{
public:
  ActionSelectLaserProfile(Nav& nav, const Goal& goal);
  void process(float frequency) {}
};

class ActionScanLaser: public ActionProcessor
{
public:
  ActionScanLaser(Nav& nav, const Goal& goal);
  void process(float frequency);

protected:
  float delay_;
};

class ActionFreeMotor: public ActionProcessor
{
public:
  ActionFreeMotor(Nav& nav, const Goal& goal);
  void process(float frequency);
};

class ActionWaitTraffic: public ActionProcessor
{
public:
  ActionWaitTraffic(Nav& nav, const Goal& goal) :
    ActionProcessor(nav, goal)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_WAIT_TRAFFIC);
  }

  void process(float frequency);
};

}  // namespace agv05

#endif  // AGV05_NAV_ACTIONS_H
