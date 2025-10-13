/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAVX_ACTIONS_H
#define AGV05_NAVX_ACTIONS_H

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include <agv05_navx/action_processor.h>
#include <agv05_navx/utils.h>


namespace agv05
{

typedef std::complex<double> complex;

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

  uint8_t area_;
};

class ActionStraight: public virtual ActionManualControl
{
public:
  /**
   * Goal params:
   * - speed
   * - enable_sensor
   * - path_start
   * - path_end
   * - sense_line
   * - io_trigger_type
   * - io_trigger_port
   * - io_trigger_pin
   * - error_io_trigger_type
   * - error_io_trigger_port
   * - error_io_trigger_pin
   * - next_motion
   * - next_speed
   * - next_distance
   */
  ActionStraight(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    state_(RUNNING),
    reaching_(config_.straight_reaching_min_speed,
              config_.straight_reaching_dec < config_.getProfile().straight_normal_dec ?
              config_.straight_reaching_dec : config_.getProfile().straight_normal_dec,  // must be smaller for VelocitySmoother to catch up
              config_.getProfile().straight_jerk_dec),
    overshoot_(0.0f),
    path_start_(goal.path_start.x, goal.path_start.y),
    path_end_(goal.path_end.x, goal.path_end.y),
    path_heading_(0.0),
    path_speed_(goal.speed),
    sense_line_(goal.sense_line), sense_line_distance_(0),
    position_align_filter_(false, config_.straight_alignment_max_time),
    io_trigger_latch_(goal.io_trigger_type, false, config_.io_trigger_debounce),
    error_io_trigger_latch_(goal.error_io_trigger_type, false, config_.io_trigger_debounce),
    io_trigger_(false)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_FORWARD ||
               goal.nav == Goal::NAV_REVERSE ||
               goal.nav == Goal::NAV_BEZIER_FORWARD ||
               goal.nav == Goal::NAV_BEZIER_REVERSE ||
               goal.nav == Goal::NAV_FORWARD_DOCK ||
               goal.nav == Goal::NAV_REVERSE_DOCK ||
               goal.nav == Goal::NAV_FORWARD_UNDOCK ||
               goal.nav == Goal::NAV_REVERSE_UNDOCK ||
               goal.nav == Goal::NAV_ROTATE_LEFT ||
               goal.nav == Goal::NAV_ROTATE_RIGHT ||
               goal.nav == Goal::NAV_DYNAMIC_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_REVERSE ||
               goal.nav == Goal::NAV_OMNI ||
               goal.nav == Goal::NAV_BEZIER_OMNI ||
               goal.nav == Goal::NAV_OMNI_DOCK ||
               goal.nav == Goal::NAV_OMNI_UNDOCK ||
               goal.nav == Goal::NAV_DYNAMIC_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_OMNI);
    ROS_ASSERT(goal.io_trigger_type < io_trigger_latch_.NUM_TRIGGER_TYPES);
    ROS_ASSERT(goal.io_trigger_port <= io_.NUM_PORTS);
    ROS_ASSERT(goal.io_trigger_pin < io_.NUM_PINS);
    ROS_ASSERT(goal.error_io_trigger_type < error_io_trigger_latch_.NUM_TRIGGER_TYPES);
    ROS_ASSERT(goal.error_io_trigger_port <= io_.NUM_PORTS);
    ROS_ASSERT(goal.error_io_trigger_pin < io_.NUM_PINS);

    initialize();
  }

  void initialize();
  void process(float frequency)
  {
    straight(frequency);
  }
  void straight(float frequency);
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
  virtual void planning(float frequency) {}
  virtual void running(float frequency);
  void alignPosition(float frequency, bool align, bool align_ready = false, bool force_exit = false);
  void stopExit(float frequency);

  virtual void pathFollow(float frequency, float speed, float angular_error, float heading_error);
  void resetPathFollowVariable();
  void pathForwardSpeedUpdate();
  void pathCurvatureUpdate(float curvature);

  virtual float computeOutputLimit(float speed, float heading_error);
  float computeReachingDistance(float target_distance);
  float computeReachingSpeed(float reaching_distance);
  virtual void alignPositionInit();
  virtual bool handlePoseUpdate()
  {
    return true;
  }
  virtual float getTargetDistance0();
  virtual float getLinearError0();
  virtual float getAngularError0();
  float getAngularError0(float linear_error);
  virtual float getHeadingError0();

  float adjustTargetDistanceByLineSensing0(float distance);
  UseLineSensor getLineSensor();
  agv05_msgs::LineSensor getLineSensorData0();

protected:
  enum State
  {
    PLANNING,
    RUNNING,
    ALIGN_POSITION,
    STOP_EXIT,
    STOP_EXIT_REACHING,
    STOP_EXIT_IO_DETECTED,
    STOP_EXIT_ERROR_TRIGGERED,
  } state_;

  bool forward_dir_;
  static VelocitySmoother<float> speed_;  // has to be static to enable forward_flag action
  ReachingKinematic<float> reaching_;
  geometry_msgs::Pose2D pose_;
  float overshoot_;

  // goal path
  const complex path_start_;
  const complex path_end_;
  double path_heading_;
  float path_curvature_;
  float path_speed_;
  float next_speed_;
  float reaching_distance_;

  // led
  uint8_t led_side_;
  uint8_t next_led_side_;  // aka next_turning

  // line sensing
  uint8_t sense_line_;
  float sense_line_distance_;

  // alignment filter
  DigitalInputFilter<float> position_align_filter_;

  // io trigger
  FlipFlop<float> io_trigger_latch_;
  FlipFlop<float> error_io_trigger_latch_;
  bool io_trigger_;
  float io_trigger_distance_;
  geometry_msgs::Pose2D io_trigger_pose_;

  // data buffer
  float path_follow_error_i_;
  float path_follow_error_d_;
};

class ActionBezier: public virtual ActionStraight
{
public:
  /**
   * Goal params:
   * - speed
   * - enable_sensor
   * - path_start
   * - path_end
   * - path_cp1
   * - path_cp2
   * - sense_line
   * - next_motion
   * - next_speed
   * - next_distance
   */
  ActionBezier(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    path_cp1_(goal.path_cp1.x, goal.path_cp1.y),
    path_cp2_(goal.path_cp2.x, goal.path_cp2.y),
    bezier_cur_(0), bezier_goal_(0), bezier_inflection_(0)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_BEZIER_FORWARD ||
               goal.nav == Goal::NAV_BEZIER_REVERSE ||
               goal.nav == Goal::NAV_BEZIER_OMNI);

    generateBezierSegments();
  }

protected:
  void generateBezierSegments();
  bool handlePoseUpdate()
  {
    advanceBezierCursor0();
    return true;
  }
  void advanceBezierCursor0();
  virtual float getTargetDistance0();
  float getLinearError0();
  virtual float getHeadingError0();
  complex interpolateBezier0(double t);
  complex derivativeBezier0(double t);
  complex derivative2Bezier0(double t);

protected:
  // goal path
  const complex path_cp1_;
  const complex path_cp2_;

  // pre-computed path segments
  std::vector<complex> bezier_points_;
  std::vector<double> bezier_target_distances_;
  std::vector<float> bezier_headings_;

  // cursor into the path segments
  size_t bezier_cur_, bezier_goal_, bezier_inflection_;
};

class ActionDock: public virtual ActionStraight
{
public:
  /**
   * Goal params:
   * - speed
   * - enable_sensor
   * - marker_type
   * - path_start (start point in marker frame - for undock only)
   * - path_end (target in marker frame)
   * - path_cp2 (alignment distance)
   * - sense_line
   * - io_trigger_type
   * - io_trigger_port
   * - io_trigger_pin
   * - error_io_trigger_type
   * - error_io_trigger_port
   * - error_io_trigger_pin
   * - next_motion
   * - next_speed
   * - next_distance
   */
  ActionDock(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    planning_throttle_(false, 0.5f),
    planning_retries_(config_.dock_marker_timeout * 2 - 1),  // retry up to dock_marker_timeout seconds
    plan_cur_(0)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_FORWARD_DOCK ||
               goal.nav == Goal::NAV_REVERSE_DOCK ||
               goal.nav == Goal::NAV_FORWARD_UNDOCK ||
               goal.nav == Goal::NAV_REVERSE_UNDOCK ||
               goal.nav == Goal::NAV_OMNI_DOCK ||
               goal.nav == Goal::NAV_OMNI_UNDOCK);

    state_ = PLANNING;
    initialize();
  }

  ~ActionDock()
  {
    // clear plan display in UI
    plan_.clear();
    base_.publishDockingPlan(plan_);
    base_.clearMarkerType();
  }

  void initialize();

protected:
  void planning(float frequency);

  virtual bool generatePlan();
  virtual bool handlePoseUpdate()
  {
    return advancePlanCursor0();
  }
  bool advancePlanCursor0();
  virtual float getTargetDistance0();
  virtual float getLinearError0();
  virtual float getHeadingError0();

  inline complex planComplex(size_t i)
  {
    const geometry_msgs::PoseStamped& p = plan_[i];
    return complex(p.pose.position.x, p.pose.position.y);
  }

  inline complex poseComplex()
  {
    return complex(pose_.x, pose_.y);
  }

protected:
  bool undock_;
  geometry_msgs::Pose2D undock_end_;

  // planning retries
  DigitalInputFilter<float> planning_throttle_;
  int planning_retries_;

  // docking plan
  std::vector<geometry_msgs::PoseStamped> plan_;
  std::vector<double> plan_distances_;

  // cursor into the path segment
  size_t plan_cur_;
};

class ActionTurn: public virtual ActionManualControl
{
public:
  /**
   * Goal params:
   * - enable_sensor
   * - path_end (heading)
   */
  ActionTurn(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    state_(TURNING),
    reaching_(config_.turn_near_block_speed,
              config_.getProfile().turn_normal_dec * 0.5f,  // must be smaller for VelocitySmoother to catch up
              config_.getProfile().turn_jerk_dec),
    position_align_filter_(false, config_.turn_alignment_center_min_time, config_.turn_alignment_max_time)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_ROTATE_LEFT ||
               goal.nav == Goal::NAV_ROTATE_RIGHT ||
               goal.nav == Goal::NAV_DYNAMIC_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_REVERSE ||
               goal.nav == Goal::NAV_OMNI_DOCK ||
               goal.nav == Goal::NAV_DYNAMIC_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_OMNI);

    initialize();
  }

  void initialize();
  void process(float frequency);

protected:
  void turning(float frequency);
  void alignPosition(float frequency);
  void stopExit(float frequency);

  void turningAlign(float frequency, float angular_error, float max_angular_speed);

  float getAngularError0();
  void turning0(float speed);

protected:
  enum State
  {
    TURNING,
    ALIGN_POSITION,
    STOP_EXIT,
  } state_;

  bool left_dir_;
  VelocitySmoother<float> speed_;
  ReachingKinematic<float> reaching_;
  geometry_msgs::Pose2D pose_;

  DigitalInputFilter<float> position_align_filter_;
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

class ActionDynamic: public virtual ActionStraight, public ActionTurn
{
public:
  /**
   * Goal params:
   * - speed
   * - goal_tolerance
   * - enable_sensor
   * - path_start
   * - path_end
   * - path_cp1
   * - path_cp2
   * - paths
   * - sense_line
   * - next_motion
   */
  ActionDynamic(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionTurn(nav, goal),
    ActionStraight(nav, goal),
    state_(RUNNING),
    plan_cur_(0),
    auto_resume_(true),
    manual_resume_(false)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_DYNAMIC_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_FORWARD ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_REVERSE ||
               goal.nav == Goal::NAV_DYNAMIC_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_OMNI);

    initialize();
  }

  ~ActionDynamic()
  {
    base_.stopPlanner(forward_flag_, plan_);
  }

  void initialize();
  void process(float frequency)
  {
    switch (state_)
    {
    case RUNNING: ActionStraight::process(frequency); break;
    case TURNING: ActionTurn::process(frequency); break;
    default: break;
    }
  }
  void handleNavControl(const NavControl& nav_control);
  uint8_t getStatus();

protected:
  typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
  typedef Kernel::Point_2 Point_2;
  typedef CGAL::Polygon_2<Kernel> Polygon_2;

protected:
  void completeNav(uint8_t result);
  uint8_t checkSafetyStatus(bool enable_sensor, bool resume = false, bool trigger_navigation_failed = false, UseLineSensor use_line_sensor = LINE_SENSOR_DISABLE);

  void running(float frequency);

  bool alignPositionDone();
  bool handlePoseUpdate();
  virtual float getTargetDistance0();
  float getLinearError0();
  virtual float getHeadingError0();
  float computeOutputLimit(float speed, float heading_error);

  Polygon_2 getPathArea(const geometry_msgs::Polygon& path);

  inline complex planComplex(size_t i)
  {
    const geometry_msgs::PoseStamped& p = plan_[i];
    return complex(p.pose.position.x, p.pose.position.y);
  }

  inline complex poseComplex()
  {
    return state_ == TURNING ? complex(ActionTurn::pose_.x, ActionTurn::pose_.y) :
                               complex(ActionStraight::pose_.x, ActionStraight::pose_.y);
  }

protected:
  enum State
  {
    RUNNING,
    TURNING,
    STOP_EXIT,
  } state_;

  // dynamically-planned path
  std::vector<geometry_msgs::PoseStamped> plan_;
  static uint32_t plan_seq_;

  // cursor into the plan segments
  size_t plan_cur_, plan_goal_;
  // path area
  Polygon_2 path_area_cur_, path_area_goal_;

  // tolerance stopping distance from goal
  float target_tolerance_;
  // distance from current to sub-goal
  float target_distance_;
  // maximum target distance from current to sub-goal
  float target_dist_max_;
  // path distance from current to goal
  float path_distance_;
  // maximum allowable path distance from current to goal
  float path_dist_max_;

  // last time when valid plan was available
  ros::Time plan_time_;

  // resume flags
  bool auto_resume_;
  bool manual_resume_;

  // is current goal a midway junction?
  bool junction_mid_;
};

class ActionOmni: public virtual ActionStraight
{
public:
  /**
   * Goal params:
   * - speed
   * - enable_sensor
   * - path_start
   * - path_end
   * - sense_line
   * - io_trigger_type
   * - io_trigger_port
   * - io_trigger_pin
   * - error_io_trigger_type
   * - error_io_trigger_port
   * - error_io_trigger_pin
   * - next_motion
   * - next_speed
   * - next_distance
   */
  ActionOmni(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_OMNI ||
               goal.nav == Goal::NAV_BEZIER_OMNI ||
               goal.nav == Goal::NAV_OMNI_UNDOCK ||
               goal.nav == Goal::NAV_DYNAMIC_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_OMNI);

    initialize();
  }

  void initialize();

protected:
  void pathFollow(float frequency, float speed, float angular_error, float heading_error);
  void pathSpeedUpdate(float end_heading, float path_distance);
  void reachingDistanceUpdate(float heading_error);
  void reachingDistanceUpdate(float end_heading, float path_heading);

  virtual float getTargetDistance0();
  float getAngularError0()
  {
    return angular_error_;
  }
  virtual float getHeadingError0()
  {
    return getHeadingError0(ActionStraight::getHeadingError0());
  }
  float getHeadingError0(float heading_error);

protected:
  VelocitySmoother<float> speed_y_;
  bool dir_y_;
};

class ActionBezierOmni: public ActionBezier, public ActionOmni
{
public:
  /**
   * Goal params:
   * - speed
   * - enable_sensor
   * - path_start
   * - path_end
   * - path_cp1
   * - path_cp2
   * - sense_line
   * - next_motion
   * - next_speed
   * - next_distance
   */
  ActionBezierOmni(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    ActionBezier(nav, goal),
    ActionOmni(nav, goal)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_BEZIER_OMNI);

    initialize();
  }

  void initialize()
  {
    pathSpeedUpdate(goal_.path_end.theta, bezier_target_distances_.front());
    reachingDistanceUpdate(goal_.path_end.theta, path_heading_ > 0.0 ? path_heading_ :
                                                 forward_dir_ ? bezier_headings_.back() :
                                                                bezier_headings_.back() + M_PI);
  }

protected:
  float getTargetDistance0()
  {
    return ActionBezier::getTargetDistance0();
  }
  float getHeadingError0()
  {
    return ActionOmni::getHeadingError0(ActionBezier::getHeadingError0());
  }
};

class ActionDynamicOmni: public ActionDynamic, public ActionOmni
{
public:
  /**
   * Goal params:
   * - speed
   * - goal_tolerance
   * - enable_sensor
   * - path_start
   * - path_end
   * - path_cp1
   * - path_cp2
   * - paths
   * - sense_line
   * - next_motion
   */
  ActionDynamicOmni(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    ActionDynamic(nav, goal),
    ActionOmni(nav, goal)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_DYNAMIC_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_LINE_OMNI ||
               goal.nav == Goal::NAV_DYNAMIC_BEZIER_OMNI);

    initialize();
  }

  void initialize()
  {
    if (goal_.path_end.theta > 0.0)
    {
      reachingDistanceUpdate(M_PI);
    }
  }

protected:
  float getTargetDistance0()
  {
    return ActionDynamic::getTargetDistance0();
  }
  float getHeadingError0()
  {
    return ActionOmni::getHeadingError0(ActionDynamic::getHeadingError0());
  }
};

class ActionUndockOmni: public ActionDock, public ActionOmni
{
public:
  /**
   * Goal params:
   * - speed
   * - enable_sensor
   * - marker_type
   * - path_start (start point in marker frame - for undock only)
   * - path_end (target in marker frame)
   * - path_cp2 (alignment distance and heading in marker frame)
   * - next_motion
   * - next_speed
   */
  ActionUndockOmni(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    ActionDock(nav, goal),
    ActionOmni(nav, goal)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_OMNI_UNDOCK);

    initialize();
  }

  void initialize()
  {
    pathHeadingUpdate(goal_.path_end.theta);
    reachingDistanceUpdate(goal_.path_start.theta, forward_dir_ ? goal_.path_end.theta :
                                                                  goal_.path_end.theta + M_PI);
  }

protected:
  void pathHeadingUpdate(double path_heading)
  {
    if (goal_.path_cp2.theta > 0.0)
    {
      path_heading -= goal_.path_cp2.theta;  // -2π < Δθ < 2π
      while (path_heading <= 0.0) path_heading += 2 * M_PI;  // normalize to 0 < Δθ < 2π
      path_heading_ = path_heading;
    }
  }

  void alignPositionInit()
  {
    ActionDock::alignPositionInit();
    pathHeadingUpdate(goal_.path_start.theta);
  }
  float getTargetDistance0()
  {
    return ActionDock::getTargetDistance0();
  }
  float getHeadingError0()
  {
    return ActionOmni::getHeadingError0(ActionDock::getHeadingError0());
  }
};

class ActionTurnOmni: public virtual ActionStraight, public ActionTurn
{
public:
  /**
   * Goal params:
   * - goal_tolerance
   * - enable_sensor
   * - path_end
   */
  ActionTurnOmni(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    ActionTurn(nav, goal),
    speed_x_(ActionStraight::speed_), dir_x_(ActionStraight::forward_dir_), speed_x_max_(ActionStraight::path_speed_),
    speed_z_(ActionTurn::speed_), dir_z_(ActionTurn::left_dir_),
    dir_y_(true)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_ROTATE_LEFT ||
               goal.nav == Goal::NAV_ROTATE_RIGHT ||
               goal.nav == Goal::NAV_OMNI_DOCK);

    initialize();
  }

  void initialize();
  void process(float frequency)
  {
    if (ActionTurn::state_ == ActionTurn::TURNING)
    {
      ActionTurn::process(frequency);
    }
    else
    {
      processAlign(frequency);
    }
  }
  void processAlign(float frequency);

protected:
  void alignPosition(float frequency);
  void stopExit(float frequency);

  void omniAlign(float frequency, float error, float speed_max, VelocitySmoother<float>& speed, bool& dir,
                 ReachingKinematic<float>& reaching, float center_error);
  void omniAlign(float frequency, float error, float speed_max, VelocitySmoother<float>& speed, bool& dir);
  void moving0();

  virtual bool handlePoseUpdate();
  virtual bool getPose()
  {
    if (base_.getPose(ActionTurn::pose_))
    {
      const complex v = std::polar<double>(1, ActionTurn::pose_.theta);
      const complex pose_v = complex(ActionTurn::pose_.x, ActionTurn::pose_.y) - path_end_;
      ActionStraight::pose_.x = dot(v, pose_v);
      ActionStraight::pose_.y = cross(v, pose_v);
      ActionStraight::pose_.theta = ActionTurn::getAngularError0();
      return true;
    }
    return false;
  }
  virtual float getTargetDistance0()
  {
    return -overshoot_;
  }
  virtual float getLinearError0()
  {
    return linear_error_;
  }
  virtual float getAngularError0()
  {
    return angular_error_;
  }
  virtual float getHeadingError0()
  {
    return heading_error_;
  }

protected:
  // straight
  VelocitySmoother<float>& speed_x_;
  bool& dir_x_;
  float& speed_x_max_;

  // turn
  VelocitySmoother<float>& speed_z_;
  bool& dir_z_;
  float speed_z_max_;

  // lateral
  VelocitySmoother<float> speed_y_;
  bool dir_y_;
  float speed_y_max_;

  float speed_ratio_;  // straight over turn
};

class ActionDockOmni: public ActionDock, public ActionTurnOmni
{
public:
  /**
   * Goal params:
   * - speed
   * - goal_tolerance
   * - enable_sensor
   * - marker_type
   * - path_end (target in marker frame)
   */
  ActionDockOmni(Nav& nav, const Goal& goal) :
    ActionManualControl(nav, goal),
    ActionStraight(nav, goal),
    ActionDock(nav, goal),
    ActionTurnOmni(nav, goal)
  {
    ROS_ASSERT(goal.nav == Goal::NAV_OMNI_DOCK);
  }

  void process(float frequency)
  {
    ActionTurnOmni::processAlign(frequency);
  }

protected:
  bool generatePlan()
  {
    return base_.getPoseInTarget(ActionTurn::pose_, true);
  }
  bool handlePoseUpdate()
  {
    return ActionTurnOmni::handlePoseUpdate();
  }
  bool getPose()
  {
    if (base_.getPoseInTarget(ActionTurn::pose_))
    {
      const complex v = std::polar<double>(1, ActionTurn::pose_.theta);
      const complex pose_v(ActionTurn::pose_.x, ActionTurn::pose_.y);
      ActionStraight::pose_.x = dot(v, pose_v);
      ActionStraight::pose_.y = cross(v, pose_v);
      ActionStraight::pose_.theta = ActionTurn::pose_.theta;
      return true;
    }
    return false;
  }
  float getTargetDistance0()
  {
    return -overshoot_;
  }
  float getLinearError0()
  {
    return linear_error_;
  }
  float getAngularError0()
  {
    return angular_error_;
  }
  float getHeadingError0()
  {
    return heading_error_;
  }
};

}  // namespace agv05

#endif  // AGV05_NAVX_ACTIONS_H
