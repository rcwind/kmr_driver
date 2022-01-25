/**
 * @file /kmr_driver/src/driver/diff_drive.cpp
 *
 * @brief Differential drive abstraction (brought in from ycs).
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kmr_driver/modules/diff_drive.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmr {

/*****************************************************************************
** Implementation
*****************************************************************************/
DiffDrive::DiffDrive() : wheel_bias(0.161),     // 左右轮距, wheel_to_wheel, in [m]
                         wheel_span(0),         // 前后轮距, wheel_to_wheel, in [m]
                         wheel_diameter(0.085), // 轮子直径, in [m]
                         wheel_ticks(3200),     // 轮子转一圈，编码器脉冲数
                         vehicle_type(diff2)    // 机器人运动模型，可选为diff2,diff4,ackerman1,ackerman2,mecanum,dualstee,quadstee
{
  imu_heading_offset = 0;

  point_velocity.resize(3); // command velocities, in [m/s] and [rad/s]

  speed_x = 0.0;
  speed_y = 0.0;
  speed_z = 0.0;

  last_velocity_left_front = 0.0;
  last_velocity_right_front = 0.0;
  last_velocity_left_rear = 0.0;
  last_velocity_right_rear = 0.0;

  last_tick_left_front = 0;
  last_tick_right_front = 0;
  last_tick_left_rear = 0;
  last_tick_right_rear = 0;

  last_rad_left_front = 0.0;
  last_rad_right_front = 0.0;
  last_rad_left_rear = 0.0;
  last_rad_right_rear = 0.0;

  ackerman_heading = 0.0;

  tick_to_rad = (double)2 * 3.1415926 / (double)wheel_ticks;

  if (vehicle_type == diff2)
    diff_drive_kinematics = new ecl::DifferentialDrive::Kinematics(wheel_bias, wheel_diameter / 2.0);
  else if (vehicle_type == diff4)
    diff_drive_kinematics = new ecl::DifferentialDrive::Kinematics(wheel_span + wheel_bias, wheel_diameter / 2.0);
}

/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param time_stamp
 * @param left_front_encoder
 * @param right_front_encoder
 * @param pose_update
 * @param pose_update_rates
 */
void DiffDrive::update(const uint16_t &time_stamp,
                       const uint16_t &left_encoder,
                       const uint16_t &right_encoder,
                       ecl::LegacyPose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates)
{
  state_mutex.lock();
  static bool init_lf = false;
  static bool init_rf = false;
  static bool init_lr = false;
  static bool init_rr = false;
  double left_diff_ticks = 0.0f;
  double right_diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
  unsigned short curr_timestamp = 0;
  curr_timestamp = time_stamp;
  //{{{ left front
  curr_tick_left = left_encoder;
  if (!init_lf)
  {
    last_tick_left_front = curr_tick_left;
    init_lf = true;
  }
  left_diff_ticks = (double)(short)((curr_tick_left - last_tick_left_front) & 0xffff);
  last_tick_left_front = curr_tick_left;
  last_rad_left_front += tick_to_rad * left_diff_ticks;
  ///}}}

  //{{{ right front
  curr_tick_right = right_encoder;
  if (!init_rf)
  {
    last_tick_right_front = curr_tick_right;
    init_rf = true;
  }
  right_diff_ticks = (double)(short)((curr_tick_right - last_tick_right_front) & 0xffff);
  last_tick_right_front = curr_tick_right;
  last_rad_right_front += tick_to_rad * right_diff_ticks;
  //}}}

  // TODO this line and the last statements are really ugly; refactor, put in another place
  if (vehicle_type == diff2)
    pose_update = diff_drive_kinematics->forward(tick_to_rad * left_diff_ticks, tick_to_rad * right_diff_ticks);

  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
    last_velocity_left_front = (tick_to_rad * left_diff_ticks) / last_diff_time;
    last_velocity_right_front = (tick_to_rad * right_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << pose_update.x()/last_diff_time,
                       pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
  state_mutex.unlock();
}
void DiffDrive::update(const uint16_t &time_stamp,
                       const uint16_t &left_front_encoder,
                       const uint16_t &right_front_encoder,
                       const uint16_t &left_rear_encoder,
                       const uint16_t &right_rear_encoder,
                       ecl::LegacyPose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates)
{
  state_mutex.lock();
  static bool init_lf = false;
  static bool init_rf = false;
  static bool init_lr = false;
  static bool init_rr = false;
  double left_front_diff_ticks = 0.0f;
  double right_front_diff_ticks = 0.0f;
  double left_rear_diff_ticks = 0.0f;
  double right_rear_diff_ticks = 0.0f;
  unsigned short curr_tick_left_front = 0;
  unsigned short curr_tick_right_front = 0;
  unsigned short curr_tick_left_rear = 0;
  unsigned short curr_tick_right_rear = 0;
  unsigned short curr_timestamp = 0;
  curr_timestamp = time_stamp;
  double d_theta;
  double d_x;
  double d_y;
  //{{{ left front
  curr_tick_left_front = left_front_encoder;
  if (!init_lf)
  {
    last_tick_left_front = curr_tick_left_front;
    init_lf = true;
  }
  left_front_diff_ticks = (double)(short)((curr_tick_left_front - last_tick_left_front) & 0xffff);
  last_tick_left_front = curr_tick_left_front;
  last_rad_left_front += tick_to_rad * left_front_diff_ticks;
  ///}}}

  //{{{ right front
  curr_tick_right_front = right_front_encoder;
  if (!init_rf)
  {
    last_tick_right_front = curr_tick_right_front;
    init_rf = true;
  }
  right_front_diff_ticks = (double)(short)((curr_tick_right_front - last_tick_right_front) & 0xffff);
  last_tick_right_front = curr_tick_right_front;
  last_rad_right_front += tick_to_rad * right_front_diff_ticks;
  //}}}

  //{{{ left rear
  curr_tick_left_rear = left_rear_encoder;
  if (!init_lr)
  {
    last_tick_left_rear = curr_tick_left_rear;
    init_lr = true;
  }
  left_rear_diff_ticks = (double)(short)((curr_tick_left_rear - last_tick_left_rear) & 0xffff);
  last_tick_left_rear = curr_tick_left_rear;
  last_rad_left_rear += tick_to_rad * left_rear_diff_ticks;
  ///}}}

  //{{{ right rear
  curr_tick_right_rear = right_rear_encoder;
  if (!init_rr)
  {
    last_tick_right_rear = curr_tick_right_rear;
    init_rr = true;
  }
  right_rear_diff_ticks = (double)(short)((curr_tick_right_rear - last_tick_right_rear) & 0xffff);
  last_tick_right_rear = curr_tick_right_rear;
  last_rad_right_rear += tick_to_rad * right_rear_diff_ticks;
  //}}}

  // TODO this line and the last statements are really ugly; refactor, put in another place
  if (vehicle_type == diff4)
    pose_update = diff_drive_kinematics->forward(tick_to_rad * (left_front_diff_ticks + left_rear_diff_ticks) / 2,
                                                tick_to_rad * (right_front_diff_ticks + right_rear_diff_ticks) / 2);
  else if (vehicle_type == mecanum)
  {
    d_x = (left_front_diff_ticks + right_front_diff_ticks + left_rear_diff_ticks + right_rear_diff_ticks) * tick_to_rad * (wheel_diameter / 2) / 4.0;
    d_y = (-left_front_diff_ticks + right_front_diff_ticks + left_rear_diff_ticks - right_rear_diff_ticks) * tick_to_rad * (wheel_diameter / 2) / 4.0;
    d_theta = (-left_front_diff_ticks + right_front_diff_ticks - left_rear_diff_ticks + right_rear_diff_ticks) * tick_to_rad * (wheel_diameter / 2) / (2 * (wheel_span + wheel_bias));
    pose_update.x(d_x); // 这里不想更改函数传递的参数，直接赋值方式，node里更新pose不能用乘法，乘法适用于差速车
    pose_update.y(d_y);
    pose_update.heading(d_theta);
  }

  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
    last_velocity_left_front = (tick_to_rad * left_front_diff_ticks) / last_diff_time;
    last_velocity_right_front = (tick_to_rad * right_front_diff_ticks) / last_diff_time;
    last_velocity_left_rear = (tick_to_rad * left_rear_diff_ticks) / last_diff_time;
    last_velocity_right_rear = (tick_to_rad * right_rear_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << pose_update.x()/last_diff_time,
                       pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
  state_mutex.unlock();
}
void DiffDrive::update(const uint16_t &time_stamp,
                       const uint16_t &left_encoder,
                       const uint16_t &right_encoder,
                       const int32_t &steering,
                       ecl::LegacyPose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates) {
  state_mutex.lock();
  static bool init_l = false;
  static bool init_r = false;
  double left_diff_ticks = 0.0f;
  double right_diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
  unsigned short curr_timestamp = 0;
  double steering_pos;
  double d_pos;
  double d_dist;
  double d_theta;
  double d_x;
  double d_y;

  curr_timestamp = time_stamp;

  // left
  curr_tick_left = left_encoder;
  if (!init_l)
  {
    last_tick_left_front = curr_tick_left;
    init_l = true;
  }
  left_diff_ticks = (double)(short)((curr_tick_left - last_tick_left_front) & 0xffff);
  last_tick_left_front = curr_tick_left;
  last_rad_left_front += tick_to_rad * left_diff_ticks;

  // right
  curr_tick_right = right_encoder;
  if (!init_r)
  {
    last_tick_right_front = curr_tick_right;
    init_r = true;
  }
  right_diff_ticks = (double)(short)((curr_tick_right - last_tick_right_front) & 0xffff);
  last_tick_right_front = curr_tick_right;
  last_rad_right_front += tick_to_rad * right_diff_ticks;

  // TODO this line and the last statements are really ugly; refactor, put in another place
  // pose_update = diff_drive_kinematics.forward(tick_to_rad * left_diff_ticks, tick_to_rad * right_diff_ticks);
  d_pos = (tick_to_rad * left_diff_ticks + tick_to_rad * right_diff_ticks) / 2;
  d_dist = d_pos * wheel_diameter / 2;// 弧长公式
  d_theta = tan(steering / 100.0 / 180.0 * 3.1415926) * d_dist / wheel_span;
#if 0
  d_x = sin(d_theta) * d_dist;
  d_y = cos(d_theta) * d_dist;
  d_x = d_x * cos(ackerman_heading) - d_y * sin(ackerman_heading);
  d_y = d_y * sin(ackerman_heading) + d_y * cos(ackerman_heading);
  ackerman_heading += d_theta;
#else
  // https://github.com/ros-controls/ros_controllers/blob/noetic-devel/ackermann_steering_controller/src/odometry.cpp
  if (fabs(d_theta) < 1e-6)
  {
      const double direction = ackerman_heading + d_theta * 0.5;

      /// Runge-Kutta 2nd order integration:
      d_x = d_dist  * cos(direction);
      d_y = d_theta * sin(direction);
      ackerman_heading += d_theta;
  }
  else
  {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = ackerman_heading;
      const double r = d_dist/d_theta;
      ackerman_heading += d_theta;
      d_x =  r * (sin(ackerman_heading) - sin(heading_old));
      d_y = -r * (cos(ackerman_heading) - cos(heading_old));
  }
#endif

  // https://github.com/stonier/ecl_core/blob/release/1.1.x/ecl_mobile_robot/src/lib/differential_drive.cpp
  // pose_update.translation(d_dist, 0);
  // pose_update.rotation(d_theta);
  pose_update.x(d_x);// 这里不想更改函数传递的参数，直接赋值方式，node里更新pose不能用乘法，乘法适用于差速车
  pose_update.y(d_y);
  pose_update.heading(d_theta);

  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
    last_velocity_left_front = (tick_to_rad * left_diff_ticks) / last_diff_time;
    last_velocity_right_front = (tick_to_rad * right_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << d_dist/last_diff_time,
                       0,
                       // pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
  state_mutex.unlock();
}
void DiffDrive::update(const uint16_t &time_stamp,
                       const uint16_t &left_encoder,
                       const uint16_t &right_encoder,
                       const int16_t &left_steering,
                       const int16_t &right_steering,
                       const double &heading,
                       const double &delta_heading,
                       ecl::LegacyPose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates) {
  state_mutex.lock();
  static bool init_l = false;
  static bool init_r = false;
  double left_diff_ticks = 0.0f;
  double right_diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
  unsigned short curr_timestamp = 0;
  double left_delta_s, right_delta_s;
  double left_delta_x, right_delta_x;
  double left_delta_y, right_delta_y;
  double left_steering_rad, right_steering_rad;
  double left_rad, right_rad;
  double delta_x, delta_y;
  static double x = 0, y = 0;
  curr_timestamp = time_stamp;
  curr_tick_left = left_encoder;
  int len, total_len;

  if (!init_l)
  {
    last_tick_left_front = curr_tick_left;
    init_l = true;
  }

  left_diff_ticks = (double)(short)((curr_tick_left - last_tick_left_front) & 0xffff);
  last_tick_left_front = curr_tick_left;
  last_rad_left_front += tick_to_rad * left_diff_ticks;

  curr_tick_right = right_encoder;
  if (!init_r)
  {
    last_tick_right_front = curr_tick_right;
    init_r = true;
  }
  right_diff_ticks = (double)(short)((curr_tick_right - last_tick_right_front) & 0xffff);
  last_tick_right_front = curr_tick_right;
  last_rad_right_front += tick_to_rad * right_diff_ticks;

  left_delta_s  = wheel_diameter / 2 * (tick_to_rad * left_diff_ticks);
  right_delta_s = wheel_diameter / 2 * (tick_to_rad * right_diff_ticks);

  left_steering_rad   = left_steering / 100.0 / 180.0 * M_PI;
  right_steering_rad  = right_steering / 100.0 / 180.0 * M_PI;

  left_rad = left_steering_rad + heading + delta_heading;
  right_rad = right_steering_rad + heading + delta_heading;

  total_len = 0;


  left_delta_x = left_delta_s * cos(left_rad);
  left_delta_y = left_delta_s * sin(left_rad);

  right_delta_x = right_delta_s * cos(right_rad);
  right_delta_y = right_delta_s * sin(right_rad);

  delta_x = (left_delta_x + right_delta_x) / 2.0;
  delta_y = (left_delta_y + right_delta_y) / 2.0;

  x += delta_x;
  y += delta_y;

#if 0
  len = sprintf(&print_buf[total_len], "speed:%f radius:%f: heading:%fdeg\n", speed, radius, heading / M_PI * 180);
  total_len += len;
  len = sprintf(&print_buf[total_len], "l:ds:%f rad:%f=ste:%f+yaw:%f+dh:%f\n", left_delta_s, left_rad, left_steering_rad, heading, delta_heading);
  total_len += len;
  len = sprintf(&print_buf[total_len], "r:ds:%f rad:%f=ste:%f+yaw:%f+dh:%f\n", right_delta_s, right_rad, right_steering_rad, heading, delta_heading);
  total_len += len;
  len = sprintf(&print_buf[total_len], "dx:%f=lx:%f+rx:%f, dy:%f=ly:%f+ry:%f\n", delta_x, left_delta_x, right_delta_x, delta_y, left_delta_y, right_delta_y);
  total_len += len;
  len = sprintf(&print_buf[total_len], "x:%f y:%f\n\n", x, y);
  total_len += len;

  write(file_fd, print_buf, total_len);
#endif

  pose_update.x(delta_x);
  pose_update.y(delta_y);
  pose_update.heading(delta_heading);
  // TODO this line and the last statements are really ugly; refactor, put in another place
  // pose_update = diff_drive_kinematics.forward(tick_to_rad * left_diff_ticks, tick_to_rad * right_diff_ticks);

  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
    last_velocity_left_front = (tick_to_rad * left_diff_ticks) / last_diff_time;
    last_velocity_right_front = (tick_to_rad * right_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << pose_update.x()/last_diff_time,
                       pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
  state_mutex.unlock();
}
void DiffDrive::update(const uint16_t &time_stamp,
                       const uint16_t &left_front_encoder,
                       const uint16_t &right_front_encoder,
                       const uint16_t &left_rear_encoder,
                       const uint16_t &right_rear_encoder,
                       const int16_t &left_front_steering,
                       const int16_t &right_front_steering,
                       const int16_t &left_rear_steering,
                       const int16_t &right_rear_steering,
                       const double &heading,
                       const double &delta_heading,
                       ecl::LegacyPose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates) {
}
void DiffDrive::reset() {
  state_mutex.lock();
  last_rad_left_front = 0.0;
  last_rad_right_front = 0.0;
  last_rad_left_rear = 0.0;
  last_rad_right_rear = 0.0;
  last_velocity_left_front = 0.0;
  last_velocity_right_front = 0.0;
  last_velocity_left_rear = 0.0;
  last_velocity_right_rear = 0.0;
  state_mutex.unlock();
}

void DiffDrive::getWheelJointStates(double &wheel_left_front_angle, double &wheel_left_front_angle_rate,
                                    double &wheel_right_front_angle, double &wheel_right_front_angle_rate,
                                    double &wheel_left_rear_angle, double &wheel_left_rear_angle_rate,
                                    double &wheel_right_rear_angle, double &wheel_right_rear_angle_rate) {
  state_mutex.lock();
  wheel_left_front_angle = last_rad_left_front;
  wheel_right_front_angle = last_rad_right_front;
  wheel_left_rear_angle = last_rad_left_rear;
  wheel_right_rear_angle = last_rad_right_rear;
  wheel_left_front_angle_rate = last_velocity_left_front;
  wheel_right_front_angle_rate = last_velocity_right_front;
  wheel_left_rear_angle_rate = last_velocity_left_rear;
  wheel_right_rear_angle_rate = last_velocity_right_rear;
  state_mutex.unlock();
}

void DiffDrive::setVelocityCommands(const double &vx, const double &vy, const double &wz) {
  // vx: in m/s
  // wz: in rad/s
  std::vector<double> cmd_vel;
  cmd_vel.push_back(vx);
  cmd_vel.push_back(vy); // velocity y = 0
  cmd_vel.push_back(wz);
  point_velocity = cmd_vel;
}

void DiffDrive::velocityCommands(const double &vx, const double &vy, const double &wz) {
  velocity_mutex.lock();
  speed_x = vx * 1000;
  speed_y = vy * 1000;
  speed_z = wz * 1000;
  velocity_mutex.unlock();
}

void DiffDrive::velocityCommands(const short &cmd_speed_x, const short &cmd_speed_y, const short &cmd_speed_z) {
  velocity_mutex.lock();
  speed_x = static_cast<double>(cmd_speed_x);   // In [mm/s]
  speed_y = static_cast<double>(cmd_speed_y); // In [mm/s]
  speed_z = static_cast<double>(cmd_speed_z); // In [0.001rad/s]
  velocity_mutex.unlock();
  return;
}

std::vector<short> DiffDrive::velocityCommands() {
  velocity_mutex.lock();
  std::vector<short> cmd(3);
  cmd[0] = bound(speed_x);  // In [mm/s]
  cmd[1] = bound(speed_y); // In [mm/s]
  cmd[2] = bound(speed_z); // In [0.001rad/s]
  velocity_mutex.unlock();
  return cmd;
}

std::vector<double> DiffDrive::pointVelocity() const {
  return point_velocity;
}

short DiffDrive::bound(const double &value) {
  if (value > static_cast<double>(SHRT_MAX)) return SHRT_MAX;
  if (value < static_cast<double>(SHRT_MIN)) return SHRT_MIN;
  return static_cast<short>(value);
}

} // namespace kmr
