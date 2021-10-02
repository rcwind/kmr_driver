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

  tick_to_rad = (double)2 * 3.1415926 / (double)wheel_ticks;

  if (vehicle_type == diff2)
    diff_drive_kinematics = new ecl::DifferentialDrive::Kinematics(wheel_bias, wheel_diameter / 2.0);
  else if (vehicle_type == diff4)
    diff_drive_kinematics = new ecl::DifferentialDrive::Kinematics((wheel_span + wheel_bias) / 2.0, wheel_diameter / 2.0);
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
  if (!init_lf)
  {
    last_tick_left_rear = curr_tick_left_rear;
    init_lf = true;
  }
  left_rear_diff_ticks = (double)(short)((curr_tick_left_rear - last_tick_left_rear) & 0xffff);
  last_tick_left_rear = curr_tick_left_rear;
  last_rad_left_rear += tick_to_rad * left_rear_diff_ticks;
  ///}}}

  //{{{ right rear
  curr_tick_right_rear = right_rear_encoder;
  if (!init_rf)
  {
    last_tick_right_rear = curr_tick_right_rear;
    init_rf = true;
  }
  right_rear_diff_ticks = (double)(short)((curr_tick_right_rear - last_tick_right_rear) & 0xffff);
  last_tick_right_rear = curr_tick_right_rear;
  last_rad_right_rear += tick_to_rad * right_rear_diff_ticks;
  //}}}

  // TODO this line and the last statements are really ugly; refactor, put in another place
  if (vehicle_type == diff2)
    pose_update = diff_drive_kinematics->forward(tick_to_rad * left_front_diff_ticks, tick_to_rad * right_front_diff_ticks);
  else if (vehicle_type == diff4)
    pose_update = diff_drive_kinematics->forward(tick_to_rad * (left_front_diff_ticks + left_rear_diff_ticks) / 2,
                                                tick_to_rad * (right_front_diff_ticks + right_rear_diff_ticks) / 2);
  else if (vehicle_type == ackerman1)
  {

  }
  else if (vehicle_type == ackerman2)
  {

  }
  else if (vehicle_type == mecanum)
  {

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
