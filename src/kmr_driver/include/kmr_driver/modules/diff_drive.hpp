/**
 * @file /kmr_driver/include/kmr_driver/modules/diff_drive.hpp
 *
 * @brief Simple module for the diff drive odometry.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KMR_DIFF_DRIVE_HPP_
#define KMR_DIFF_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <climits>
#include <stdint.h>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/mobile_robot.hpp>
#include <ecl/threads/mutex.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmr {

/*****************************************************************************
** Interfaces
*****************************************************************************/
#define VEHICLE_TYPE_LIST(F) \
	F(unkown) \
  F(diff2) \
  F(diff4) \
  F(ackerman1) \
  F(ackerman2) \
	F(mecanum) \
  F(dualstee) \
  F(quadstee) \

enum vehicle_type
{
#define DEF(a) a,
    VEHICLE_TYPE_LIST(DEF)
	VEHICLE_TYPE_NUM
#undef DEF
};

class kmr_PUBLIC DiffDrive {
public:
  DiffDrive();
  ~DiffDrive() {
    if (diff_drive_kinematics != nullptr)
      delete diff_drive_kinematics;
  }
  const ecl::DifferentialDrive::Kinematics& kinematics() { return *diff_drive_kinematics; }

  void update(const uint16_t &time_stamp,
              const uint16_t &left_encoder,
              const uint16_t &right_encoder,
              ecl::LegacyPose2D<double> &pose_update,
              ecl::linear_algebra::Vector3d &pose_update_rates);

  void update(const uint16_t &time_stamp,
              const uint16_t &left_front_encoder,
              const uint16_t &right_front_encoder,
              const uint16_t &left_rear_encoder,
              const uint16_t &right_rear_encoder,
              ecl::LegacyPose2D<double> &pose_update,
              ecl::linear_algebra::Vector3d &pose_update_rates);

  void update(const uint16_t &time_stamp,
              const uint16_t &left_encoder,
              const uint16_t &right_encoder,
              const int32_t &steering,
              ecl::LegacyPose2D<double> &pose_update,
              ecl::linear_algebra::Vector3d &pose_update_rates);

  void update(const uint16_t &time_stamp,
              const uint16_t &left_encoder,
              const uint16_t &right_encoder,
              const int16_t &left_steering,
              const int16_t &right_steering,
              const double &heading,
              const double &delta_heading,
              ecl::LegacyPose2D<double> &pose_update,
              ecl::linear_algebra::Vector3d &pose_update_rates);
  void update(const uint16_t &time_stamp,
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
              ecl::linear_algebra::Vector3d &pose_update_rates);
  void reset();
  void getWheelJointStates(double &wheel_left_front_angle, double &wheel_left_front_angle_rate,
                           double &wheel_right_front_angle, double &wheel_right_angle_front_rate,
                           double &wheel_left_rear_angle, double &wheel_left_angle_rear_rate,
                           double &wheel_right_rear_angle, double &wheel_right_angle_rear_rate);
  void setVelocityCommands(const double &vx, const double &vy, const double &wz);
  void velocityCommands(const double &vx, const double &vy, const double &wz);
  void velocityCommands(const short &speed_x, const short &speed_y, const short &speed_z);
  void velocityCommands(const std::vector<double> &cmd) { velocityCommands(cmd[0], cmd[1], cmd[2]); }
  void velocityCommands(const std::vector<short>  &cmd) { velocityCommands(cmd[0], cmd[1], cmd[2]); }

  /*********************
  ** Command Accessors
  **********************/
  std::vector<short> velocityCommands(); // (speed, radius), in [mm/s] and [mm]
  std::vector<double> pointVelocity() const; // (vx, wz), in [m/s] and [rad/s]

  int get_vehicle_type(void)
  {
    return vehicle_type;
  }

  /*********************
  ** Property Accessors
  **********************/
  // double wheel_bias() const { return bias; }

private:
  unsigned short last_timestamp;
  double last_velocity_left_front, last_velocity_right_front;
  double last_velocity_left_rear, last_velocity_right_rear;
  double last_diff_time;

  unsigned short last_tick_left_front, last_tick_right_front;
  unsigned short last_tick_left_rear, last_tick_right_rear;
  double last_rad_left_front, last_rad_right_front;
  double last_rad_left_rear, last_rad_right_rear;

  //double v, w; // in [m/s] and [rad/s]
  std::vector<double> point_velocity; // (vx, wz), in [m/s] and [rad/s]
  double speed_x, speed_y, speed_z; // in [mm/s, mm/s, 0.001rad/s]
  double wheel_bias; //wheelbase, wheel_to_wheel, in [m]
  double wheel_span;
  double wheel_diameter; // in [m]
  int wheel_ticks;
  int vehicle_type;
  double ackerman_heading;
  int imu_heading_offset;
  double tick_to_rad;

  ecl::DifferentialDrive::Kinematics *diff_drive_kinematics = nullptr;
  ecl::Mutex velocity_mutex, state_mutex;

  // Utility
  short bound(const double &value);
};

} // namespace kmr

#endif /* KMR_DIFF_DRIVE_HPP_ */
