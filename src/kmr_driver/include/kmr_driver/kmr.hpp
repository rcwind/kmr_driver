/**
 * @file include/kmr_driver/kmr.hpp
 *
 * @brief Device driver core interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KMR_HPP_
#define KMR_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <iomanip>
#include <ecl/config.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/geometry/legacy_pose2d.hpp>
#include "version_info.hpp"
#include "parameters.hpp"
#include "event_manager.hpp"
#include "command.hpp"
#include "modules.hpp"
#include "packets.hpp"
#include "packet_handler/packet_finder.hpp"
#include "macros.hpp"

/*****************************************************************************
** Extern Templates
*****************************************************************************/

#ifdef ECL_IS_WIN32
  /* Help windows create common instances of sigslots across kmr dll
   * and end user program (otherwise it creates two separate variables!) */
  EXP_TEMPLATE template class kmr_PUBLIC ecl::SigSlotsManager<>;
  EXP_TEMPLATE template class kmr_PUBLIC ecl::SigSlotsManager<const kmr::VersionInfo&>;
  EXP_TEMPLATE template class kmr_PUBLIC ecl::SigSlotsManager<const std::string&>;
  EXP_TEMPLATE template class kmr_PUBLIC ecl::SigSlotsManager<kmr::Command::Buffer&>;
  EXP_TEMPLATE template class kmr_PUBLIC ecl::SigSlotsManager<kmr::PacketFinderBase::BufferType&>;
#endif

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kmr
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/

union union_sint16
{
  short word;
  unsigned char byte[2];
};

/*****************************************************************************
** Parent Interface
*****************************************************************************/

class PacketFinder : public PacketFinderBase
{
public:
  virtual ~PacketFinder() {}
  bool checkSum();
};

/*****************************************************************************
 ** Interface [Kmr]
 *****************************************************************************/
/**
 * @brief  The core kmr driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class kmr_PUBLIC Kmr
{
public:
  Kmr();
  ~Kmr();

  /*********************
   ** Configuration
   **********************/
  void init(Parameters &parameters) throw (ecl::StandardException);
  bool isAlive() const { return is_alive; } /**< Whether the connection to the robot is alive and currently streaming. **/
  bool isShutdown() const { return shutdown_requested; } /**< Whether the worker thread is alive or not. **/
  bool isEnabled() const { return is_enabled; } /**< Whether the motor power is enabled or disabled. **/
  bool enable(); /**< Enable power to the motors. **/
  bool disable(); /**< Disable power to the motors. **/
  void shutdown() { shutdown_requested = true; } /**< Gently terminate the worker thread. **/

  /******************************************
  ** Packet Processing
  *******************************************/
  void spin();
  void fixPayload(ecl::PushAndPop<unsigned char> & byteStream);

  /******************************************
  ** Getters - Data Protection
  *******************************************/
  void lockDataAccess();
  void unlockDataAccess();

  /******************************************
  ** Getters - User Friendly Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  ecl::Angle<double> getHeading() const;
  double getAngularVelocity() const;
  VersionInfo versionInfo() const { return VersionInfo(firmware.data.version, hardware.data.version, unique_device_id.data.udid0, unique_device_id.data.udid1, unique_device_id.data.udid2); }
  Battery batteryStatus() const { return Battery(core_sensors.data.battery, core_sensors.data.charger_status); }

  /******************************************
  ** Getters - Raw Data Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  Steering::Data getSteeringData() const { return steering.data; }
  CoreSensors::Data getCoreSensorData() const { return core_sensors.data; }
  Cliff::Data getCliffData() const { return cliff.data; }
  Inertia::Data getInertiaData() const { return inertia.data; }
  GpInput::Data getGpInputData() const { return gp_input.data; }
  ThreeAxisGyro::Data getRawInertiaData() const { return three_axis_gyro.data; }
  Ultrasonic::Data getUltrasonicData() const { return ultrasonic.data; }

  /*********************
  ** Feedback
  **********************/
  void getWheelJointStates(double &wheel_left_front_angle, double &wheel_left_front_angle_rate,
                           double &wheel_right_front_angle, double &wheel_right_front_angle_rate,
                           double &wheel_left_rear_angle, double &wheel_left_rear_angle_rate,
                           double &wheel_right_rear_angle, double &wheel_right_rear_angle_rate);
  void updateOdometry(ecl::LegacyPose2D<double> &pose_update,
                      ecl::linear_algebra::Vector3d &pose_update_rates);

  /*********************
  ** Soft Commands
  **********************/
  void resetOdometry();

  /*********************
  ** Hard Commands
  **********************/
  void setMagTracker(const unsigned char &action);
  void setDock(const unsigned char &dock);
  void setBaseControl(const double &linear_velocity_x, const double &linear_velocity_y, const double &angular_velocity);
  void setLed(const int index, const int number, const unsigned char colour[3]);
  void setDigitalOutput(const DigitalOutput &digital_output);
  void setExternalPower(const DigitalOutput &digital_output);
  void playSoundSequence(const enum SoundSequences &number);

  /*********************
  ** Debugging
  **********************/
  void printSigSlotConnections() const;

private:
  /*********************
  ** Thread
  **********************/
  ecl::Thread thread;
  bool shutdown_requested; // helper to shutdown the worker thread.

  /*********************
  ** Odometry
  **********************/
  DiffDrive diff_drive;
  bool is_enabled;

  /*********************
  ** Inertia
  **********************/
  double heading_offset;

  /*********************
  ** Driver Paramters
  **********************/
  Parameters parameters;
  bool is_connected;

  /*********************
  ** Acceleration Limiter
  **********************/
  AccelerationLimiter acceleration_limiter;

  /*********************
  ** Packet Handling
  **********************/
  CoreSensors core_sensors;
  Steering steering;
  Ultrasonic ultrasonic;
  Inertia inertia;
  Cliff cliff;
  GpInput gp_input;
  Hardware hardware; // requestable
  Firmware firmware; // requestable
  UniqueDeviceID unique_device_id; // requestable
  ThreeAxisGyro three_axis_gyro;

  ecl::Serial serial;
  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  bool is_alive; // used as a flag set by the data stream watchdog

  int version_info_reminder;

  /*********************
  ** Commands
  **********************/
  void sendBaseControlCommand();
  void sendCommand(Command command);
  ecl::Mutex command_mutex; // protection against the user calling the command functions from multiple threads
  // data_mutex is protection against reading and writing data structures simultaneously as well as
  // ensuring multiple get*** calls are synchronised to the same data update
  // refer to https://github.com/yujinrobot/kmr/issues/240
  ecl::Mutex data_mutex;
  Command kmr_command; // used to maintain some state about the command history
  Command::Buffer command_buffer;
  std::vector<short> velocity_commands_debug;

  /*********************
  ** Events
  **********************/
  EventManager event_manager;

  /*********************
  ** Logging
  **********************/
  std::vector<std::string> log(std::string msg) { return log("", "", msg); }
  std::vector<std::string> log(std::string level, std::string msg) { return log(level, "", msg); }
  std::vector<std::string> log(std::string level, std::string name, std::string msg) {
    std::vector<std::string> ret;
    if( level != "" ) ret.push_back(level);
    if( name != "" ) ret.push_back(name);
    if( msg != "" ) ret.push_back(msg);
    return ret;
  }

  /*********************
  ** Signals
  **********************/
  ecl::Signal<> sig_stream_data;
  ecl::Signal<const VersionInfo&> sig_version_info;
  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;
  ecl::Signal<const std::vector<std::string>&> sig_named;
  ecl::Signal<Command::Buffer&> sig_raw_data_command; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<PacketFinder::BufferType&> sig_raw_data_stream; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<const std::vector<short>&> sig_raw_control_command;
};

} // namespace kmr

#endif /* KMR_HPP_ */
