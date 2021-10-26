/**
 * @file include/kmr_driver/command.hpp
 *
 * @brief Command structure.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 **/
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KMR_COMMAND_DATA_HPP__
#define KMR_COMMAND_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include "modules.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/


namespace kmr
{

class kmr_PUBLIC Command : public packet_handler::payloadBase
{
public:
  typedef ecl::PushAndPop<unsigned char> Buffer;
  typedef ecl::Stencil< Buffer > BufferStencil;

  /**
   * These values are used to detect the type of sub-payload that is ensuing.
   */
  enum Name
  {
    BaseControl = 1, Sound = 3, SoundSequence = 4, RequestExtra = 9, ChangeFrame = 10, RequestEeprom = 11,
    SetDigitalOut = 12, Dock = 13, SetLed = 14
  };

  enum VersionFlag
  {
    HardwareVersion = 0x01, FirmwareVersion = 0x02/*, Time = 0x04*/, UniqueDeviceID = 0x08
  };

  /**
   * @brief Data structure containing data for commands.
   *
   * It is important to keep this
   * state as it will have to retain knowledge of the last known command in
   * some instances - e.g. for gp_out commands, quite often the incoming command
   * is only to set the output for a single led while keeping the rest of the current
   * gp_out values as is.
   *
   * For generating individual commands we modify the data here, then copy the command
   * class (avoid doing mutexes) and spin it off for sending down to the device.
   */
  struct Data
  {
    Data()
      : command(BaseControl), speed_x(0), speed_y(0), speed_z(0), request_flags(0), gp_out(0x00f0) // set all the power pins high, others low.
      , dock(0) 
    {
    }

    Name command;

    uint16_t led_start_index;
    uint16_t led_count;
    char led_color[3];

    // BaseControl
    int16_t speed_x;
    int16_t speed_y;
    int16_t speed_z;

    // Sound - not yet implemented
    uint16_t note;
    unsigned char duration;

    // SoundSequence
    // 0 - turning on, 1 - turn off, 2 - recharge start, 3 - press button,
    // 4 - error sound, 5 - start cleaning, 6 - cleaning end
    unsigned char segment_name;

    // RequestExtra (version flags)
    uint16_t request_flags;

    // ChangeFrame & RequestEeprom
    unsigned char frame_id;

    // SetDigitalOut
    // 0xff - digital output pins 0-7 (0x0001, 0x0002, 0x0004, 0x0008)
    uint8_t gp_out;

    // dock
    unsigned char dock;

    // SetControllerGain
    unsigned char reserved;
  };

  virtual ~Command() {}

  static Command SetLedArray(const uint16_t start_index, const uint16_t number, const unsigned char rgb_colour[3], Command::Data &current_data);
  static Command SetDigitalOutput(const DigitalOutput &digital_output, Command::Data &current_data);
  static Command SetExternalPower(const DigitalOutput &digital_output, Command::Data &current_data);
  static Command PlaySoundSequence(const enum SoundSequences &number, Command::Data &current_data);
  static Command GetVersionInfo();
  static Command SetVelocityControl(DiffDrive& diff_drive);
  static Command SetVelocityControl(const int16_t &speed_x, const int16_t &speed_y, const int16_t &speed_z);
  static Command SetDock(const unsigned char &dock);
  static Command SetMagTracker(const unsigned char &action);

  Data data;

  void resetBuffer(Buffer &buffer);
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream) { return true; } /**< Unused **/

private:
  static const unsigned char header0;
  static const unsigned char header1;

};

} // namespace kmr

#endif /* KMR_COMMAND_DATA_HPP__ */

