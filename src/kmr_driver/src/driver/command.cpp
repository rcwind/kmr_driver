/**
 * @file src/driver/command.cpp
 *
 * @brief Implementation of the command packets.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kmr_driver/command.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmr {

/*****************************************************************************
** Static variables initialization
*****************************************************************************/

const unsigned char Command::header0 = 0xaa;
const unsigned char Command::header1 = 0x55;

/*****************************************************************************
** Implementation [Command Generators]
*****************************************************************************/

Command Command::SetLedArray(const uint16_t start_index, const uint16_t number, const unsigned char rgb_colour[3], Command::Data &current_data)
{
  // gp_out is 16 bits
  uint16_t value;
  Command outgoing;
  outgoing.data.led_start_index = start_index;
  outgoing.data.led_count = number;
  outgoing.data.led_color[0] = rgb_colour[0];
  outgoing.data.led_color[1] = rgb_colour[1];
  outgoing.data.led_color[2] = rgb_colour[2];
  outgoing.data.command = Command::SetLed;
  return outgoing;
}

/**
 * Set one of the digital out pins available to the user.
 *
 * They set the last 4 bits on the data.gp_out variable.
 *
 * @todo could use far better documentation here/example here.
 *
 * @param digital_output : mask and value to send
 * @param current_data : need to store settings as the gp_output command is a combo command
 * @return Command : the command to send down the wire.
 */
Command Command::SetDigitalOutput(const DigitalOutput &digital_output, Command::Data &current_data)
{
  uint8_t values = 0x00;
  uint8_t clear_mask = 0x00;
  for ( unsigned int i = 0; i < 8; ++i ) {
    if ( digital_output.mask[i] ) {
      if ( digital_output.values[i] ) {
        values |= ( 1 << i );
      }
    } else {
      clear_mask |= ( 1 << i ); // don't clear this bit, so set a 1 here
    }
  }
  current_data.gp_out = (current_data.gp_out & clear_mask) | values;
  Command outgoing;
  outgoing.data = current_data;
  outgoing.data.command = Command::SetDigitalOut;
  return outgoing;
}

/**
 * Set one of the external power sources available to the user.
 *
 * They set the second 4 bits(0x00f0) on the data.gp_out variable.
 *
 * @todo could use far better documentation here/example here.
 *
 * @param digital_output : mask and value to send
 * @param current_data : need to store settings as the gp_output command is a combo command
 * @return Command : the command to send down the wire.
 */
Command Command::SetExternalPower(const DigitalOutput &digital_output, Command::Data &current_data)
{
  uint16_t values = 0x0000;
  uint16_t clear_mask = 0xff0f;
  for ( unsigned int i = 0; i < 4; ++i ) {
    if ( digital_output.mask[i] ) {
      if ( digital_output.values[i] ) {
        values |= ( 1 << (i+4) );
      }
    } else {
      clear_mask |= ( 1 << (i+4) ); // don't clear this bit, so set a 1 here
    }
  }
  current_data.gp_out = (current_data.gp_out & clear_mask) | values;
  Command outgoing;
  outgoing.data = current_data;
  outgoing.data.command = Command::SetDigitalOut;
  return outgoing;
}
Command Command::SetMagTracker(const unsigned char &action)
{
  Command outgoing;
  outgoing.data.segment_name = action;
  outgoing.data.command = Command::SoundSequence;
  return outgoing;
}
Command Command::PlaySoundSequence(const enum SoundSequences &number, Command::Data & /* current_data */)
{
  uint16_t value; // gp_out is 16 bits
  value = number; // defined with the correct bit specification.

  Command outgoing;
  outgoing.data.segment_name = value;
  outgoing.data.command = Command::SoundSequence;
  return outgoing;
}

Command Command::GetVersionInfo()
{
  Command outgoing;
  outgoing.data.request_flags = 0;
  outgoing.data.request_flags |= static_cast<uint16_t>(HardwareVersion);
  outgoing.data.request_flags |= static_cast<uint16_t>(FirmwareVersion);
  outgoing.data.request_flags |= static_cast<uint16_t>(UniqueDeviceID);
  outgoing.data.command = Command::RequestExtra;
  return outgoing;
}

Command Command::SetVelocityControl(DiffDrive& diff_drive)
{
  Command outgoing;
  std::vector<short> velocity_commands = diff_drive.velocityCommands();
  outgoing.data.speed_x = velocity_commands[0];
  outgoing.data.speed_y = velocity_commands[1];
  outgoing.data.speed_z = velocity_commands[2];
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

Command Command::SetVelocityControl(const int16_t &speed_x, const int16_t &speed_y, const int16_t &speed_z)
{
  Command outgoing;
  outgoing.data.speed_x = speed_x;
  outgoing.data.speed_y = speed_y;
  outgoing.data.speed_z = speed_z;
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

Command Command::SetDock(const unsigned char &dock)
{
  Command outgoing;
  outgoing.data.dock = dock;
  outgoing.data.command = Command::Dock;
  return outgoing;
}

/*****************************************************************************
** Implementation [Serialisation]
*****************************************************************************/
/**
 * Clears the command buffer and resets the header.
 */
void Command::resetBuffer(Buffer& buffer) {
  buffer.clear();
  buffer.resize(64);
  buffer.push_back(Command::header0);
  buffer.push_back(Command::header1);
  buffer.push_back(0); // just initialise, we usually write in the payload here later (size of payload only, not stx, not etx, not length)
}

bool Command::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  // need to be sure we don't pass through an emum to the Trans'd buildBytes functions.
  unsigned char cmd = static_cast<unsigned char>(data.command);
  unsigned char length = 0;
  switch (data.command)
  {
    case BaseControl:
      buildBytes(cmd, byteStream);
      buildBytes(length=6, byteStream);
      buildBytes(data.speed_x, byteStream);
      buildBytes(data.speed_y, byteStream);
      buildBytes(data.speed_z, byteStream);
      break;
    case Sound:
      buildBytes(cmd, byteStream);
      buildBytes(length=3, byteStream);
      buildBytes(data.note, byteStream);
      buildBytes(data.duration, byteStream);
      break;
    case SoundSequence:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.segment_name, byteStream);
      break;
    case RequestExtra:
      buildBytes(cmd, byteStream);
      buildBytes(length=2, byteStream);
      buildBytes(data.request_flags, byteStream);
      break;
    case ChangeFrame:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.frame_id, byteStream);
      break;
    case RequestEeprom:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.frame_id, byteStream);
      break;
    case SetDigitalOut:
    { // this one controls led, external power sources, gp digitial output
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.gp_out, byteStream);
      break;
    }
    case SetLed:
    { // this one controls led, external power sources, gp digitial output
      buildBytes(cmd, byteStream);
      buildBytes(length=7, byteStream);
      buildBytes(data.led_start_index, byteStream);
      buildBytes(data.led_count, byteStream);
      buildBytes(data.led_color[0], byteStream);
      buildBytes(data.led_color[1], byteStream);
      buildBytes(data.led_color[2], byteStream);
      break;
    }
    case Dock:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.dock, byteStream);
      break;
    default:
      return false;
      break;
  }
  return true;
}



} // namespace kmr
