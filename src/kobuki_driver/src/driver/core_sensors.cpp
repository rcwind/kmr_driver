/**
 * @file /kobuki_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/packets/core_sensors.hpp"
#include "../../include/kobuki_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  buildBytes(Header::CoreSensors, byteStream);
  buildBytes(length, byteStream);
  buildBytes(data.time_stamp, byteStream);	//2
  buildBytes(data.bumper, byteStream);		//1
  buildBytes(data.left_encoder, byteStream);	//2
  buildBytes(data.right_encoder, byteStream);	//2
  buildBytes(data.charger, byteStream);		//1
  buildBytes(data.battery, byteStream);		//2

  return true;
}
bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (byteStream.size() < length+2)
  {
    //std::cout << "kobuki_node: kobuki_default: deserialise failed. not enough byte stream." << std::endl;
    return false;
  }

  unsigned char header_id, length_packed;
  buildVariable(header_id, byteStream);
  buildVariable(length_packed, byteStream);
  if( header_id != Header::CoreSensors ) return false;
  if( length_packed != length ) return false;

  buildVariable(data.time_stamp, byteStream);
  buildVariable(data.bumper, byteStream);
  buildVariable(data.left_encoder, byteStream);
  buildVariable(data.right_encoder, byteStream);
  buildVariable(data.charger, byteStream);
  buildVariable(data.battery, byteStream);

  return true;
}



} // namespace kobuki
