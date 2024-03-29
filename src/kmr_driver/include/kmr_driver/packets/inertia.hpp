/**
 * @file /include/kmr_driver/packets/inertia.hpp
 *
 * @brief Inertia packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KMR_INERTIA_DATA_HPP__
#define KMR_INERTIA_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmr
{

/*****************************************************************************
** Interface
*****************************************************************************/

class Inertia : public packet_handler::payloadBase
{
public:
  Inertia() : packet_handler::payloadBase(false, 4) {};
  struct Data {
    int16_t angle;
    int16_t angle_rate;
  } data;

  virtual ~Inertia() {};

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    buildBytes(Header::Inertia, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.angle, byteStream);
    buildBytes(data.angle_rate, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kmr_node: kmr_inertia: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::Inertia ) return false;
    if( length_packed != length ) return false;

    buildVariable(data.angle, byteStream);
    buildVariable(data.angle_rate, byteStream);

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
  }
};

} // namespace kmr

#endif /* KMR_INERTIA_DATA_HPP__ */

