/**
 * @file include/kmr_driver/packets/core_sensors.hpp
 *
 * @brief Core sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KMR_CORE_SENSORS_HPP__
#define KMR_CORE_SENSORS_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../macros.hpp"
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmr
{

/*****************************************************************************
** Interface
*****************************************************************************/

class kmr_PUBLIC CoreSensors : public packet_handler::payloadBase
{
public:
  CoreSensors() : packet_handler::payloadBase(false, 19) {};

  struct Data {
    uint16_t time_stamp;
    uint8_t vehicle;
    uint8_t bumper;
    uint8_t wheel_drop;
    uint8_t cliff;
    uint8_t charger_status;
    int16_t charger_current;
    uint16_t battery;
    uint16_t left_front_encoder;
    uint16_t right_front_encoder;
    uint16_t left_rear_encoder;
    uint16_t right_rear_encoder;
  } data;

  struct Flags {

    // bumper
    static const uint8_t LeftBumper   = 0x04;
    static const uint8_t CenterBumper = 0x02;
    static const uint8_t RightBumper  = 0x01;

    // cliff sensor
    static const uint8_t LeftCliff    = 0x04;
    static const uint8_t CenterCliff  = 0x02;
    static const uint8_t RightCliff   = 0x01;

    // wheel drop sensor
    static const uint8_t LeftWheel    = 0x02;
    static const uint8_t RightWheel   = 0x01;

    // Charging source
    // - first four bits distinguish between adapter or docking base charging
    static const uint8_t AdapterType  = 0x10;
    // - last 4 bits specified the charging status (see Battery.hpp for details)
    static const uint8_t BatteryStateMask = 0x0F;
    static const uint8_t Discharging  = 0x00;
    static const uint8_t Charged      = 0x02;
    static const uint8_t Charging     = 0x06;

  };

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace kmr

#endif /* KMR_CORE_SENSORS_HPP__ */
