/**
 * @file /kmr_driver/include/kmr_driver/modules/battery.hpp
 *
 * @brief Human friendly batter indicator class.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kmr_core/hydro-devel/kmr_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KMR_BATTERY_HPP_
#define KMR_BATTERY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include "../packets/core_sensors.hpp"
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmr {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Battery level module.
 *
 * Currently hard codes the battery status. It might be useful to provide
 * some configurable parameters for this module in the future.
 **/
class kmr_PUBLIC Battery {
public:
  enum Source {
    None,
    Adapter,
    Dock
  };
  enum Level {
    Dangerous,
    Low,
    Healthy,
    Maximum
  };
  enum State {
    Discharging,
    Charged,
    Charging
  };

  Battery() {} /**< Default constructor. **/
  Battery (const uint8_t &new_voltage, const uint8_t &charger_flag);
  Level level() const;
  float percent() const;

  static double capacity;
  static double low;
  static double dangerous;
  double voltage;
  State charging_state;
  Source charging_source;
};

} // namespace kmr

#endif /* KMR_BATTERY_HPP_ */
