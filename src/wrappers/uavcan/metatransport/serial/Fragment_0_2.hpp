/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef ARDUINO_TRANSFER_uavcan_metatransport_serial_Fragment_0_HPP_
#define ARDUINO_TRANSFER_uavcan_metatransport_serial_Fragment_0_HPP_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <libcanard/canard.h>

#include <types/uavcan/metatransport/serial/Fragment_0_2.h>

#include <utility/convert.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace uavcan {
namespace metatransport {
namespace serial {
/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

template <CanardPortID ID = 5555 > //FIXME https://forum.uavcan.org/t/automatic-configuration-of-port-identifiers/840/6
class Fragment_0_2
{

public:

  uavcan_metatransport_serial_Fragment_0_2 data;

  static constexpr CanardPortID       PORT_ID = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = uavcan_metatransport_serial_Fragment_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND = CanardTransferKindMessage;

  Fragment_0_2()
  {
    uavcan_metatransport_serial_Fragment_0_2_initialize_(&data);
  }

  Fragment_0_2(Fragment_0_2 const & other)
  {
    memcpy(&data, &other.data, sizeof(data));
  }

  static Fragment_0_2 deserialize(CanardTransfer const & transfer)
  {
    Fragment_0_2 h;
    size_t inout_buffer_size_bytes = transfer.payload_size;
    uavcan_metatransport_serial_Fragment_0_2_deserialize_(&h.data, (uint8_t *)(transfer.payload), &inout_buffer_size_bytes);
    return h;
  }

  size_t serialize(uint8_t * payload) const
  {
    size_t inout_buffer_size_bytes = Fragment_0_2::MAX_PAYLOAD_SIZE;
    return (uavcan_metatransport_serial_Fragment_0_2_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS) ? 0 : inout_buffer_size_bytes;
  }

  // void operator = (Health const health)
  // {
  //   data.health.value = arduino::_107_::uavcan::to_integer(health);
  // }

  // void operator = (Mode const mode)
  // {
  //   data.mode.value = arduino::_107_::uavcan::to_integer(mode);
  // }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* uavcan */
} /* metatransport */
} /*serial*/

#endif /* ARDUINO_TRANSFER_uavcan_metatransport_serial_Fragment_0_HPP_ */
