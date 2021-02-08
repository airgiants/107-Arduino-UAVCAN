// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.0.0 (serialization was enabled)
// Source file:   /home/alex/projects/107-systems/public_regulated_data_types/reg/drone/service/battery/Error.0.1.uavcan
// Generated at:  2021-02-08 09:43:03.735464 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.drone.service.battery.Error
// Version:       0.1
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  True

#ifndef REG_DRONE_SERVICE_BATTERY_ERROR_0_1_INCLUDED_
#define REG_DRONE_SERVICE_BATTERY_ERROR_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/alex/projects/107-systems/public_regulated_data_types/reg/drone/service/battery/Error.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/alex/projects/107-systems/public_regulated_data_types/reg/drone/service/battery/Error.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 1,
              "/home/alex/projects/107-systems/public_regulated_data_types/reg/drone/service/battery/Error.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_service_battery_Error_0_1_HAS_FIXED_PORT_ID_ false

#define reg_drone_service_battery_Error_0_1_FULL_NAME_             "reg.drone.service.battery.Error"
#define reg_drone_service_battery_Error_0_1_FULL_NAME_AND_VERSION_ "reg.drone.service.battery.Error.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_service_battery_Error_0_1_EXTENT_BYTES_                    1UL
#define reg_drone_service_battery_Error_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 1UL
static_assert(reg_drone_service_battery_Error_0_1_EXTENT_BYTES_ >= reg_drone_service_battery_Error_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated uint8 NONE = 0
#define reg_drone_service_battery_Error_0_1_NONE (0U)
/// saturated uint8 BAD_BATTERY = 10
#define reg_drone_service_battery_Error_0_1_BAD_BATTERY (10U)
/// saturated uint8 NEEDS_SERVICE = 11
#define reg_drone_service_battery_Error_0_1_NEEDS_SERVICE (11U)
/// saturated uint8 BMS_ERROR = 20
#define reg_drone_service_battery_Error_0_1_BMS_ERROR (20U)
/// saturated uint8 CONFIGURATION = 30
#define reg_drone_service_battery_Error_0_1_CONFIGURATION (30U)
/// saturated uint8 OVERDISCHARGE = 50
#define reg_drone_service_battery_Error_0_1_OVERDISCHARGE (50U)
/// saturated uint8 OVERLOAD = 51
#define reg_drone_service_battery_Error_0_1_OVERLOAD (51U)
/// saturated uint8 CELL_OVERVOLTAGE = 60
#define reg_drone_service_battery_Error_0_1_CELL_OVERVOLTAGE (60U)
/// saturated uint8 CELL_UNDERVOLTAGE = 61
#define reg_drone_service_battery_Error_0_1_CELL_UNDERVOLTAGE (61U)
/// saturated uint8 CELL_COUNT = 62
#define reg_drone_service_battery_Error_0_1_CELL_COUNT (62U)
/// saturated uint8 TEMPERATURE_HOT = 100
#define reg_drone_service_battery_Error_0_1_TEMPERATURE_HOT (100U)
/// saturated uint8 TEMPERATURE_COLD = 101
#define reg_drone_service_battery_Error_0_1_TEMPERATURE_COLD (101U)

typedef struct
{
    /// saturated uint8 value
    uint8_t value;
} reg_drone_service_battery_Error_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_service_battery_Error_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_battery_Error_0_1_serialize_(
    const reg_drone_service_battery_Error_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    const size_t capacity_bytes = *inout_buffer_size_bytes;
    if ((8U * (size_t) capacity_bytes) < 8UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;

    {   // saturated uint8 value
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->value);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad0_ > 0);
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += _pad0_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.

    NUNAVUT_ASSERT(offset_bits == 8ULL);

    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_battery_Error_0_1_deserialize_(
    reg_drone_service_battery_Error_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;

    // saturated uint8 value
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->value = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->value = 0U;
    }
    offset_bits += 8U;

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);
    NUNAVUT_ASSERT(capacity_bytes >= *inout_buffer_size_bytes);

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_service_battery_Error_0_1_initialize_(reg_drone_service_battery_Error_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_service_battery_Error_0_1_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}

#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_SERVICE_BATTERY_ERROR_0_1_INCLUDED_