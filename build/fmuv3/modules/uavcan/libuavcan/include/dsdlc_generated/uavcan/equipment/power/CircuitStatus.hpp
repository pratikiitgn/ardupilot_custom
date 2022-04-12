/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/pixhawk/github_ws/ardupilot_custom/modules/uavcan/dsdl/uavcan/equipment/power/1091.CircuitStatus.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Generic electrical circuit info.
#

uint16 circuit_id

float16 voltage
float16 current

uint8 ERROR_FLAG_OVERVOLTAGE  = 1
uint8 ERROR_FLAG_UNDERVOLTAGE = 2
uint8 ERROR_FLAG_OVERCURRENT  = 4
uint8 ERROR_FLAG_UNDERCURRENT = 8
uint8 error_flags
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.power.CircuitStatus
saturated uint16 circuit_id
saturated float16 voltage
saturated float16 current
saturated uint8 error_flags
******************************************************************************/

#undef circuit_id
#undef voltage
#undef current
#undef error_flags
#undef ERROR_FLAG_OVERVOLTAGE
#undef ERROR_FLAG_UNDERVOLTAGE
#undef ERROR_FLAG_OVERCURRENT
#undef ERROR_FLAG_UNDERCURRENT

namespace uavcan
{
namespace equipment
{
namespace power
{

template <int _tmpl>
struct UAVCAN_EXPORT CircuitStatus_
{
    typedef const CircuitStatus_<_tmpl>& ParameterType;
    typedef CircuitStatus_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_FLAG_OVERVOLTAGE;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_FLAG_UNDERVOLTAGE;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_FLAG_OVERCURRENT;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ERROR_FLAG_UNDERCURRENT;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > circuit_id;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > voltage;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > current;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > error_flags;
    };

    enum
    {
        MinBitLen
            = FieldTypes::circuit_id::MinBitLen
            + FieldTypes::voltage::MinBitLen
            + FieldTypes::current::MinBitLen
            + FieldTypes::error_flags::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::circuit_id::MaxBitLen
            + FieldTypes::voltage::MaxBitLen
            + FieldTypes::current::MaxBitLen
            + FieldTypes::error_flags::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_FLAG_OVERVOLTAGE >::Type ERROR_FLAG_OVERVOLTAGE; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_FLAG_UNDERVOLTAGE >::Type ERROR_FLAG_UNDERVOLTAGE; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_FLAG_OVERCURRENT >::Type ERROR_FLAG_OVERCURRENT; // 4
    static const typename ::uavcan::StorageType< typename ConstantTypes::ERROR_FLAG_UNDERCURRENT >::Type ERROR_FLAG_UNDERCURRENT; // 8

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::circuit_id >::Type circuit_id;
    typename ::uavcan::StorageType< typename FieldTypes::voltage >::Type voltage;
    typename ::uavcan::StorageType< typename FieldTypes::current >::Type current;
    typename ::uavcan::StorageType< typename FieldTypes::error_flags >::Type error_flags;

    CircuitStatus_()
        : circuit_id()
        , voltage()
        , current()
        , error_flags()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<56 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1091 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.power.CircuitStatus";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool CircuitStatus_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        circuit_id == rhs.circuit_id &&
        voltage == rhs.voltage &&
        current == rhs.current &&
        error_flags == rhs.error_flags;
}

template <int _tmpl>
bool CircuitStatus_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(circuit_id, rhs.circuit_id) &&
        ::uavcan::areClose(voltage, rhs.voltage) &&
        ::uavcan::areClose(current, rhs.current) &&
        ::uavcan::areClose(error_flags, rhs.error_flags);
}

template <int _tmpl>
int CircuitStatus_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::circuit_id::encode(self.circuit_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::voltage::encode(self.voltage, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::current::encode(self.current, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::error_flags::encode(self.error_flags, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int CircuitStatus_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::circuit_id::decode(self.circuit_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::voltage::decode(self.voltage, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::current::decode(self.current, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::error_flags::decode(self.error_flags, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature CircuitStatus_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x8313D33D0DDDA115ULL);

    FieldTypes::circuit_id::extendDataTypeSignature(signature);
    FieldTypes::voltage::extendDataTypeSignature(signature);
    FieldTypes::current::extendDataTypeSignature(signature);
    FieldTypes::error_flags::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename CircuitStatus_<_tmpl>::ConstantTypes::ERROR_FLAG_OVERVOLTAGE >::Type
    CircuitStatus_<_tmpl>::ERROR_FLAG_OVERVOLTAGE = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename CircuitStatus_<_tmpl>::ConstantTypes::ERROR_FLAG_UNDERVOLTAGE >::Type
    CircuitStatus_<_tmpl>::ERROR_FLAG_UNDERVOLTAGE = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename CircuitStatus_<_tmpl>::ConstantTypes::ERROR_FLAG_OVERCURRENT >::Type
    CircuitStatus_<_tmpl>::ERROR_FLAG_OVERCURRENT = 4U; // 4

template <int _tmpl>
const typename ::uavcan::StorageType< typename CircuitStatus_<_tmpl>::ConstantTypes::ERROR_FLAG_UNDERCURRENT >::Type
    CircuitStatus_<_tmpl>::ERROR_FLAG_UNDERCURRENT = 8U; // 8

/*
 * Final typedef
 */
typedef CircuitStatus_<0> CircuitStatus;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::power::CircuitStatus > _uavcan_gdtr_registrator_CircuitStatus;

}

} // Namespace power
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::power::CircuitStatus >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::power::CircuitStatus::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::power::CircuitStatus >::stream(Stream& s, ::uavcan::equipment::power::CircuitStatus::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "circuit_id: ";
    YamlStreamer< ::uavcan::equipment::power::CircuitStatus::FieldTypes::circuit_id >::stream(s, obj.circuit_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "voltage: ";
    YamlStreamer< ::uavcan::equipment::power::CircuitStatus::FieldTypes::voltage >::stream(s, obj.voltage, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "current: ";
    YamlStreamer< ::uavcan::equipment::power::CircuitStatus::FieldTypes::current >::stream(s, obj.current, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "error_flags: ";
    YamlStreamer< ::uavcan::equipment::power::CircuitStatus::FieldTypes::error_flags >::stream(s, obj.error_flags, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace power
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::power::CircuitStatus::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::power::CircuitStatus >::stream(s, obj, 0);
    return s;
}

} // Namespace power
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_HPP_INCLUDED