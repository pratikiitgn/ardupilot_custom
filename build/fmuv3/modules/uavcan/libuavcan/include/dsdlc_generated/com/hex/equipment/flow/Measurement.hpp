/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/pratik/pratik_github/ardupilot_custom/libraries/AP_UAVCAN/dsdl/com/hex/equipment/flow/20200.Measurement.uavcan
 */

#ifndef COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_HPP_INCLUDED
#define COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
float32 integration_interval    # Integration Interval in seconds
float32[2] rate_gyro_integral   # Integrated Gyro Data in radians
float32[2] flow_integral        # Integrated LOS Data in radians
uint8 quality                   # Flow Data Quality Lowest(0)-Highest(255) Unitless
******************************************************************************/

/********************* DSDL signature source definition ***********************
com.hex.equipment.flow.Measurement
saturated float32 integration_interval
saturated float32[2] rate_gyro_integral
saturated float32[2] flow_integral
saturated uint8 quality
******************************************************************************/

#undef integration_interval
#undef rate_gyro_integral
#undef flow_integral
#undef quality

namespace com
{
namespace hex
{
namespace equipment
{
namespace flow
{

template <int _tmpl>
struct UAVCAN_EXPORT Measurement_
{
    typedef const Measurement_<_tmpl>& ParameterType;
    typedef Measurement_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > integration_interval;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 2 > rate_gyro_integral;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 2 > flow_integral;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > quality;
    };

    enum
    {
        MinBitLen
            = FieldTypes::integration_interval::MinBitLen
            + FieldTypes::rate_gyro_integral::MinBitLen
            + FieldTypes::flow_integral::MinBitLen
            + FieldTypes::quality::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::integration_interval::MaxBitLen
            + FieldTypes::rate_gyro_integral::MaxBitLen
            + FieldTypes::flow_integral::MaxBitLen
            + FieldTypes::quality::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::integration_interval >::Type integration_interval;
    typename ::uavcan::StorageType< typename FieldTypes::rate_gyro_integral >::Type rate_gyro_integral;
    typename ::uavcan::StorageType< typename FieldTypes::flow_integral >::Type flow_integral;
    typename ::uavcan::StorageType< typename FieldTypes::quality >::Type quality;

    Measurement_()
        : integration_interval()
        , rate_gyro_integral()
        , flow_integral()
        , quality()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<168 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 20200 };

    static const char* getDataTypeFullName()
    {
        return "com.hex.equipment.flow.Measurement";
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
bool Measurement_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        integration_interval == rhs.integration_interval &&
        rate_gyro_integral == rhs.rate_gyro_integral &&
        flow_integral == rhs.flow_integral &&
        quality == rhs.quality;
}

template <int _tmpl>
bool Measurement_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(integration_interval, rhs.integration_interval) &&
        ::uavcan::areClose(rate_gyro_integral, rhs.rate_gyro_integral) &&
        ::uavcan::areClose(flow_integral, rhs.flow_integral) &&
        ::uavcan::areClose(quality, rhs.quality);
}

template <int _tmpl>
int Measurement_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::integration_interval::encode(self.integration_interval, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rate_gyro_integral::encode(self.rate_gyro_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::flow_integral::encode(self.flow_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::quality::encode(self.quality, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Measurement_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::integration_interval::decode(self.integration_interval, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rate_gyro_integral::decode(self.rate_gyro_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::flow_integral::decode(self.flow_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::quality::decode(self.quality, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Measurement_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x6A908866BCB49C18ULL);

    FieldTypes::integration_interval::extendDataTypeSignature(signature);
    FieldTypes::rate_gyro_integral::extendDataTypeSignature(signature);
    FieldTypes::flow_integral::extendDataTypeSignature(signature);
    FieldTypes::quality::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Measurement_<0> Measurement;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::com::hex::equipment::flow::Measurement > _uavcan_gdtr_registrator_Measurement;

}

} // Namespace flow
} // Namespace equipment
} // Namespace hex
} // Namespace com

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::com::hex::equipment::flow::Measurement >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::com::hex::equipment::flow::Measurement::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::com::hex::equipment::flow::Measurement >::stream(Stream& s, ::com::hex::equipment::flow::Measurement::ParameterType obj, const int level)
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
    s << "integration_interval: ";
    YamlStreamer< ::com::hex::equipment::flow::Measurement::FieldTypes::integration_interval >::stream(s, obj.integration_interval, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "rate_gyro_integral: ";
    YamlStreamer< ::com::hex::equipment::flow::Measurement::FieldTypes::rate_gyro_integral >::stream(s, obj.rate_gyro_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "flow_integral: ";
    YamlStreamer< ::com::hex::equipment::flow::Measurement::FieldTypes::flow_integral >::stream(s, obj.flow_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "quality: ";
    YamlStreamer< ::com::hex::equipment::flow::Measurement::FieldTypes::quality >::stream(s, obj.quality, level + 1);
}

}

namespace com
{
namespace hex
{
namespace equipment
{
namespace flow
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::com::hex::equipment::flow::Measurement::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::com::hex::equipment::flow::Measurement >::stream(s, obj, 0);
    return s;
}

} // Namespace flow
} // Namespace equipment
} // Namespace hex
} // Namespace com

#endif // COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_HPP_INCLUDED