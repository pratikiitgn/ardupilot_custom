/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/pratik/ardupilot/modules/uavcan/dsdl/uavcan/equipment/indication/1081.LightsCommand.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/equipment/indication/SingleLightCommand.hpp>

/******************************* Source text **********************************
#
# Lights control command.
#

SingleLightCommand[<=20] commands
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.indication.LightsCommand
uavcan.equipment.indication.SingleLightCommand[<=20] commands
******************************************************************************/

#undef commands

namespace uavcan
{
namespace equipment
{
namespace indication
{

template <int _tmpl>
struct UAVCAN_EXPORT LightsCommand_
{
    typedef const LightsCommand_<_tmpl>& ParameterType;
    typedef LightsCommand_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::equipment::indication::SingleLightCommand, ::uavcan::ArrayModeDynamic, 20 > commands;
    };

    enum
    {
        MinBitLen
            = FieldTypes::commands::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::commands::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::commands >::Type commands;

    LightsCommand_()
        : commands()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<485 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1081 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.indication.LightsCommand";
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
bool LightsCommand_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        commands == rhs.commands;
}

template <int _tmpl>
bool LightsCommand_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(commands, rhs.commands);
}

template <int _tmpl>
int LightsCommand_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::commands::encode(self.commands, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int LightsCommand_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::commands::decode(self.commands, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature LightsCommand_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xB918CEDB4B81242DULL);

    FieldTypes::commands::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef LightsCommand_<0> LightsCommand;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::indication::LightsCommand > _uavcan_gdtr_registrator_LightsCommand;

}

} // Namespace indication
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::indication::LightsCommand >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::indication::LightsCommand::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::indication::LightsCommand >::stream(Stream& s, ::uavcan::equipment::indication::LightsCommand::ParameterType obj, const int level)
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
    s << "commands: ";
    YamlStreamer< ::uavcan::equipment::indication::LightsCommand::FieldTypes::commands >::stream(s, obj.commands, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace indication
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::indication::LightsCommand::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::indication::LightsCommand >::stream(s, obj, 0);
    return s;
}

} // Namespace indication
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_HPP_INCLUDED