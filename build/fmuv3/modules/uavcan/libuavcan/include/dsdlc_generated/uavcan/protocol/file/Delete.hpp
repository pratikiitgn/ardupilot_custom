/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/pixhawk/github_ws/ardupilot_custom/modules/uavcan/dsdl/uavcan/protocol/file/47.Delete.uavcan
 */

#ifndef UAVCAN_PROTOCOL_FILE_DELETE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_FILE_DELETE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/protocol/file/Error.hpp>
#include <uavcan/protocol/file/Path.hpp>

/******************************* Source text **********************************
#
# Delete remote file system entry.
# If the remote entry is a directory, all nested entries will be removed too.
#

Path path

---

Error error
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.file.Delete
uavcan.protocol.file.Path path
---
uavcan.protocol.file.Error error
******************************************************************************/

#undef path
#undef error

namespace uavcan
{
namespace protocol
{
namespace file
{

struct UAVCAN_EXPORT Delete_
{
    template <int _tmpl>
    struct Request_
    {
        typedef const Request_<_tmpl>& ParameterType;
        typedef Request_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
            typedef ::uavcan::protocol::file::Path path;
        };

        enum
        {
            MinBitLen
                = FieldTypes::path::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::path::MaxBitLen
        };

        // Constants

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::path >::Type path;

        Request_()
            : path()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<1608 == MaxBitLen>::check();
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

    };

    template <int _tmpl>
    struct Response_
    {
        typedef const Response_<_tmpl>& ParameterType;
        typedef Response_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
            typedef ::uavcan::protocol::file::Error error;
        };

        enum
        {
            MinBitLen
                = FieldTypes::error::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::error::MaxBitLen
        };

        // Constants

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::error >::Type error;

        Response_()
            : error()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<16 == MaxBitLen>::check();
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

    };

    typedef Request_<0> Request;
    typedef Response_<0> Response;

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindService };
    enum { DefaultDataTypeID = 47 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.file.Delete";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

private:
    Delete_(); // Don't create objects of this type. Use Request/Response instead.
};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Delete_::Request_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        path == rhs.path;
}

template <int _tmpl>
bool Delete_::Request_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(path, rhs.path);
}

template <int _tmpl>
int Delete_::Request_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::path::encode(self.path, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Delete_::Request_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::path::decode(self.path, codec,  tao_mode);
    return res;
}

template <int _tmpl>
bool Delete_::Response_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        error == rhs.error;
}

template <int _tmpl>
bool Delete_::Response_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(error, rhs.error);
}

template <int _tmpl>
int Delete_::Response_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error::encode(self.error, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Delete_::Response_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error::decode(self.error, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
inline ::uavcan::DataTypeSignature Delete_::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x37184F4D5E898F0EULL);

    Request::FieldTypes::path::extendDataTypeSignature(signature);

    Response::FieldTypes::error::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Delete_ Delete;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::file::Delete > _uavcan_gdtr_registrator_Delete;

}

} // Namespace file
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::file::Delete::Request >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::file::Delete::Request::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::file::Delete::Request >::stream(Stream& s, ::uavcan::protocol::file::Delete::Request::ParameterType obj, const int level)
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
    s << "path: ";
    YamlStreamer< ::uavcan::protocol::file::Delete::Request::FieldTypes::path >::stream(s, obj.path, level + 1);
}

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::file::Delete::Response >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::file::Delete::Response::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::file::Delete::Response >::stream(Stream& s, ::uavcan::protocol::file::Delete::Response::ParameterType obj, const int level)
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
    s << "error: ";
    YamlStreamer< ::uavcan::protocol::file::Delete::Response::FieldTypes::error >::stream(s, obj.error, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace file
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::file::Delete::Request::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::file::Delete::Request >::stream(s, obj, 0);
    return s;
}

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::file::Delete::Response::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::file::Delete::Response >::stream(s, obj, 0);
    return s;
}

} // Namespace file
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_FILE_DELETE_HPP_INCLUDED