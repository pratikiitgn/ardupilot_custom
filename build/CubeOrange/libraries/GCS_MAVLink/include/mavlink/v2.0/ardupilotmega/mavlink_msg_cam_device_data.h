#pragma once
// MESSAGE CAM_DEVICE_DATA PACKING

#define MAVLINK_MSG_ID_CAM_DEVICE_DATA 11040


typedef struct __mavlink_cam_device_data_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float roll; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
} mavlink_cam_device_data_t;

#define MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN 28
#define MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN 28
#define MAVLINK_MSG_ID_11040_LEN 28
#define MAVLINK_MSG_ID_11040_MIN_LEN 28

#define MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC 131
#define MAVLINK_MSG_ID_11040_CRC 131



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAM_DEVICE_DATA { \
    11040, \
    "CAM_DEVICE_DATA", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cam_device_data_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_cam_device_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_cam_device_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_cam_device_data_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_cam_device_data_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_cam_device_data_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_cam_device_data_t, yawspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAM_DEVICE_DATA { \
    "CAM_DEVICE_DATA", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cam_device_data_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_cam_device_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_cam_device_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_cam_device_data_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_cam_device_data_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_cam_device_data_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_cam_device_data_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a cam_device_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cam_device_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN);
#else
    mavlink_cam_device_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAM_DEVICE_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
}

/**
 * @brief Pack a cam_device_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cam_device_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN);
#else
    mavlink_cam_device_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAM_DEVICE_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
}

/**
 * @brief Encode a cam_device_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cam_device_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cam_device_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cam_device_data_t* cam_device_data)
{
    return mavlink_msg_cam_device_data_pack(system_id, component_id, msg, cam_device_data->time_boot_ms, cam_device_data->roll, cam_device_data->pitch, cam_device_data->yaw, cam_device_data->rollspeed, cam_device_data->pitchspeed, cam_device_data->yawspeed);
}

/**
 * @brief Encode a cam_device_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_device_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cam_device_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cam_device_data_t* cam_device_data)
{
    return mavlink_msg_cam_device_data_pack_chan(system_id, component_id, chan, msg, cam_device_data->time_boot_ms, cam_device_data->roll, cam_device_data->pitch, cam_device_data->yaw, cam_device_data->rollspeed, cam_device_data->pitchspeed, cam_device_data->yawspeed);
}

/**
 * @brief Send a cam_device_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cam_device_data_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAM_DEVICE_DATA, buf, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
#else
    mavlink_cam_device_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAM_DEVICE_DATA, (const char *)&packet, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
#endif
}

/**
 * @brief Send a cam_device_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cam_device_data_send_struct(mavlink_channel_t chan, const mavlink_cam_device_data_t* cam_device_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cam_device_data_send(chan, cam_device_data->time_boot_ms, cam_device_data->roll, cam_device_data->pitch, cam_device_data->yaw, cam_device_data->rollspeed, cam_device_data->pitchspeed, cam_device_data->yawspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAM_DEVICE_DATA, (const char *)cam_device_data, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cam_device_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAM_DEVICE_DATA, buf, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
#else
    mavlink_cam_device_data_t *packet = (mavlink_cam_device_data_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAM_DEVICE_DATA, (const char *)packet, MAVLINK_MSG_ID_CAM_DEVICE_DATA_MIN_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN, MAVLINK_MSG_ID_CAM_DEVICE_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE CAM_DEVICE_DATA UNPACKING


/**
 * @brief Get field time_boot_ms from cam_device_data message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_cam_device_data_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from cam_device_data message
 *
 * @return [rad] Roll angle (-pi..+pi)
 */
static inline float mavlink_msg_cam_device_data_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from cam_device_data message
 *
 * @return [rad] Pitch angle (-pi..+pi)
 */
static inline float mavlink_msg_cam_device_data_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from cam_device_data message
 *
 * @return [rad] Yaw angle (-pi..+pi)
 */
static inline float mavlink_msg_cam_device_data_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rollspeed from cam_device_data message
 *
 * @return [rad/s] Roll angular speed
 */
static inline float mavlink_msg_cam_device_data_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitchspeed from cam_device_data message
 *
 * @return [rad/s] Pitch angular speed
 */
static inline float mavlink_msg_cam_device_data_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yawspeed from cam_device_data message
 *
 * @return [rad/s] Yaw angular speed
 */
static inline float mavlink_msg_cam_device_data_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a cam_device_data message into a struct
 *
 * @param msg The message to decode
 * @param cam_device_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_cam_device_data_decode(const mavlink_message_t* msg, mavlink_cam_device_data_t* cam_device_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cam_device_data->time_boot_ms = mavlink_msg_cam_device_data_get_time_boot_ms(msg);
    cam_device_data->roll = mavlink_msg_cam_device_data_get_roll(msg);
    cam_device_data->pitch = mavlink_msg_cam_device_data_get_pitch(msg);
    cam_device_data->yaw = mavlink_msg_cam_device_data_get_yaw(msg);
    cam_device_data->rollspeed = mavlink_msg_cam_device_data_get_rollspeed(msg);
    cam_device_data->pitchspeed = mavlink_msg_cam_device_data_get_pitchspeed(msg);
    cam_device_data->yawspeed = mavlink_msg_cam_device_data_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN? msg->len : MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN;
        memset(cam_device_data, 0, MAVLINK_MSG_ID_CAM_DEVICE_DATA_LEN);
    memcpy(cam_device_data, _MAV_PAYLOAD(msg), len);
#endif
}
