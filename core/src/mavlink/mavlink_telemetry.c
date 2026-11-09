#include "mavlink_internal.h"

#include "core/common/time.h"

void core_mavlink_internal_decode_into_telemetry(core_mavlink_conn_t *conn, const mavlink_message_t *msg) {
    uint64_t now = core_now_ms();

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        /* A GCS or companion on the same link must not be mistaken for the
         * vehicle: autonomy reads armed state and flight mode from this. */
        if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
            uint8_t operator_sysid = conn->cfg.operator_gcs_sysid ? conn->cfg.operator_gcs_sysid : 255;
            if (hb.type == MAV_TYPE_GCS && msg->sysid == operator_sysid) {
                conn->telemetry.gcs_heartbeat.sysid = msg->sysid;
                conn->telemetry.gcs_heartbeat.count++;
                conn->telemetry.gcs_heartbeat.last_update_ms = now;
                conn->telemetry.gcs_heartbeat.valid = true;
            }
            break;
        }
        conn->vehicle_sysid = msg->sysid;
        conn->vehicle_compid = msg->compid;
        conn->telemetry.heartbeat.type = hb.type;
        conn->telemetry.heartbeat.autopilot = hb.autopilot;
        conn->telemetry.heartbeat.base_mode = hb.base_mode;
        conn->telemetry.heartbeat.custom_mode = hb.custom_mode;
        conn->telemetry.heartbeat.system_status = hb.system_status;
        conn->telemetry.heartbeat.last_update_ms = now;
        conn->telemetry.heartbeat.valid = true;
        conn->last_heartbeat_rx_ms = now;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(msg, &pos);
        conn->telemetry.global_position.lat = pos.lat;
        conn->telemetry.global_position.lon = pos.lon;
        conn->telemetry.global_position.alt_mm = pos.alt;
        conn->telemetry.global_position.relative_alt_mm = pos.relative_alt;
        conn->telemetry.global_position.vx = pos.vx;
        conn->telemetry.global_position.vy = pos.vy;
        conn->telemetry.global_position.vz = pos.vz;
        conn->telemetry.global_position.hdg_cdeg = pos.hdg;
        conn->telemetry.global_position.last_update_ms = now;
        conn->telemetry.global_position.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(msg, &gps);
        conn->telemetry.gps_raw.fix_type = gps.fix_type;
        conn->telemetry.gps_raw.lat = gps.lat;
        conn->telemetry.gps_raw.lon = gps.lon;
        conn->telemetry.gps_raw.alt_mm = gps.alt;
        conn->telemetry.gps_raw.eph = gps.eph;
        conn->telemetry.gps_raw.epv = gps.epv;
        conn->telemetry.gps_raw.vel_cms = gps.vel;
        conn->telemetry.gps_raw.cog_cdeg = gps.cog;
        conn->telemetry.gps_raw.satellites_visible = gps.satellites_visible;
        conn->telemetry.gps_raw.time_usec = gps.time_usec;
        conn->telemetry.gps_raw.last_update_ms = now;
        conn->telemetry.gps_raw.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(msg, &att);
        conn->telemetry.attitude.roll = att.roll;
        conn->telemetry.attitude.pitch = att.pitch;
        conn->telemetry.attitude.yaw = att.yaw;
        conn->telemetry.attitude.rollspeed = att.rollspeed;
        conn->telemetry.attitude.pitchspeed = att.pitchspeed;
        conn->telemetry.attitude.yawspeed = att.yawspeed;
        conn->telemetry.attitude.last_update_ms = now;
        conn->telemetry.attitude.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU2: {
        mavlink_scaled_imu2_t imu;
        mavlink_msg_scaled_imu2_decode(msg, &imu);
        conn->telemetry.imu.xacc_mps2 = (float)imu.xacc * 9.80665f / 1000.0f;
        conn->telemetry.imu.yacc_mps2 = (float)imu.yacc * 9.80665f / 1000.0f;
        conn->telemetry.imu.zacc_mps2 = (float)imu.zacc * 9.80665f / 1000.0f;
        conn->telemetry.imu.xgyro_rps = (float)imu.xgyro / 1000.0f;
        conn->telemetry.imu.ygyro_rps = (float)imu.ygyro / 1000.0f;
        conn->telemetry.imu.zgyro_rps = (float)imu.zgyro / 1000.0f;
        conn->telemetry.imu.last_update_ms = now;
        conn->telemetry.imu.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t baro;
        mavlink_msg_scaled_pressure_decode(msg, &baro);
        conn->telemetry.baro.press_abs_hpa = baro.press_abs;
        conn->telemetry.baro.last_update_ms = now;
        conn->telemetry.baro.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS: {
        mavlink_radio_status_t radio;
        mavlink_msg_radio_status_decode(msg, &radio);
        conn->telemetry.radio_status.rssi = radio.rssi;
        conn->telemetry.radio_status.remrssi = radio.remrssi;
        conn->telemetry.radio_status.txbuf = radio.txbuf;
        conn->telemetry.radio_status.noise = radio.noise;
        conn->telemetry.radio_status.remnoise = radio.remnoise;
        conn->telemetry.radio_status.rxerrors = radio.rxerrors;
        conn->telemetry.radio_status.fixed = radio.fixed;
        conn->telemetry.radio_status.last_update_ms = now;
        conn->telemetry.radio_status.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        /* Only the autopilot's own view of its radio counts. */
        if (msg->sysid != conn->vehicle_sysid || msg->compid != conn->vehicle_compid) {
            break;
        }
        conn->telemetry.rc_status.rssi = mavlink_msg_rc_channels_get_rssi(msg);
        conn->telemetry.rc_status.last_update_ms = now;
        conn->telemetry.rc_status.valid = true;
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        conn->telemetry.command_ack.command = ack.command;
        conn->telemetry.command_ack.result = ack.result;
        conn->telemetry.command_ack.last_update_ms = now;
        conn->telemetry.command_ack.valid = true;
        break;
    }
    default:
        /* Not one of the messages we track yet - ignored, extend here later. */
        break;
    }
}
