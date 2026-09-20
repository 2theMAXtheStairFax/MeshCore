#pragma once

#include "LocationProvider.h"
#include <RTClib.h>

// Minimal read-only MAVLink v1/v2 frame parser used as a LocationProvider.
// Feeds off a flight controller's telemetry UART instead of a dedicated GPS
// module -- position comes from the FC's own (usually EKF-fused) estimate.
//
// Only three messages are decoded: GLOBAL_POSITION_INT (#33) for lat/lon/alt,
// GPS_RAW_INT (#24) for fix type + satellite count, and SYSTEM_TIME (#2) for
// wall-clock time. Everything else is parsed just enough to skip over it and
// stay frame-synced. This avoids pulling in mavlink/c_library_v2, which would
// add hundreds of unused message definitions for three fields.
//
// This class never transmits. On ArduPilot the chosen serial port's stream
// rates (SRx_POSITION for GLOBAL_POSITION_INT, SRx_RAW_SENS for GPS_RAW_INT)
// must be configured to a non-zero rate, or these messages simply never
// arrive -- there is no request/response handshake here to enable them.

#define MAVLOC_MSG_ID_SYSTEM_TIME         2
#define MAVLOC_MSG_ID_GPS_RAW_INT         24
#define MAVLOC_MSG_ID_GLOBAL_POSITION_INT 33

#define MAVLOC_STX_V1  0xFE
#define MAVLOC_STX_V2  0xFD

class MAVLinkLocationProvider : public LocationProvider {
    mesh::RTCClock* _clock;
    Stream* _serial;

    enum ParseState { WAIT_STX, HEADER, PAYLOAD, CRC_LO, CRC_HI, SIGNATURE };
    ParseState _state = WAIT_STX;

    bool _is_v2 = false;
    uint8_t _header[9];       // v2: LEN,INCOMPAT,COMPAT,SEQ,SYSID,COMPID,MSGID0,MSGID1,MSGID2. v1 only uses first 5.
    uint8_t _header_len = 0, _header_idx = 0;
    uint8_t _incompat_flags = 0;
    uint16_t _msgid = 0;

    uint8_t _payload[64];     // GPS_RAW_INT is 44 bytes with ArduPilot's extension fields (30 mandatory + 14 ext)
    uint8_t _payload_len = 0, _payload_idx = 0;

    uint16_t _crc = 0xFFFF;
    uint16_t _crc_recv = 0;
    uint8_t _sig_remaining = 0;

    // last known fix, updated as messages arrive
    int32_t _lat_degE7 = 0, _lon_degE7 = 0;
    int32_t _alt_mm = 0;
    uint8_t _fix_type = 0;        // MAV_GPS_FIX_TYPE: 3 = 3D_FIX, higher = RTK etc.
    uint8_t _satellites = 255;    // 255 = unknown
    uint64_t _unix_time_usec = 0;
    unsigned long _last_pos_fix_millis = 0;
    static const unsigned long STALE_MS = 5000;

    static uint16_t crcAccumulate(uint8_t data, uint16_t crc) {
        uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
        tmp ^= (uint8_t)(tmp << 4);
        return (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
    }

    // MAVLink CRC_EXTRA per message (from the common dialect); 0 = not one we decode
    static uint8_t crcExtraFor(uint16_t msgid) {
        switch (msgid) {
            case MAVLOC_MSG_ID_SYSTEM_TIME:         return 137;
            case MAVLOC_MSG_ID_GPS_RAW_INT:         return 24;
            case MAVLOC_MSG_ID_GLOBAL_POSITION_INT: return 104;
            default: return 0;
        }
    }

    static uint32_t readU32(const uint8_t* p, uint8_t off) {
        return (uint32_t)p[off] | ((uint32_t)p[off+1] << 8) | ((uint32_t)p[off+2] << 16) | ((uint32_t)p[off+3] << 24);
    }
    static uint64_t readU64(const uint8_t* p, uint8_t off) {
        return (uint64_t)readU32(p, off) | ((uint64_t)readU32(p, off + 4) << 32);
    }

    void handleMessage() {
        switch (_msgid) {
            case MAVLOC_MSG_ID_GLOBAL_POSITION_INT:
                if (_payload_len < 16) break;   // time_boot_ms,lat,lon,alt
                _lat_degE7 = (int32_t)readU32(_payload, 4);
                _lon_degE7 = (int32_t)readU32(_payload, 8);
                _alt_mm    = (int32_t)readU32(_payload, 12);   // MSL altitude, mm
                _last_pos_fix_millis = millis();
                break;

            case MAVLOC_MSG_ID_GPS_RAW_INT:
                if (_payload_len < 30) break;
                _fix_type   = _payload[28];
                _satellites = _payload[29];
                break;

            case MAVLOC_MSG_ID_SYSTEM_TIME:
                if (_payload_len < 8) break;
                _unix_time_usec = readU64(_payload, 0);
                if (_clock != NULL && _unix_time_usec > 0) {
                    _clock->setCurrentTime((uint32_t)(_unix_time_usec / 1000000ULL));
                }
                break;
        }
    }

    void processByte(uint8_t c) {
        switch (_state) {
            case WAIT_STX:
                if (c == MAVLOC_STX_V1)      { _is_v2 = false; _header_len = 5; _header_idx = 0; _state = HEADER; }
                else if (c == MAVLOC_STX_V2) { _is_v2 = true;  _header_len = 9; _header_idx = 0; _state = HEADER; }
                break;

            case HEADER:
                _header[_header_idx++] = c;
                if (_header_idx == _header_len) {
                    _payload_len     = _header[0];   // LEN is always the first byte after STX
                    _incompat_flags  = _is_v2 ? _header[1] : 0;
                    _msgid           = _is_v2 ? ((uint16_t)_header[6] | ((uint16_t)_header[7] << 8)) : _header[4];

                    _crc = 0xFFFF;
                    for (uint8_t i = 0; i < _header_len; i++) _crc = crcAccumulate(_header[i], _crc);

                    _payload_idx = 0;
                    _state = (_payload_len == 0) ? CRC_LO : PAYLOAD;
                }
                break;

            case PAYLOAD:
                if (_payload_idx < sizeof(_payload)) _payload[_payload_idx] = c;
                _crc = crcAccumulate(c, _crc);
                if (++_payload_idx >= _payload_len) _state = CRC_LO;
                break;

            case CRC_LO:
                _crc_recv = c;
                _state = CRC_HI;
                break;

            case CRC_HI: {
                _crc_recv |= ((uint16_t)c << 8);
                uint8_t extra = crcExtraFor(_msgid);
                if (extra != 0 && _payload_len <= sizeof(_payload) && crcAccumulate(extra, _crc) == _crc_recv) {
                    handleMessage();
                }
                if (_is_v2 && (_incompat_flags & 0x01)) {   // signed packet: skip trailing 13-byte signature
                    _sig_remaining = 13;
                    _state = SIGNATURE;
                } else {
                    _state = WAIT_STX;
                }
                break;
            }

            case SIGNATURE:
                if (--_sig_remaining == 0) _state = WAIT_STX;
                break;
        }
    }

public:
    MAVLinkLocationProvider(Stream& ser, mesh::RTCClock* clock = NULL) : _clock(clock), _serial(&ser) {}

    void begin() override { }
    void reset() override { _state = WAIT_STX; _fix_type = 0; _last_pos_fix_millis = 0; }
    void stop() override { }
    bool isEnabled() override { return true; }

    void loop() override {
        while (_serial->available()) {
            processByte((uint8_t)_serial->read());
        }
    }

    void sendSentence(const char* sentence) override { }   // not applicable -- FC is the position source, not us

    // MicroNMEA reports lat/lon in degrees x1e6; MAVLink gives degrees x1e7.
    long getLatitude() override  { return _lat_degE7 / 10; }
    long getLongitude() override { return _lon_degE7 / 10; }
    long getAltitude() override  { return _alt_mm; }   // already millimetres, matches MicroNMEALocationProvider's units
    long satellitesCount() override { return (_satellites == 255) ? 0 : _satellites; }

    bool isValid() override {
        return _fix_type >= 3 && (millis() - _last_pos_fix_millis) < STALE_MS;
    }

    long getTimestamp() override { return (long)(_unix_time_usec / 1000000ULL); }
};
