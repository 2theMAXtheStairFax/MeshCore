#pragma once

#include "LocationProvider.h"
#include <MicroNMEA.h>
#include <RTClib.h>
#include <helpers/RefCountedDigitalPin.h>

#ifndef GPS_EN
    #ifdef PIN_GPS_EN
        #define GPS_EN PIN_GPS_EN
    #else
        #define GPS_EN (-1)
    #endif
#endif

#ifndef GPS_EN_ACTIVE
    #ifdef PIN_GPS_EN_ACTIVE
        #define GPS_EN_ACTIVE PIN_GPS_EN_ACTIVE
    #else
        #define GPS_EN_ACTIVE HIGH
    #endif
#endif

#ifndef GPS_RESET
    #ifdef PIN_GPS_RESET
        #define GPS_RESET PIN_GPS_RESET
    #else
        #define GPS_RESET (-1)
    #endif
#endif

#ifndef GPS_RESET_ACTIVE
    #ifdef PIN_GPS_RESET_ACTIVE
        #define GPS_RESET_ACTIVE PIN_GPS_RESET_ACTIVE
    #else
        #define GPS_RESET_ACTIVE LOW
    #endif
#endif

class MicroNMEALocationProvider : public LocationProvider {
    char _nmeaBuffer[100];
    MicroNMEA nmea;
    mesh::RTCClock* _clock;
    Stream* _gps_serial;
    RefCountedDigitalPin* _peripher_power;
    int8_t _claims = 0;
    int _pin_reset;
    int _pin_en;
    unsigned long next_check = 0;
    long time_valid = 0;
    unsigned long _last_time_sync = 0;
    static const unsigned long TIME_SYNC_INTERVAL = 1800000; // Re-sync every 30 minutes

#ifdef GPS_UBLOX_M10
    // u-blox M10 constellation select is UBX-CFG-VALSET (binary protocol),
    // not a text sentence like MediaTek's PCAS04 - no NMEA equivalent, so
    // this bypasses nmea.sendSentence() and writes the frame directly.
    // Reapplied on every reset() since the module may not persist config
    // across a power cycle depending on backup supercap/battery state.
    void configureUbloxM10() {
        static const uint32_t keys[] = {
            0x1031001fUL, // CFG-SIGNAL_GPS_ENA
            0x10310001UL, // CFG-SIGNAL_GPS_L1CA_ENA
            0x10310021UL, // CFG-SIGNAL_GAL_ENA
            0x10310007UL, // CFG-SIGNAL_GAL_E1_ENA
            0x10310022UL, // CFG-SIGNAL_BDS_ENA
            0x1031000dUL, // CFG-SIGNAL_BDS_B1_ENA
            0x10310025UL, // CFG-SIGNAL_GLO_ENA
            0x10310018UL, // CFG-SIGNAL_GLO_L1_ENA
            0x10310024UL, // CFG-SIGNAL_QZSS_ENA
            0x10310012UL, // CFG-SIGNAL_QZSS_L1CA_ENA
        };
        const int n = sizeof(keys) / sizeof(keys[0]);

        uint8_t payload[4 + 5 * n];
        payload[0] = 0x00;   // message version
        payload[1] = 0x07;   // layers: RAM | BBR | Flash
        payload[2] = 0x00;   // reserved
        payload[3] = 0x00;   // reserved
        int p = 4;
        for (int i = 0; i < n; i++) {
            payload[p++] = (uint8_t)(keys[i] >> 0);
            payload[p++] = (uint8_t)(keys[i] >> 8);
            payload[p++] = (uint8_t)(keys[i] >> 16);
            payload[p++] = (uint8_t)(keys[i] >> 24);
            payload[p++] = 0x01;   // enable
        }

        static const uint8_t sync[2] = { 0xB5, 0x62 };
        uint8_t head[4] = { 0x06, 0x8A, (uint8_t)(p & 0xFF), (uint8_t)((p >> 8) & 0xFF) };

        uint8_t ck_a = 0, ck_b = 0;
        for (int i = 0; i < 4; i++) { ck_a = (uint8_t)(ck_a + head[i]); ck_b = (uint8_t)(ck_b + ck_a); }
        for (int i = 0; i < p; i++) { ck_a = (uint8_t)(ck_a + payload[i]); ck_b = (uint8_t)(ck_b + ck_a); }

        delay(250);
        _gps_serial->write(sync, 2);
        _gps_serial->write(head, 4);
        _gps_serial->write(payload, p);
        _gps_serial->write(ck_a);
        _gps_serial->write(ck_b);
        delay(250);
    }
#endif

public :
    MicroNMEALocationProvider(Stream& ser, mesh::RTCClock* clock = NULL, int pin_reset = GPS_RESET, int pin_en = GPS_EN,RefCountedDigitalPin* peripher_power=NULL) :
    nmea(_nmeaBuffer, sizeof(_nmeaBuffer)), _clock(clock), _gps_serial(&ser), _peripher_power(peripher_power), _pin_reset(pin_reset), _pin_en(pin_en) {
        if (_pin_reset != -1) {
            pinMode(_pin_reset, OUTPUT);
            digitalWrite(_pin_reset, GPS_RESET_ACTIVE);
        }
        if (_pin_en != -1) {
            pinMode(_pin_en, OUTPUT);
            digitalWrite(_pin_en, !GPS_EN_ACTIVE);
        }
    }

    void claim() {
        _claims++;
        if (_peripher_power) _peripher_power->claim();
    }

    void release() {
        if (_claims == 0) return; // avoid negative _claims
        _claims--;
        if (_peripher_power) _peripher_power->release();
    }

    void begin() override {
        claim();
        if (_pin_en != -1) {
            digitalWrite(_pin_en, GPS_EN_ACTIVE);
        }
        if (_pin_reset != -1) {
            digitalWrite(_pin_reset, !GPS_RESET_ACTIVE);
        }
    }

    void reset() override {
        if (_pin_reset != -1) {
            digitalWrite(_pin_reset, GPS_RESET_ACTIVE);
            delay(10);
            digitalWrite(_pin_reset, !GPS_RESET_ACTIVE);
        }
#ifdef GPS_UBLOX_M10
        configureUbloxM10();
#endif
    }

    void stop() override {
        if (_pin_en != -1) {
            digitalWrite(_pin_en, !GPS_EN_ACTIVE);
        }
        if (_pin_reset != -1) {
            digitalWrite(_pin_reset, GPS_RESET_ACTIVE);
        }
        release();
    }

    bool isEnabled() override {
        // directly read the enable pin if present as gps can be
        // activated/deactivated outside of here ...
        if (_pin_en != -1) {
            return digitalRead(_pin_en) == GPS_EN_ACTIVE;
        } else {
            return true; // no enable so must be active
        }
    }

    void syncTime() override { nmea.clear(); LocationProvider::syncTime(); }
    long getLatitude() override { return nmea.getLatitude(); }
    long getLongitude() override { return nmea.getLongitude(); }
    long getAltitude() override { 
        long alt = 0;
        nmea.getAltitude(alt);
        return alt;
    }
    long satellitesCount() override { return nmea.getNumSatellites(); }
    bool isValid() override { return nmea.isValid(); }

    long getTimestamp() override { 
        DateTime dt(nmea.getYear(), nmea.getMonth(),nmea.getDay(),nmea.getHour(),nmea.getMinute(),nmea.getSecond());
        return dt.unixtime();
    } 

    void sendSentence(const char *sentence) override {
        nmea.sendSentence(*_gps_serial, sentence);
    }

    void loop() override {

        while (_gps_serial->available()) {
            char c = _gps_serial->read();
            #ifdef GPS_NMEA_DEBUG
            Serial.print(c);
            #endif
            nmea.process(c);
        }

        if (!isValid()) time_valid = 0;

        if ((long)(millis() - next_check) > 0) {
            next_check = millis() + 1000;
            // Re-enable time sync periodically when GPS has valid fix
            if (!_time_sync_needed && _clock != NULL && (millis() - _last_time_sync) > TIME_SYNC_INTERVAL) {
                _time_sync_needed = true;
            }
            if (_time_sync_needed && time_valid > 2) {
                if (_clock != NULL) {
                    _clock->setCurrentTime(getTimestamp());
                    _time_sync_needed = false;
                    _last_time_sync = millis();
                }
            }
            if (isValid()) {
                time_valid ++;
            }
        }
    }
};
