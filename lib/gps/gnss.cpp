#include <Arduino.h>
#include <TinyGPS++.h>

#include "gnss.h"
#include "config.h"

namespace gnss {

namespace {
    TinyGPSPlus gps;
    Location lastLocation;

    void cacheFix() {
        if (gps.location.isValid()) {
            lastLocation.latitude  = gps.location.lat();
            lastLocation.longitude = gps.location.lng();
            lastLocation.altitude  = gps.altitude.meters();
            lastLocation.speed     = gps.speed.mps();
            lastLocation.course    = gps.course.deg();
            lastLocation.valid     = true;
        } else {
            lastLocation.valid = false;
        }
    }
}

HardwareSerial& GNSS = Serial7;

int setup() {
    GNSS.begin(GNSS_BAUD_RATE);
    return 0;
}

void poll() {
    bool parsedSentence = false;
    while (GNSS.available() > 0) {
        if (gps.encode(GNSS.read())) {
            parsedSentence = true;
        }
    }

    if (parsedSentence) {
        cacheFix();
    }
}

Location latest() {
    return lastLocation;
}

Location read() {
    poll();
    return latest();
}

void end() {
    GNSS.end();
}

}

void serialEvent7() {
    gnss::poll();
}
