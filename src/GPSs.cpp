#include "GPSs.h"
#include <Adafruit_GPS.h>

GPSs::GPSs(int id, HardwareSerial* serialIn) {
    this->id = id;
    this->wGPS = Adafruit_GPS(serialIn);
}

void GPSs::begin() {
    this->wGPS.begin(9600);
    this->wGPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    this->wGPS.sendCommand(PMTK_ENABLE_WAAS);
}

Health GPSs::healthCheck() const {
    return Health::HEALTHY;
}

bool GPSs::ready() {
    char c = this->wGPS.read(); // TODO: See if we can throw this away
    return this->wGPS.newNMEAreceived();
}

SensorData GPSs::read() {
    SensorData sensorData = SensorData(id, 1);
    uint8_t buf[8];
    BufferPacker<8> packer;
    if(this->wGPS.parse(this->wGPS.lastNMEA())) {
        if(this->wGPS.fix) {
            Serial.println("LOCKED");
            packer.pack(float(this->wGPS.latitudeDegrees));
            packer.pack(float(this->wGPS.longitudeDegrees));
            packer.deepCopyTo(buf);

            Serial.print("LAT: ");
            Serial.print(this->wGPS.latitudeDegrees, 5);
            Serial.print(" \t LONG: ");
            Serial.println(this->wGPS.longitudeDegrees, 5);

            for(int i; i < 8; i++) {
                Serial.print(buf[i]);
                Serial.print(", ");
            }
            Serial.println("");
        } else {
            packer.pack(0);
            packer.deepCopyTo(buf);
            Serial.println("NO LOCK");
        }
    } else {
        packer.pack(0);
        packer.deepCopyTo(buf);
    }
    return sensorData;
}