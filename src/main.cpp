#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <DataCollector.h>
#include <RVC.h>
#include <GPSs.h>
constexpr bool ACCEL_CRIT = false;
constexpr uint32_t ACCEL_INTERVAL = 50;

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
//RVC accelerometer = RVC(ReservedIDs::RVCId, ACCEL_CRIT, ACCEL_INTERVAL, &rvc);

GPSs myGPS = GPSs(10, &Serial3);



constexpr size_t NUM_SENSORS = 1;

Sensor* SENSORS[] = {
    &myGPS
};

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> comsCAN;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCAN;

DataCollector dcr = DataCollector(ReservedIDs::DCRId, NUM_SENSORS, SENSORS, false);

constexpr uint32_t CAN_BAUD_RATE = 250000;
constexpr uint32_t SERIAL_BAUD_RATE = 9600;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    comsCAN.begin();
    comsCAN.setBaudRate(CAN_BAUD_RATE);
    motorCAN.begin();
    motorCAN.setBaudRate(CAN_BAUD_RATE);

    //Serial2.begin(115200);
    //accelerometer.begin(&Serial2);
    myGPS.begin();
    
    dcr.begin(&motorCAN, &comsCAN);
}

void loop() {
    dcr.checkSensors();
}