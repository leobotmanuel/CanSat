#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>

LSM6 ag;
LIS3MDL mag;
LPS pta;

float pressure;
float temperature;
float altitude;

void ag_init() {
  if (!ag.init()){
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  ag.enableDefault();
}


void mag_init() {
  if (!mag.init()){
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();
}


void pta_init() {
  if (!pta.init()){
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  pta.enableDefault();
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  ag_init();
  mag_init();
  pta_init();  
}

void ag_read() {
    ag.read();
}

void mag_read() {
    mag.read();
}


void pta_read() {
  pressure = pta.readPressureMillibars();
  temperature = pta.readTemperatureC();
  altitude = pta.pressureToAltitudeMeters(pressure);
}

void altimu10(){
  const char floatsize = 7;
  const char decimalsize = 3;
  char report[255];
  char pressure_str[floatsize];
  char temperature_str[floatsize];
  char altitude_str[floatsize];
  dtostrf(pressure, floatsize, decimalsize, pressure_str);
  dtostrf(temperature, floatsize, decimalsize, temperature_str);
  dtostrf(altitude, floatsize, decimalsize, altitude_str);
  snprintf(report, sizeof(report), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%s", 
    ag.a.x, ag.a.y, ag.a.z,
    ag.g.x, ag.g.y, ag.g.z,
    mag.m.x, mag.m.y, mag.m.z,
    pressure_str, temperature_str, altitude_str);
  Serial.println(report);
}

void loop() {
  ag_read();
  mag_read();
  pta_read();
  altimu10();
}
