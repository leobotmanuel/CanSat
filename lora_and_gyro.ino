#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include <SPI.h>
#include <LoRa.h>


LSM6 ag;
LIS3MDL mag;
LPS pta;

float pressure;
float temperature;
float altitude;
char report[255];
const int DELAY = 1000;

void ag_init() {
  if (!ag.init()) {
    Serial.println("Failed to detect and initialize gyroscope!");
    while (1);
  }
  ag.enableDefault();
}


void mag_init() {
  if (!mag.init()) {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();
}


void pta_init() {
  if (!pta.init()) {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  pta.enableDefault();
}


void lora_init() {
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }  
}

void setup() {
  Serial.begin(9600);
  Serial.println("iniciando");

  Wire.begin();
  ag_init();
  mag_init();
  pta_init();
  lora_init();
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


void lora_send() {
  LoRa.beginPacket();
  LoRa.print(report);
  LoRa.endPacket();
  Serial.println(report);
}

char* altimu10() {
  const char floatsize = 7;
  const char decimalsize = 3;
  char pressure_str[floatsize + 1];
  char temperature_str[floatsize + 1];
  char altitude_str[floatsize + 1];
  dtostrf(pressure, floatsize, decimalsize, pressure_str);
  dtostrf(temperature, floatsize, decimalsize, temperature_str);
  dtostrf(altitude, floatsize, decimalsize, altitude_str);
  snprintf(report, sizeof(report), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%s",
           ag.a.x, ag.a.y, ag.a.z,
           ag.g.x, ag.g.y, ag.g.z,
           mag.m.x, mag.m.y, mag.m.z,
           pressure_str, temperature_str, altitude_str);
  return report;
}


void loop() {
  ag_read();
  mag_read();
  pta_read();
  altimu10();
  lora_send();
  delay(DELAY);
}
