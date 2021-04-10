#include <Adafruit_GPS.h>
#include <SPI.h>
#include <LoRa.h>

#define GPSSerial Serial2

Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

uint32_t timer = millis();

void setup()
{
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop()
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.print(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (millis() - timer > 2000) {
    timer = millis();
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }

  float nuestraLatitud = GPS.lat;
  float nuestraLongitud = GPS.lon;
  String latitud;
  String longitud;
  int counter = 0;
  String datos = "";
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    Serial.print("Received packet '");
    while (LoRa.available()) {
      datos += (char)LoRa.read();
    }
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }

  for(int i = 0; i < datos.length(); i++) {
    if (datos[i] == ';') {
      counter++;
    }
    else {
      if (counter == 0) {
        latitud += datos[i];
      }
      else {
        longitud += datos[i];
      }
    }
  }
  float suLatitud = latitud.toFloat();
  float suLongitud = longitud.toFloat();

  suLongitud += 180;
  suLatitud += 90;
  nuestraLongitud += 180;
  nuestraLatitud += 90;

  Serial.print(suLongitud);
  Serial.println("º (Longitud");
  Serial.print(suLatitud);
  Serial.println("º (Latitud)");


  float distlat = (nuestraLatitud - suLatitud);
  float distlon = (nuestraLongitud - suLongitud);

  if (distlat < 0){
    distlat *= -1;
  }
  if (distlon < 0){
    distlon *= -1;
  }
  
  Serial.print(distlat);
  Serial.println("º");
  Serial.print(distlon);
  Serial.println("º");

  distlat *= 111.3194 * 2;
  distlon *= 111.3194;

  float distancia = sqrt(pow(distlat, 2) + pow(distlon, 2));
  distancia *= 1000;
  Serial.print(distancia);
  Serial.println("m");
  distancia /= 1000;
  
  Serial.print(distlat);
  Serial.println("km");
  Serial.print(distlon);
  Serial.println("km");

  float dif = distlat/distancia;

  float alpha = acos(dif);
  float angulo = (alpha*180)/PI;
  Serial.print(angulo);
  Serial.println("º con respecto al norte");  
  
  Serial.print("\n");
  delay(5000);
}
