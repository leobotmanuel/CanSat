//Importamos las librerias
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include <Adafruit_BME280.h>
#include <math.h>
#include "Adafruit_CCS811.h"
#include <ML8511.h>
#include <Adafruit_GPS.h>

//Definimos los pines del modulo LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//Definimos pines del sensor UV
#define ANALOGPIN A0
#define ENABLEPIN 7

//Definimos la banda de LoRa
#define BANDA 868E6

//Definimos el pin del GPS
#define GPSSerial Serial2 

//Conectar GPS
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

//Contador de paquetes LoRa
int contador = 0;

LSM6 ag;
LIS3MDL mag;
LPS pta;

//Variables para almanecer los datos del giroscopio(presion, temperatura y altura)
float presion_giroscopio;
float temperatura_giroscopio;
float altura_giroscopio;

//Adaptar nombre de las librerias
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

//Activar el sensor UV
ML8511 sensorUV(ANALOGPIN, ENABLEPIN);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  GPS.Begin(9600); 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
  
  sensorUV.enable();
  iniciar_giroscopio();
  iniciar_magnetometro();
  iniciar_barometro();
  iniciarLora();
}



void loop() {
  String datos_del_CanSat = crear_cadena();
  enviar_por_LoRa(datos_del_CanSat);
  delay(1000);
  
  
}

void iniciar_giroscopio() {
  if (!ag.init()) {
    Serial.println("Error al iniciar el giroscopio!");
    while (1);
  }
  ag.enableDefault();
}


void iniciar_magnetometro() {
  if (!mag.init()) {
    Serial.println("Error al iniciar el magnetometro!");
    while (1);
  }
  mag.enableDefault();
}

void iniciar_barometro() {
  if (!pta.init()) {
    Serial.println("Error al iniciar el sensor de presion!");
    while (1);
  }
  pta.enableDefault();
}


void iniciarLora() {
  //Pines SPI de LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  //Pines LoRa
  LoRa.setPins(SS, RST, DIO0);

  //Iniciamos LoRa
  if (!LoRa.begin(BANDA)) {
    Serial.println("Error al inicial LoRa!");
    while (1);
  }
  Serial.println("LoRa iniciada correctamente!");
  delay(2000);
}


String datos_del_bme() {
  //Leer los valores del bme
  sensors_event_t temp_event, pressure_event, humidity_event;
  float temperatura_bme = bme.readTemperature();
  float presion_bme = bme.readPressure();
  float humedad_bme = bme.readHumidity();
  float altitud_bme = bme.readAltitude(1035.5);
  String valores_bme = String(temperatura_bme);
  valores_bme += ",";
  valores_bme += String(presion_bme);
  valores_bme += ",";
  valores_bme += String(humedad_bme);
  valores_bme += ",";
  valores_bme += String(altitud_bme);
  return valores_bme;
}

String datos_del_GPS() {
  float latitud = GPS.lat;
  float longitud = GPS.lon;
  float velocidad = GPS.speed;
  float altitud = GPS.altitude; 
  String valores_GPS = String(latitud);
  valores_GPS += ",";
  valores_GPS += String(longitud);
  valores_GPS += ",";
  valores_GPS += String(velocidad);
  valores_GPS += ",";
  valores_GPS += String(altitud);
  return valores_GPS;
}
  

String datos_del_giroscopio() {
  ag.read();
  mag.read();
  presion_giroscopio = pta.readPressureMillibars();
  temperatura_giroscopio = pta.readTemperatureC();
  altura_giroscopio = pta.pressureToAltitudeMeters(presion_giroscopio);
  const char floatsize = 7;
  const char decimalsize = 3;
  char reporte[255];
  char presion_giroscopio_str[floatsize + 1];
  char temperatura_giroscopio_str[floatsize + 1];
  char altura_giroscopio_str[floatsize + 1];
  dtostrf(presion_giroscopio, floatsize, decimalsize, presion_giroscopio_str);
  dtostrf(temperatura_giroscopio, floatsize, decimalsize, temperatura_giroscopio_str);
  dtostrf(altura_giroscopio, floatsize, decimalsize, altura_giroscopio_str);
  snprintf(reporte, sizeof(reporte), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%s",
           ag.a.x, ag.a.y, ag.a.z,
           ag.g.x, ag.g.y, ag.g.z,
           mag.m.x, mag.m.y, mag.m.z,
           presion_giroscopio_str, temperatura_giroscopio_str, altura_giroscopio_str);
           return reporte;
}

String datosDelAire() {
  if(ccs.available()){
    if(!ccs.readData()){
      float CO2 = ccs.geteCO2;
      float gv = ccs.getTVOC;
    }
  }
  String datosAire = String(CO2);
  datosAire += ",";
  datosAire += String(gv);
  return datosAire;
}


string datosUV() {
  UV = sensorUV.getUV();
  float duv = UV / .2;
  String strduv = String(duv);
  return strduv;
}

void configurar_GPS() {
  char c = GPS.read();
  if (GPSECHO)
  if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
  Serial.print(GPS.lastNMEA());
  if (!GPS.parse(GPS.lastNMEA()))
  return; 
  }


String crear_cadena() {
  //Creamos la cadena de datos para enviar
  String datos = String(millis()/1000);
  datos += ",";
  datos += datos_del_giroscopio();
  datos += ",";
  datos += datos_del_GPS();
  datos += ",";
  datos += datos_del_bme();
  datos += ",";
  datos += datosDelAire();
  datos += ",";
  datos += datosUV();
  Serial.println(datos);
  return datos;
}


void enviar_por_LoRa(String datos) {
  //Enviamos paquete de datos
  Serial.print("Enviando paquete...");
  Serial.println(contador);
  LoRa.beginPacket();
  LoRa.print(datos);
  LoRa.endPacket();
  contador += 1;
  delay (1000);
} 
