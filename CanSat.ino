//Importamos las librerias
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>

//Librerias para la OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Definimos los pines del modulo LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//Definimos la banda de LoRa
#define BANDA 868E6

//Definimos los pines de la OLED
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16

//Definimos el display de la OLED
#define ANCHO_PANTALLA 128
#define ALTO_PANTALLA 64

//Contador de paquetes
int contador = 0;

LSM6 ag;
LIS3MDL mag;
LPS pta;

float pressure;
float temperature;
float altitude;

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


void setup() {


  //Imprimimos mensaje de inicio en la OLED
  Serial.begin(9600);
  Serial.println("iniciando...");

  Wire.begin();
  ag_init();
  mag_init();
  pta_init();

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

String altimu10() {
  const char floatsize = 7;
  const char decimalsize = 3;
  char report[255];
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

  //Creamos la cadena de datos para enviar
  String datos = String(millis()/1000);
  datos += ",";
  datos += String(altimu10());
  Serial.println(datos);
  //Enviamos paquete de datos
  Serial.print("Enviando paquete...");
  Serial.println(contador);
  LoRa.beginPacket();
  LoRa.print(datos);
  LoRa.endPacket();
  contador += 1;
  delay (1000);
}
