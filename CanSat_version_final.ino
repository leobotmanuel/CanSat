  //Importamos las librerias
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>

//Definimos los pines del modulo LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//Definimos la banda de LoRa
#define BANDA 868E6

//Contador de paquetes LoRa
int contador = 0;

LSM6 ag;
LIS3MDL mag;
LPS pta;

//Variables para almanecer los datos del giroscopio(presion, temperatura y altura)
float presion_giroscopio;
float temperatura_giroscopio;
float altura_giroscopio;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  iniciar_giroscopio();
  iniciar_magnetometro();
  iniciar_barometro();
  iniciarLora();
}

void loop() {
  leer_sensores();
  String datos_del_CanSat = crear_cadena();
  enviar_por_LoRa(datos_del_CanSat);
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


void leer_sensores() {
  //Leer el valor del giroscopio
  ag.read();
  //Leer el valor del magnetometro
  mag.read();
  //Leer la presion, la temperatura y la altura del giroscopio
  presion_giroscopio = pta.readPressureMillibars();
  temperatura_giroscopio = pta.readTemperatureC();
  altura_giroscopio = pta.pressureToAltitudeMeters(presion_giroscopio);
}


String datos_del_giroscopio() {
  const char floatsize = 7;
  const char decimalsize = 3;
  char reporte[255];
  char presion_giroscopio_str[floatsize + 1];
  char temperatura_giroscopio_str[floatsize + 1];
  char altura_giroscopio_str[floatsize + 1];
  dtostrf(presion_giroscopio, floatsize, decimalsize, presion_giroscopio_str);
  dtostrf(temperatura_giroscopio, floatsize, decimalsize, temperatura_giroscopio_str);
  dtostrf(altura_giroscopio, floatsize, decimalsize, altura_giroscopio_str);
  snprintf(report, sizeof(report), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%s",
           ag.a.x, ag.a.y, ag.a.z,
           ag.g.x, ag.g.y, ag.g.z,
           mag.m.x, mag.m.y, mag.m.z,
           presion_giroscopio_str, temperatura_giroscopio_str, altura_giroscopio_str);
           return reporte;
}


String crear_cadena() {
  //Creamos la cadena de datos para enviar
  String datos = String(millis()/1000);
  datos += ",";
  datos += String(datos_del_giroscopio());
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
