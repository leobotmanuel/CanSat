//Importamos las librerias
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <math.h>
#include "Adafruit_CCS811.h"
#include <ML8511.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MLX90614.h>
#include <AES.h>
#include <Servo.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

static const int servoPin = 2; //  works with TTGO
static const int servoPin2 = 23; // works with TTGO

Servo servo1;
Servo servo2;

//Definimos los pines del modulo LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//Definimos pines del sensor UV
#define ANALOGPIN A0
#define ENABLEPIN 19

//Definimos la banda de LoRa
#define BANDA 868E6

//Definimos el pin del GPS
#define GPSSerial Serial2 

//Conectar GPS
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

//Contador de paquetes LoRa
String cadena;
String prov;
String grados[3];
int contador_r = 0;
int contador = 0;

LSM6 ag;
LIS3MDL mag;
LPS pta;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
AES aes;

byte *key = (unsigned char*)"0123456789010123";
int my_iv = 36753562;

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
  Serial.begin(115200);
  Serial.println("hola");
  Wire.begin();
  mlx.begin(); 
  if (!bme.begin()) {
    Serial.println(F("No se ha encontrado el sensor BME280"));
    while (1) delay(10);
  }
  GPS.begin(9600); 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
  
  iniciar_giroscopio();
  iniciar_magnetometro();
  iniciar_barometro();
  iniciarLora();

  servo1.attach(servoPin);
  servo2.attach(servoPin2);
  
    if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  while(!ccs.available());

  delay(500);
}

void loop() {
  configurar_GPS();
  String datos_del_CanSat = crear_cadena();
  enviar_por_LoRa(datos_del_CanSat);
  delay(2000);
  
  
}

void control_vuelo() {
  contador_r = 0;
  cadena = "";
  
  int cualquiermierda = LoRa.parsePacket();
  if (cualquiermierda) {

    while (LoRa.available()) {
      cadena.concat((char)LoRa.read());
    }
   
    for (int h = 0; h < cadena.length(); h++) {
      if (cadena[h] != ',') {
        prov = prov + cadena[h];
      } else if (cadena[h] == ',') {
        grados [contador_r] = prov;
        prov = "";
        contador_r ++;
      }
    }
    if (grados[0] == "puto el que lo lea"){
      Serial.println(grados[1]);
      Serial.println(grados[2]);

      servo1.write(grados[1].toInt());
      servo2.write(grados[2].toInt());
    }
  }
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
  float temperatura_bme = bme.readTemperature();
  float presion_bme = bme.readPressure();
  float humedad_bme = bme.readHumidity();
  float altitud_bme = bme.readAltitude(1013.25);
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
      float co = ccs.geteCO2();
      float gv = ccs.getTVOC();
      String datosAire = String(co);
      datosAire += ",";
      datosAire += String(gv);
      return datosAire;
    }
   }
}


String datosUV() {
  float UV = sensorUV.getUV();
  float duv = UV * .1;
  String strduv = String(duv);
  return strduv;
}

String datosIR() {
  double Cir = mlx.readObjectTempC();
  String strCir = String(Cir);
  return strCir;
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
}


String crear_cadena() {
  //Creamos la cadena de datos para enviar
  String datos = datos_del_bme();
  datos += ",";
  datos += datosUV();
  datos += ",";
  datos += datos_del_GPS();
  datos += ",";
  datos += datosDelAire();
  datos += ",";
  datos += datos_del_giroscopio();
  datos += ",";
  datos += datosIR();
  Serial.println(datos);

  //Ciframos los datos
  
  const char *plain_ptr = datos.c_str();
  int plainLength = datos.length();
  int padedLength = plainLength + N_BLOCK - plainLength % N_BLOCK;
  byte iv [N_BLOCK] ;
  byte cipher [padedLength] ;
  String cadenadef;
  String iv_cambia;
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  aes.do_aes_encrypt((unsigned char*)plain_ptr, plainLength, cipher, key, 128, iv);
  for (int i = 0; i < 16; i++)
  {
    iv_cambia += iv[i];
  }
  //cadenadef = iv_cambia;
  cadenadef = String((char *)cipher);
  Serial.println(cadenadef);
  return cadenadef;
  
}

void enviar_por_LoRa(String cadenadef) {
  //Enviamos paquete de datos
  Serial.print("Enviando paquete...");
  Serial.println(contador);
  LoRa.beginPacket();
  LoRa.print(cadenadef);
  LoRa.endPacket();
  contador += 1;
}
