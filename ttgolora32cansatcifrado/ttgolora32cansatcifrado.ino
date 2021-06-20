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

// Pines para los servos, el LED y el zumbador
static const int servoPin = 2;
static const int servoPin2 = 23;
static const int led = 12;
static const int zumbador = 13;

// Creamos dos objetos para los servos
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
#define ANALOGPIN 36
#define ENABLEPIN 19

//Definimos la banda de LoRa
#define BANDA 868E6

//Definimos el pin del GPS
#define GPSSerial Serial2

//Conectar GPS
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

//Contador y separador de paquetes LoRa
String cadena;
String prov;
String grados[3];
int contador_r = 0;
int contador = 0;
long lastSendTime = 0;
int interval = 2000;
String datos = "1639";

// Creamos objetos para los sensores y el cifrado
LSM6 ag;
LIS3MDL mag;
LPS pta;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
AES aes;

// Creamos la clave del cifrado
byte *key = (unsigned char*)"0123456789010123";
int my_iv = 36753562;

//Variables para almanecer los datos del giroscopio(presion, temperatura y altura)
float presion_giroscopio;
float temperatura_giroscopio;
float altura_giroscopio;

// Adaptar nombre de las librerias
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

// Activar el sensor UV
ML8511 sensorUV(ANALOGPIN, ENABLEPIN);

// Control de la batería
int sensorPin = 36;
float sensorValue;
float voltajePila;
float porcentaje;

void setup() {
  // Iniciamos el Serial a 115200 baudios
  Serial.begin(115200);

  // Declaramos los pines del LED y del zumbador como pines de salida
  pinMode(led, OUTPUT);
  pinMode(zumbador, OUTPUT);

  // Iniciamos los sensores y LoRa
  Wire.begin();
  mlx.begin();
  if (!bme.begin()) {
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

  if (!ccs.begin()) {
    while (1);
  }
  while (!ccs.available());

  delay(500);
}

void loop() {
  // Leemos el GPS
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    //Serial.print(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  String cadenadef = "";
  // Apagamos el LED y el zumbador
  digitalWrite(led, LOW);
  digitalWrite(zumbador, LOW);
  if (millis() - lastSendTime > interval) {
    // Enviamos los datos de los sensores a la estación
    String datos_del_CanSat = crear_cadena();
    enviar_por_LoRa(datos_del_CanSat);

    // Encendemos el LED y el zumbador
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    digitalWrite(zumbador, HIGH);
    delay(100);
    digitalWrite(zumbador, LOW);
    lastSendTime = millis();
    interval = 1000;
  }

  // Recibimos los datos del control de vuelo
  onReceive(LoRa.parsePacket());

}



void iniciar_giroscopio() {
  if (!ag.init()) {
    while (1);
  }
  ag.enableDefault();
}


void iniciar_magnetometro() {
  if (!mag.init()) {
    while (1);
  }
  mag.enableDefault();
}

void iniciar_barometro() {
  if (!pta.init()) {
    while (1);
  }
  pta.enableDefault();
}


void iniciarLora() {
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BANDA)) {
    while (1);
  }
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
  if (GPS.fix) {
    float latitud = GPS.latitude;
    float longitud = GPS.longitude;
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
  if (ccs.available()) {
    if (!ccs.readData()) {
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
  float Cir = mlx.readObjectTempC();
  String strCir = String(Cir);
  return strCir;
}

String crear_cadena() {
  //Creamos la cadena de datos para enviar
  datos = "1639";
  datos += ",";
  datos += datos_del_bme();
  datos += ",";
  datos += datosUV();
  datos += ",";
  datos += datosDelAire();
  datos += ",";
  datos += datos_del_giroscopio();
  datos += ",";
  datos += control_bateria();
  datos += ",";
  datos += datosIR();
  datos += ",";
  datos += datos_del_GPS();
  datos += ",";
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

  cadenadef = String((char *)cipher);
  return String(cadenadef);
}


void enviar_por_LoRa(String datos) {
  LoRa.beginPacket();
  LoRa.print(datos);
  LoRa.endPacket();
  contador += 1;
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;

  contador_r = 0;
  cadena = "";

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
  if (grados[0] == "cva") {

    servo1.write((-1)*grados[1].toInt());
    servo2.write((-1)*grados[2].toInt());
  }
}

float control_bateria() {
  sensorValue = analogRead(sensorPin);
  voltajePila = (4.36 / 1250.00) * sensorValue;
  porcentaje = (100.00 * voltajePila) / 4.36;
  return porcentaje;
}
