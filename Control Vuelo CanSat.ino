#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

static const int servoPin = 34; //  works with TTGO
static const int servoPin2 = 35; // works with TTGO

Servo servo1;
Servo servo2;

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

String cadena;
String prov;
String grados[3];
int contador = 0;

void setup() {

  Serial.begin(9600);

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  servo1.attach(servoPin);
  servo2.attach(servoPin2);
}

void loop() {

  contador = 0;
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
        grados [contador] = prov;
        prov = "";
        contador ++;
      }
    }
    if (grados[0] == "puto el que lo lea"){
      Serial.println(grados[1]);
      Serial.println(grados[2]);

      servo1.write(int(grados[1]));
      servo2.write(int(grados[2]));
    } 
  }
}
