#include <LoRa.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definimos los pines del modulo LoRa

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

// Pines para la OLED
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


// Declaramos el ancho de banda del LoRa
#define BANDA 868E6

// Declaramos las variables

int angulo1 = 0;  
int angulo2 = 0;

// Contador de paquetes LoRa
int counter = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() 
{
  Serial.begin(9600);
  iniciar_Lora();
  
  // Reseteamos la OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  // Iniciamos la OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("CONTROL VUELO");
  display.display();
}

void loop() 
{
  int joy = analogRead(A1);

  if (joy <= 2770)
  {
    
    angulo1 = map(joy, 0, 2749, 45, 0);
    
  } else {
    
    angulo1 = 0;
    
  }

  if (joy >= 2770)
  {
  
    angulo2 = map(joy, 2749, 4095, 0, 45);
    
  }else{
    
    angulo2 = 0;
    
  }
  Serial.println(joy);
  Serial.println(angulo1);
  Serial.println(angulo2);

  Serial.print("Mandando paquete: ");
  Serial.println(counter);

  // Envio de paquetes
  LoRa.beginPacket();
  LoRa.print(angulo1);
  LoRa.print(angulo2);
  LoRa.print(counter);
  LoRa.endPacket();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("CONTROL DE VUELO");
  display.setCursor(0,20);
  display.setTextSize(1);
  display.print("Angulo1: ");
  display.setCursor(60,20);
  display.print(angulo1);
  display.setCursor(0,30);
  display.print("Angulo2: ");
  display.setCursor(60,30);
  display.print(angulo2);
  display.setCursor(0,50);
  display.print("Enviando: ");
  display.setCursor(60,50);
  display.print(counter);      
  display.display();

  counter += 1;
  delay(3000);
  
}
void iniciar_Lora() {

//Pines SPI de LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  
//Pines LoRa
  LoRa.setPins(SS, RST, DIO0);

//Iniciamos LoRa
  if (!LoRa.begin(BANDA)) 
  {
    Serial.println("Error al inicial LoRa!");
    while (1);
  }
  
  Serial.println("LoRa iniciada correctamente!");
  delay(2000);
} 
