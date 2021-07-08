#include <SpritzCipher.h>

//Declaramos la clave de cifrado
char testKey[3] = { 0x00, 0x01, 0x02 };

void setup() {
  Serial.begin(9600);
}

void loop() {

  //Declaramos la string de datos del cansat
  String datos = "data,cansat";

  //Declaramos la string de los datos cifrados que enviaremos por LoRa
  String datos_cif;

  //Declaramos la string de los datos descifrados
  String datos_plain;
  
  //Declaramos el tamaño de la cadena de datos del cansat
  int leng = datos.length() + 1;

  //Declaramos el buffer donde se guardaran los datos para cifar
  char testMsg[leng];

  // Pasamos la string de datos a bytes y lo guardamos en el buffer
  datos.toCharArray(testMsg, leng);

  //Llamamos a la funcion que va a cifrar/descifrar los datos
  testFunc(testMsg, sizeof(testMsg), testKey, sizeof(testKey), datos_cif);

  delay(5000);
  Serial.println();
}
void testFunc(const byte *msg, byte msgLen, const byte *key, byte keyLen, String datos_cif)
{
  spritz_ctx s_ctx;

  //Declaramos el output buffer de cifrado
  char buf[50];
  char bufcif[50];
  unsigned int i;

  //Ciframos los datos del cansat
  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, msg, msgLen, buf);

  for (i = 0; i < msgLen; i++) {
    datos_cif += String(buf[i]);
  }

  Serial.print("cadena cifrada: ");
  Serial.println(datos_cif);

  datos_cif.toCharArray(bufcif, 50);
  
  //Desciframos los datos del cansat en la estacion de tierra
  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, bufcif, msgLen, bufcif);

  /* Print MSG after decryption */
  for (i = 0; i < msgLen; i++) {
    Serial.write(bufcif[i]);
  }
  Serial.println(); 
}
