
#include <SpritzCipher.h>




String datos = "data,cansat";
byte testMsg[30];
const byte testKey[3] = { 0x00, 0x01, 0x02 };


void testFunc(const byte *msg, byte msgLen, const byte *key, byte keyLen)
{
  spritz_ctx s_ctx;
  byte buf[8]; 
  unsigned int i;


  for (i = 0; i < msgLen; i++) {
    Serial.write(msg[i]);
  }
  Serial.println();


  for (i = 0; i < keyLen; i++) {
    if (key[i] < 0x10) { 
      Serial.write('0');
    }
    Serial.print(key[i], HEX);
  }
  Serial.println();

  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, msg, msgLen, buf);


  for (i = 0; i < msgLen; i++) {
    if (buf[i] < 0x10) { 
      Serial.write('0');
    }
    Serial.print(buf[i], HEX);
  }
  Serial.println();

  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, buf, msgLen, buf);


  for (i = 0; i < msgLen; i++) {
    Serial.write(buf[i]);
  }
  Serial.println();


  if (spritz_compare(buf, msg, msgLen)) {

    digitalWrite(LED_BUILTIN, HIGH); 
    Serial.println("\n** WARNING: Output != Test_Vector **");
  }
  Serial.println();
}

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ; 
  }
datos.getBytes(testMsg, 30);
Serial.println(testMsg[0]);  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  Serial.println("[Spritz library encryption/decryption function test]\n");


  testFunc(testMsg, sizeof(testMsg), testKey, sizeof(testKey));

  delay(5000); 
  Serial.println();
}
