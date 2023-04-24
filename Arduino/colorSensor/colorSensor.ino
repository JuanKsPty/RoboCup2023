#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

void setup(){
  Serial.begin(9600); 
  if(tcs.begin()){
    Serial.println("iniciar");
    delay(2000);
  }
  else {
    Serial.println("error cacacc");
    while (1) {
      
    }
  
  }

}

void loop(){
  uint16_t r,g,b,c;
  tcs.getRawData(&r, &g, &b, &c);
  Serial.println("r= " + String(r) +", g =" + String(g) + ", b =" + String(b) + ", c =" + String(c));
  delay(1000);
}