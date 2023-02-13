#include <Wire.h>
#include "FDC2214.h"

int i=0; //Variable for serial information relay to processing
int delayRate= 3; //Delay variable to send from Serial to processing 

FDC2214 capsense(FDC2214_I2C_ADDR_0); // Use FDC2214_I2C_ADDR_1 
void setup() {
  pinMode(13,OUTPUT); 
  digitalWrite(13,LOW); //makeshift ground
  Serial.begin(115200);
  Wire.begin();
  //Wire.setClock(800000L);//fast clock
  bool capOk = capsense.begin(0xF, 0x6, 0x5, false); 
  if (capOk) Serial.println("Sensor OK");  
  else Serial.println("Sensor Fail");  

  pinMode(8,INPUT_PULLUP);
}


void loop(){

  float sensor0 = -1*capsense.getReading28(0); //Channel(0) on FDC2214
  float sensor1 = -1*capsense.getReading28(1); //Channel(1) on FDC2214
  float sensor2 = -1*capsense.getReading28(2); //Channel(2) on FDC2214
  i++; 

  if (i%delayRate==0){
    
  Serial.print(sensor0);Serial.print(",");
  Serial.print(sensor1);Serial.print(",");  
  Serial.print(sensor2);Serial.print(",");
  Serial.println();
  
  }


}

