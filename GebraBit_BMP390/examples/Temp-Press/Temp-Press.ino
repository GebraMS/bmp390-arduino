#include "GebraBit_BMP390.h"

GebraBit_BMP390 BMP390;

void setup() {
  
  Serial.begin(9600);

  SPI.begin();
  
  GB_BMP390_initialize( &BMP390 );
  
  GB_BMP390_Configuration(&BMP390, FIFO_DISABLE) ;
  
}

void loop() {

  GB_BMP390_Get_Data(&BMP390, FROM_REGISTER);

  Serial.print("Compensated Temperature: ");
  Serial.print(BMP390.COMPENSATED_TEMPERATURE);
  Serial.println(" °C");
  
  Serial.print("Compensated Pressure: ");
  Serial.print(BMP390.COMPENSATED_PRESSURE);
  Serial.println(" mBar");
  
  delay(1000);

}

