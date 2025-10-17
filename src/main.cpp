#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h" 

const uint8_t INTERRUPT_PIN = 2;
const uint32_t BAUD_RATE = 1600000;

void interrupt_callback(void);
bool interrupt;

Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> acc;

void setup(void)
{
  Serial.begin(BAUD_RATE);     
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!bno.setAccRange( bno.BNO055_ACC_CONFIG_2G ))
  {
    Serial.print("Ooops, changing Acc range seems to be refused!");    
  }      
  interrupt = false;
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt_callback, RISING);   
  bno.enableAnyMotion(0, 0);  
  bno.enableInterruptsOnXYZ(ENABLE, ENABLE, ENABLE);
  bno.setExtCrystalUse(true);    
}

void loop(void)
{
  if (interrupt) {
    acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    
    Serial.printf("%.2f,%.2f,%.2f\n", acc.x(), acc.y(), acc.z());
  }
  else
  {
    bno.resetInterrupts();
    interrupt = false;
  }
}

void interrupt_callback(void)
{
  interrupt = true;
}