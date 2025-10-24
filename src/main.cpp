#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Arduino.h"
#include "MyBno_filters.h"
#include "MyBno_config.h"
#include "MySD.h"

const uint8_t INTERRUPT_PIN = 2; // GPIO do ESP32

Adafruit_BNO055 bno = Adafruit_BNO055();
struct BnoState
{
  imu::Vector<3> acc;
  uint32_t start_time, count;
  float time;
  float ax, ay, az;
  float axf, ayf, azf;
  float axff, ayff, azff;
  char dataMessage[50];
} bnoState;

void interrupt_callback(void);
volatile bool interrupt;

void bno_setup(void);
void bno_read(void);

void setup(void)
{
  Serial.begin(BAUD_RATE);
  bno_setup();
  sd_setup();
  bnoState.start_time = millis();
}

void loop(void)
{
  if (interrupt)
  {
    bno_read();
    SD.begin();  
    appendFile(SD, "/bno.txt", bnoState.dataMessage);
    if( millis() - bnoState.start_time >= 10000) {
      Serial.print( "Start time: ");    Serial.println( bnoState.start_time); 
      Serial.print( "End time: ");      Serial.println( millis()); 
      Serial.print( "Total time (s): ");Serial.println( (millis() - bnoState.start_time) / 1000.0); 
      Serial.print( "Total samples: "); Serial.println( bnoState.count);      
      Serial.println("end");
      delay(50000);
    }     
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

void bno_setup(void)
{
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    ESP.restart();
  }
  if (!bno.setAccRange(ACC_RANGE))
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

void bno_read(void)
{
  bnoState.acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bnoState.time = (millis()-bnoState.start_time)/1000.00;
  bnoState.count++;
  bnoState.ax = bnoState.acc.x();
  bnoState.ay = bnoState.acc.y();
  bnoState.az = bnoState.acc.z();
#if FILTER_ORDER_EMA_FIRST
  filterEMA(bnoState.ax, bnoState.ay, bnoState.az, EMA_ALPHA,
            bnoState.axf, bnoState.ayf, bnoState.azf);

  filterMedianN(bnoState.axf, bnoState.ayf, bnoState.azf,
                bnoState.axff, bnoState.ayff, bnoState.azff);
#else
  filterMedianN(bnoState.ax, bnoState.ay, bnoState.az,
                bnoState.axf, bnoState.ayf, bnoState.azf);

  filterEMA(bnoState.axf, bnoState.ayf, bnoState.azf, EMA_ALPHA,
            bnoState.axff, bnoState.ayff, bnoState.azff);
#endif
  //Serial.printf("%.2f,%.2f,%.2f\n", bnoState.axff, bnoState.ayff, bnoState.azff);
  sprintf( bnoState.dataMessage, "%.3f,%.2f,%.2f,%.2f\n" , bnoState.time, bnoState.axff, bnoState.ayff, bnoState.azff);
}