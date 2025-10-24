#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Arduino.h"
#include "MyBno_filters.h"
#include "MyBno_config.h"
#include "MySD.h"

const uint8_t INTERRUPT_PIN = 2; // GPIO do ESP32
const uint8_t CS_PIN = 5;
#define BUFFER_SIZE 1024

Adafruit_BNO055 bno = Adafruit_BNO055();
struct BnoState
{
  imu::Vector<3> acc;  
  float ax, ay, az;
  float axf, ayf, azf;
  float axff, ayff, azff;
  float time;
  char dataMessage[50];
  uint32_t start_time, count ;  
} bnoState;

char buffer[BUFFER_SIZE];
volatile size_t bufferIndex = 0;
File dataFile;

void interrupt_callback(void);
volatile bool interrupt;

void bno_setup(void);
void bno_read(void);

void setup(void)
{
  Serial.begin(BAUD_RATE);

  bno_setup();

  if (!SD.begin(CS_PIN)) {
    Serial.println("Falha ao iniciar SD!");
    while (1);
  }
  dataFile = SD.open("/bno.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Erro ao abrir arquivo no SD!");
    while (1);
  }
  Serial.println("Inicialização concluída!");

  bnoState.start_time = millis();
}

void loop(void)
{
  if (interrupt)
  {
    bno_read();
    String sample = bnoState.dataMessage;

    // Armazena no buffer
    size_t len = sample.length();
    if (bufferIndex + len >= BUFFER_SIZE) {
      // buffer cheio → grava no SD
      dataFile.write((uint8_t*)buffer, bufferIndex);
      dataFile.flush();
      bufferIndex = 0;  // zera para começar a encher de novo
    }

    // copia a amostra pro buffer
    memcpy(&buffer[bufferIndex], sample.c_str(), len);
    bufferIndex += len;

    if( millis() - bnoState.start_time >= 10000) {
      dataFile.close();
      // Ler arquivo
      Serial.println("\n--- LENDO /bno.txt ---");
      File readFile = SD.open("/bno.txt", FILE_READ);
      if (readFile) {
        while (readFile.available()) Serial.write(readFile.read());
        readFile.close();
      }
      Serial.println("\n--- FIM ---");
      
      Serial.print( "Start time: ");    Serial.println( bnoState.start_time/1000.00); 
      Serial.print( "End time: ");      Serial.println( bnoState.time); 
      Serial.print( "Total time (s): ");Serial.println( bnoState.time - bnoState.start_time/1000.00); 
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