#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Arduino.h"
#include "MyBno_filters.h"
#include "MyBno_config.h"

#include <SD.h>

const uint8_t INTERRUPT_PIN = 2; // GPIO do ESP32
/*
Uncomment and set up if you want to use custom pins for the SPI communication
#define REASSIGN_PINS
int sck = -1;
int miso = -1;
int mosi = -1;
int cs = -1;
*/

#define BUFFER_SIZE 1024
File dataFile;
char bufferA[BUFFER_SIZE];
char bufferB[BUFFER_SIZE];
char *activeBuffer = bufferA;
char *writeBuffer = bufferB;
volatile int bufferIndex = 0;
volatile bool bufferReady = false;
volatile bool writingBuffer = false;

Adafruit_BNO055 bno = Adafruit_BNO055();
struct BnoState
{
  imu::Vector<3> acc;
  unsigned long time, start_time;
  float ax, ay, az;
  float axf, ayf, azf;
  float axff, ayff, azff;  
} bnoState;

int samples;

void interrupt_callback(void);
volatile bool interrupt;

void bno_setup(void);
void bno_read(void);
void sd_setup(void);
void sd_write(void);

void setup(void)
{
  Serial.begin(BAUD_RATE);
  bno_setup();
  sd_setup();
  bnoState.start_time = millis()/1000.00;
}

void loop(void)
{
  if (interrupt)
  {
    bno_read();
    sd_write();  
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
  samples++;
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
  int len = snprintf(activeBuffer + bufferIndex, BUFFER_SIZE - bufferIndex,
                     "%.3f,%.2f,%.2f,%.2f\n",
                     bnoState.time, bnoState.axff, bnoState.ayff, bnoState.azff);

  // Verifica se cabe no buffer atual
  if (bufferIndex + len >= BUFFER_SIZE) {
    Serial.print("Samples:");Serial.println(samples);
    Serial.print("Time:");Serial.println(bnoState.time-bnoState.start_time);
    samples = 0;    
    // Troca de buffers (double buffering)
    if (!writingBuffer) {
      writingBuffer = true;
      char* temp = activeBuffer;
      activeBuffer = writeBuffer;
      writeBuffer = temp;
      bufferReady = true;
      bufferIndex = 0;
    } else {
      // Se o outro buffer ainda está sendo gravado, descarta amostra (overflow)
      return;
    }
  } else {
    bufferIndex += len;
  }

}

void sd_setup(void) {

  #ifdef REASSIGN_PINS
    SPI.begin(sck, miso, mosi, cs);
    if (!SD.begin(cs)) {
  #else
    if (!SD.begin()) {
  #endif
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);  
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  dataFile = SD.open("/bno_data.csv", FILE_WRITE, true);
  if (!dataFile) {
    Serial.println("Erro ao abrir SD!");
    return;
  }     
  #ifndef READ_SD_FILE
    #define READ_SD_FILE 0
  #endif
  if (READ_SD_FILE) {
    dataFile = SD.open("/bno_data.csv", FILE_READ);
    if (!dataFile) {
    Serial.println("Failed to open file for reading");
    return;
  }
    while (dataFile.available()) {
    Serial.write(dataFile.read());
    }
    dataFile.close();
  }   
}


void sd_write() {
  if (bufferReady) {
    dataFile = SD.open("/bno_data.csv", FILE_APPEND);
    if (dataFile) {
      dataFile.write((const uint8_t*)writeBuffer, bufferIndex);
      dataFile.close();
    }
    writingBuffer = false;
    bufferReady = false;
  }
}