#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Arduino.h"
#include "MyBno_filters.h"
#include "MyBno_config.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"


const uint8_t INTERRUPT_PIN = 2; // GPIO do ESP32

Adafruit_BNO055 bno = Adafruit_BNO055();
struct BnoState
{
  imu::Vector<3> acc;
  unsigned long time, start_time;
  float ax, ay, az;
  float axf, ayf, azf;
  float axff, ayff, azff;
  char dataMessage[50];
} bnoState;

void interrupt_callback(void);
volatile bool interrupt;

void bno_setup(void);
void bno_read(void);
void sd_setup(void);
void appendFile(fs::FS &fs, const char *path, const char *message);
void readFile(fs::FS &fs, const char *path);

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
    appendFile(SD, "/data.txt", bnoState.dataMessage);
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
  sprintf( bnoState.dataMessage, "%.3f,%.2f,%.2f,%.2f\n" , bnoState.time, bnoState.axff, bnoState.ayff, bnoState.azff);
  Serial.printf("%.2f,%.2f,%.2f\n", bnoState.axff, bnoState.ayff, bnoState.azff);  
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
   
  appendFile(SD, "/data.txt", "Inicio:\n");
  #ifndef READ_SD_FILE
    #define READ_SD_FILE 0
  #endif
  if (READ_SD_FILE) {readFile(SD, "/data.txt");}  
  
}



void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
}