#include "MySD.h"

void setup() {
  Serial.begin(BAUD_RATE);
  sd_setup();

}

void loop() {
  writeFile(SD, "/bno.txt", "Hello ");
  appendFile(SD, "/bno.txt", "World!\n");
  readFile(SD, "/bno.txt"); 
}
