#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define data_size 8

unsigned long start_time = 0;
bool time_set = false;

void setup() {
  Wire.begin(0x20);                // join i2c bus with address #9
  Wire.onReceive(log_data); // register event

  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect.

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");
}

void log_data(int howMany) {
  if (!time_set) {
    start_time = millis();
    time_set = true; 
  }
  unsigned char data[data_size];
  for (unsigned char i = 0; i < data_size; i++) {
    data[i] = Wire.read();
  }
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println((millis() - start_time) / 1000);
    for (unsigned char i = 0; i < data_size; i++) {
      dataFile.print("  ");
      dataFile.print(data[i]);
    }
    dataFile.close();
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}
