#include <OneWire.h>
#include <SD.h>
#include <SPI.h>

// Sensor array control and logging module
// Modified by Benjamin Doblack on:
// 2015/08/12
// 2015/08/13
// 2015/08/14
// 2015/08/15
// 2015/08/16
// Modified by Brian Sarracino-Aguilera
// 2015/08


OneWire  ds(9);  // on pin 9 (a 4.7K resistor is necessary)
byte i, j;
byte T[9][8] = {};
const byte Sensors = 9;
byte Tsensor = 0;
const byte MUXPsensors = 8;
const byte SOLOPsensors = 1; // max 4
byte Psensor = 0;
uint16_t P[MUXPsensors+SOLOPsensors] = {};

unsigned long cT = 0;

// Photoresistor deMUX pin
const byte inputPR1_8 = A0;
// Photoresistor solo pins
const byte inputPR9_12[4]  = {A1, A2, A3, A4};

// variables for S0-2 pin mappings on 74HC4051
const byte S0 = 7;
const byte S1 = 6;
const byte S2 = 5;

// IC state tracking variable (counter)
byte muxState = 0;

const int chipSelect = 4;

void setup(void) {
  for ( i = 0; i < 9; i++) {
    T[i][0] = 0x28;
    T[i][1] = 0xFF;
  }

  T[0][2] = 0x08; T[0][3] = 0x5E; T[0][4] = 0x51; T[0][5] = 0x15; T[0][6] = 0x03; T[0][7] = 0x9A;
  T[1][2] = 0x58; T[1][3] = 0xC9; T[1][4] = 0x01; T[1][5] = 0x15; T[1][6] = 0x02; T[1][7] = 0x4D;
  T[2][2] = 0xF4; T[2][3] = 0x78; T[2][4] = 0x51; T[2][5] = 0x15; T[2][6] = 0x03; T[2][7] = 0x64;
  T[3][2] = 0xE6; T[3][3] = 0xEA; T[3][4] = 0x02; T[3][5] = 0x15; T[3][6] = 0x03; T[3][7] = 0xE2;
  T[4][2] = 0xC5; T[4][3] = 0x6A; T[4][4] = 0x51; T[4][5] = 0x15; T[4][6] = 0x03; T[4][7] = 0x12;
  T[5][2] = 0xCD; T[5][3] = 0x65; T[5][4] = 0x51; T[5][5] = 0x15; T[5][6] = 0x03; T[5][7] = 0xB6;
  T[6][2] = 0x2C; T[6][3] = 0x47; T[6][4] = 0x51; T[6][5] = 0x15; T[6][6] = 0x03; T[6][7] = 0xD6;
  T[7][2] = 0xBB; T[7][3] = 0x75; T[7][4] = 0x51; T[7][5] = 0x15; T[7][6] = 0x03; T[7][7] = 0x7F;
  T[8][2] = 0x41; T[8][3] = 0xC8; T[8][4] = 0x01; T[8][5] = 0x15; T[8][6] = 0x02; T[8][7] = 0x4D;

  // initialize photoresistor pin as input
  pinMode(inputPR1_8, INPUT);
  for(i=0; i<SOLOPsensors; i++) {
    pinMode(inputPR9_12[i], INPUT);
  }
  // initialize PWM pins for de/mux control
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  Serial.begin(9600);

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Device #;Temperature in C;Photoresistor Intensity 0-1023;t(ms)");
    dataFile.close();
    Serial.println("Device #;Temperature in C;Photoresistor Intensity 0-1023;t(ms)");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

byte toggle = 0;

void loop(void) {
  byte data[12];
  byte *addr = T[Tsensor];
  float celsius;

  if ( Tsensor == 0 ) {
    ds.reset();
    ds.skip();
    ds.write(0x44, 1);        // start conversion
    cT= millis();
  }

  while(Psensor < MUXPsensors) {
    P[Psensor++] = analogRead(inputPR1_8);
    pinModeSet(muxState);
    muxState = incMUXState(muxState);
    delay(5);
  }
  while(Psensor-MUXPsensors < SOLOPsensors) {
    P[Psensor] = analogRead(inputPR9_12[Psensor-MUXPsensors]);
    Psensor++;
    delay(5);
  }
  Psensor=0;
  delay(550);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datadump.txt", FILE_WRITE);

  // if the file isn't open, pop up an error:
  if(!dataFile) {
    Serial.println("error opening datalog.txt");
    return;
  }

  for ( i = 0; i < Sensors; i++) {
    dataFile.print(i + 1);
    Serial.print(i+1);
    dataFile.print(";");
    Serial.print(";");

    if (OneWire::crc8(addr, 7) != addr[7]) {
      dataFile.print("!!!BAD CRC:IGNORE LINE");
      dataFile.println();
      return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    for ( j = 0; j < 9; j++) {           // we need 9 bytes
      data[j] = ds.read();
    }
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);

    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms

    celsius = (float)raw / 16.0;
    dataFile.print(celsius);
    Serial.print(celsius);
    dataFile.print(";");
    Serial.print(";");
    dataFile.print(P[i]);
    Serial.print(P[i]);
    dataFile.print(";");
    Serial.print(";");
    dataFile.print(cT);
    Serial.println(cT);
    dataFile.println();
    addr = nextSensor();
  }
  dataFile.close();
}

byte *nextSensor() {
  Tsensor++;
  if (Tsensor >= Sensors)
    Tsensor = 0;
  return T[Tsensor];
}

byte incMUXState(byte in) {
  if ( in >= 7 )
    return 0;
  return in + 1;
}

byte decMUXState(byte in) {
  if ( in == 0 )
    return 7;
  return in - 1;
}

void pinModeSet(byte in) {
  if ( in & 0x01 )
    digitalWrite(S0, LOW);
  else
    digitalWrite(S0, HIGH);
  if ( in & 0x02 )
    digitalWrite(S1, LOW);
  else
    digitalWrite(S1, HIGH);
  if ( in & 0x04 )
    digitalWrite(S2, LOW);
  else
    digitalWrite(S2, HIGH);
}
