#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

// Pin assignment
const int Vi_pin=A1;
const int Vs_pin=A2;
const int Vz_pin=A3;
const int Vr_pin=A4;

const byte DDS_RESET_pin = 6; // Serial bit sent to DDS
const byte DDS_DATA_pin = 10; // Serial bit sent to DDS
const byte DDS_CLOCK_pin = 11; // Loads one bit, W_CLK
const byte DDS_UPDATE_pin = 12; // Shifts register, FQ_UD

// Variable declaration
unsigned long Vi;
unsigned long Vs;
unsigned long Vz;
unsigned long Vr;
unsigned long rho;      // Reflection coefficient
unsigned long SWR;      // Standing wave ratio
unsigned long Zin_Mag;  // Magnitude of Zin
unsigned long Zin_Re;   // Real component of Zin

unsigned long max_frequency_step = 100000;  // Max Frequency
unsigned long max_frequency = 10000000;     // Max Frequency
int min_frequency = 1000000;                // Minimum Frequency


// ***
// *** Setup function
// ***
void setup() {
  Serial.begin(115200);

  // Voltage reading pins are INPUTs
  pinMode(Vi_pin,INPUT);
  pinMode(Vs_pin,INPUT);
  pinMode(Vz_pin,INPUT);
  pinMode(Vr_pin,INPUT);
  
  // DDS pins are OUTPUTs
  pinMode(DDS_RESET_pin, OUTPUT);
  pinMode(DDS_DATA_pin, OUTPUT);
  pinMode(DDS_CLOCK_pin, OUTPUT);
  pinMode(DDS_UPDATE_pin, OUTPUT);
}


// ***
// *** Main loop function
// ***
void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(DATA_pin, HIGH);
//  digitalWrite(CLOCK_pin, HIGH);
//  digitalWrite(UPDATE_pin, HIGH);
//  delay(1500);
//

  digitalWrite(DDS_DATA_pin, LOW);
  digitalWrite(DDS_CLOCK_pin, LOW);
  digitalWrite(DDS_UPDATE_pin, LOW);
  delay(1500);

  delayMicroseconds(1);
  digitalWrite(DDS_UPDATE_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(DDS_UPDATE_pin, LOW);
}


// ***
// *** Function to initialize the DDS chip
// ***
void init_dds()
{
  digitalWrite(DDS_RESET_pin, LOW);
  digitalWrite(DDS_DATA_pin, LOW);
  digitalWrite(DDS_CLOCK_pin, LOW);
  digitalWrite(DDS_UPDATE_pin, LOW);
}


// ***
// *** Function to read the four voltages
// ***
void ReadVoltages(){
  Vi=analogRead(Vi_pin);
  Vs=analogRead(Vs_pin);
  Vz=analogRead(Vz_pin);
  Vr=analogRead(Vr_pin);
}


// ***
// *** Function to convert the voltages to an impedence
// ***
void Calculate(){
  rho=Vr/(Vi/2);                                       //reflection coeff
  SWR=(1+abs(rho))/(1-abs(rho));                       //standing wave ratio
  Zin_Mag=50*Vz/Vs;                                    //magnitude of Zin
  Zin_Re = (Zin_Mag^2 + 50^2)*SWR/(50*(SWR^2+1));      //real componend of Zin
}


// ***
// *** Function to send desired frequency to DDS chip
// ***
void writeDDSchip(unsigned long freq){
  
  // Variables
  unsigned long DDSLong;
  unsigned long bitMask32 = 0b00000000000000000000000000000001;
  byte last8 = 0b00000000;
  byte bitMask8 = 0b00000001;
  float clock_freq = 40000000; // 40 MHz

  // Calculate the first 32 bits of the 40 bit DDS instruction
  DDSLong = freq * pow(2,32) / clock_freq;

  // Write frequency values (dependent on the input)
  for (bitMask32=1; bitMask32>0; bitMask32 <<= 1){ // iterate through 32 bits of DDSLong
    if (DDSLong & bitMask32) digitalWrite(DDS_DATA_pin, HIGH);
    else digitalWrite(DDS_DATA_pin, LOW);

    // Toggle clock pin for DDS to receive the data after every single bit
    digitalWrite(DDS_CLOCK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(DDS_CLOCK_pin, LOW);
  }

  // Write last 8 values (constant everytime)
  for (bitMask8=1; bitMask8>0; bitMask8 <<= 1){ // iterate through 8 bits
    if (last8 & bitMask8) digitalWrite(DDS_DATA_pin, HIGH);
    else digitalWrite(DDS_DATA_pin, LOW);

    // Toggle clock pin for DDS to receive the data after every single bit
    digitalWrite(DDS_CLOCK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(DDS_CLOCK_pin, LOW);
  }

  // Update DDS
  digitalWrite(DDS_UPDATE_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(DDS_UPDATE_pin, LOW);
}
