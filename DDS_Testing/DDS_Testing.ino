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
const unsigned long max_frequency_step = 0.1; // Max Frequency in MHz
const unsigned long max_frequency = 10;       // Max Frequency in MHz
const int min_frequency = 1;                  // Minimum Frequency in MHz

unsigned long Vi;
unsigned long Vs;
unsigned long Vz;
unsigned long Vr;
unsigned long rho;      // Reflection coefficient
unsigned long SWR;      // Standing wave ratio
unsigned long Zin_Mag;  // Magnitude of Zin
unsigned long Zin_Re;   // Real component of Zin

// Convert frequencies to Hz
const unsigned int max_frequency_step = max_frequency_step * 1000000;
const unsigned int max_frequency = max_frequency * 1000000;
const unsigned int min_frequency = min_frequency * 1000000;

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

/*  // DDS Reset is active high
  digitalWrite(DDS_RESET_pin, HIGH);*/
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(DATA_pin, HIGH);
//  digitalWrite(CLOCK_pin, HIGH);
//  digitalWrite(UPDATE_pin, HIGH);
//  delay(1500);
//

  digitalWrite(DATA_pin, LOW);
  digitalWrite(CLOCK_pin, LOW);
  digitalWrite(UPDATE_pin, LOW);
  delay(1500);
}

// Function to read the four voltages
void ReadVoltages(){
  Vi=analogRead(Vi_pin);
  Vs=analogRead(Vs_pin);
  Vz=analogRead(Vz_pin);
  Vr=analogRead(Vr_pin);
}


// Function to convert the voltages to an impedence
void Calculate(){
  rho=Vr/(Vi/2);                                       //reflection coeff
  SWR=(1+abs(rho))/(1-abs(rho));                       //standing wave ratio
  Zin_Mag=50*Vz/Vs;                                    //magnitude of Zin
  Zin_Re = (Zin_Mag^2 + 50^2)*SWR/(50*(SWR^2+1));      //real componend of Zin
}

