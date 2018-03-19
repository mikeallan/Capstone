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

const byte DDS_RESET_pin = 6; // DDS reset pin
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

unsigned long starting_freq = 100000;  // Starting Frequency
unsigned long max_freq = 10000000;     // Max Frequency
unsigned long min_freq = 1000000;                // Minimum Frequency
unsigned long freq = starting_freq;    // Frequency

bool counter = true;

// ***
// *** Setup function
// ***
void setup() {
  while (!Serial);  // make sure serial monitor is up
  delay(500);
  
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

  // Print the start of the table
  Serial.println(F("Zin_Re, freq"));
  Serial.println(F("------------"));

  init_dds();
} // end setup()



// ***
// *** Main loop function
// ***
void loop() {

//  digitalWrite(DDS_DATA_pin, LOW);
//  digitalWrite(DDS_CLOCK_pin, LOW);
//  digitalWrite(DDS_UPDATE_pin, LOW);
  
  /***** For testing *****/
  if (counter){
    freq = 10000000;
    writeDDSchip(freq);
    readVoltages();
    calculate();
  }
  counter = false;
} // end loop()



// ***
// *** Function to say which freq to send to DDS
// ***
void incrementFreq(){
  for (unsigned long freq = starting_freq; freq < max_freq; freq += 100000){
    writeDDSchip(freq);
  }
  readVoltages();
  calculate();
  delayMicroseconds(1);
}



// ***
// *** Function to send desired frequency to DDS chip, takes frequency in Hz
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
    if (DDSLong & bitMask32) {
      digitalWrite(DDS_DATA_pin, HIGH);
      Serial.print("1");
    }
    else {
      digitalWrite(DDS_DATA_pin, LOW);
      Serial.print("0");
    }

    // Toggle clock pin for DDS to receive the data after every single bit
    digitalWrite(DDS_CLOCK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(DDS_CLOCK_pin, LOW);
  }

  Serial.println("\nLast 8\n");
  
  // Write last 8 values (constant everytime)
  for (bitMask8=1; bitMask8>0; bitMask8 <<= 1){ // iterate through 8 bits
    if (last8 & bitMask8){
      digitalWrite(DDS_DATA_pin, HIGH);
      Serial.print("1");
    }
    else{ 
      digitalWrite(DDS_DATA_pin, LOW);
      Serial.print("0");
    }

    // Toggle clock pin for DDS to receive the data after every single bit
    digitalWrite(DDS_CLOCK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(DDS_CLOCK_pin, LOW);
  }

  // Update DDS
  digitalWrite(DDS_UPDATE_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(DDS_UPDATE_pin, LOW);
} // End of writeDDSchip()



// ***
// *** Function to initialize the DDS chip
// ***
void init_dds()
{
  digitalWrite(DDS_RESET_pin, LOW);
  digitalWrite(DDS_DATA_pin, LOW);
  digitalWrite(DDS_CLOCK_pin, LOW);
  digitalWrite(DDS_UPDATE_pin, LOW);
  delay(1000);
} // End of init_dds()



// ***
// *** Function to read the four voltages
// ***
void readVoltages(){
  Vi=analogRead(Vi_pin);
  Vs=analogRead(Vs_pin);
  Vz=analogRead(Vz_pin);
  Vr=analogRead(Vr_pin);
} // end of readVoltages()



// ***
// *** Function to convert the voltages to an impedence
// ***
void calculate(){
  rho=Vr/(Vi/2);                                       //reflection coeff
  SWR=(1+abs(rho))/(1-abs(rho));                       //standing wave ratio
  Zin_Mag=50*Vz/Vs;                                    //magnitude of Zin
  Zin_Re = (Zin_Mag^2 + 50^2)*SWR/(50*(SWR^2+1));      //real componend of Zin
} // end of calculate()
