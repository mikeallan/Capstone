#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


//Define a pulsating function
#define pulseHigh(pin) {digitalWrite(pin, HIGH); delayMicroseconds(1); digitalWrite(pin, LOW); }


// Pin assignment
#define Vi_pin A1
#define Vs_pin A2
#define Vz_pin A3
#define Vr_pin A4

#define dds_RESET_pin 6 // dds reset pin - pin 22 on dds
#define dds_DATA_pin 10 // Serial bit sent to dds - pin 25 on dds
#define dds_W_CLK_pin 11 // Loads one bit, W_CLK - pin 7 on dds
#define dds_FQ_UD_pin 12 // Shifts register, FQ_UD - pin 8 on dds

// Variable declaration
unsigned long Vi;
unsigned long Vs;
unsigned long Vz;
unsigned long Vr;
unsigned long rho;      // Reflection coefficient
unsigned long SWR;      // Standing wave ratio
unsigned long Zin_Mag;  // Magnitude of Zin
unsigned long Zin_Re;   // Real component of Zin

unsigned long starting_freq = 0.1*10.e6;  // Starting Frequency
unsigned long max_freq = 10*10.e6;     // Max Frequency
unsigned long min_freq = 0.1*10.e6;                // Minimum Frequency
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
  pinMode(Vi_pin, INPUT);
  pinMode(Vs_pin, INPUT);
  pinMode(Vz_pin, INPUT);
  pinMode(Vr_pin, INPUT);

  // dds pins are OUTPUTs
  pinMode(dds_RESET_pin, OUTPUT);
  pinMode(dds_DATA_pin, OUTPUT);
  pinMode(dds_W_CLK_pin, OUTPUT);
  pinMode(dds_FQ_UD_pin, OUTPUT);

  // Print the start of the table
//  Serial.println(F("Zin_Re, freq"));
//  Serial.println(F("------------"));

  pulseHigh(dds_RESET_pin);
  pulseHigh(dds_W_CLK_pin);
  pulseHigh(dds_FQ_UD_pin); // this pulse enables serial mode - Datasheet page 12 figure 10
} // end setup()


// ***
// *** Main loop function
// ***
void loop() {

  /***** For testing *****/
  if (counter) {
    freq = 0*10.e6;
    writeddschip(freq);
    readVoltages();
    calculate();
  }
  counter = false;
} // end loop()



// ***
// *** Function to say which freq to send to dds
// ***
void incrementFreq() {
  for (unsigned long freq = starting_freq; freq < max_freq; freq += 100000) {
    writeddschip(freq);
  }
  readVoltages();
  calculate();
  delayMicroseconds(1);
}



// ***
// *** Function to send desired frequency to dds chip, takes frequency in Hz
// ***
void writeddschip(unsigned long freq) {

  // Variables
  unsigned long ddsLong;
  unsigned long bitMask32 = 0b00000000000000000000000000000001;
  byte last8 = 0b00000000;
  byte bitMask8 = 0b00000001;
  float clock_freq = 40000000; // 40 MHz

  // Calculate the first 32 bits of the 40 bit dds instruction
  ddsLong = freq * pow(2, 32) / clock_freq;

  // Write frequency values (dependent on the input)
  for (bitMask32 = 1; bitMask32 > 0; bitMask32 <<= 1) { // iterate through 32 bits of ddsLong
    if (ddsLong & bitMask32) {
      digitalWrite(dds_DATA_pin, HIGH);
      Serial.print("1");
    }
    else {
      digitalWrite(dds_DATA_pin, LOW);
      Serial.print("0");
    }

    // Toggle clock pin for dds to receive the data after every single bit
    digitalWrite(dds_W_CLK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(dds_W_CLK_pin, LOW);
  }

  Serial.println("\nLast 8\n");

  // Write last 8 values (constant everytime)
  for (bitMask8 = 1; bitMask8 > 0; bitMask8 <<= 1) { // iterate through 8 bits
    if (last8 & bitMask8) {
      digitalWrite(dds_DATA_pin, HIGH);
      Serial.print("1");
    }
    else {
      digitalWrite(dds_DATA_pin, LOW);
      Serial.print("0");
    }

    // Toggle clock pin for dds to receive the data after every single bit
    digitalWrite(dds_W_CLK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(dds_W_CLK_pin, LOW);
  }

  pulseHigh(dds_FQ_UD_pin); // Update dds
} // End of writeddschip()



// ***
// *** Function to read the four voltages
// ***
void readVoltages() {
  Vi = analogRead(Vi_pin);
  Vs = analogRead(Vs_pin);
  Vz = analogRead(Vz_pin);
  Vr = analogRead(Vr_pin);
} // end of readVoltages()



// ***
// *** Function to convert the voltages to an impedence
// ***
void calculate() {
  rho = Vr / (Vi / 2);                                 //reflection coeff
  SWR = (1 + abs(rho)) / (1 - abs(rho));               //standing wave ratio
  Zin_Mag = 50 * Vz / Vs;                              //magnitude of Zin
  Zin_Re = (Zin_Mag ^ 2 + 50 ^ 2) * SWR / (50 * (SWR ^ 2 + 1)); //real componend of Zin
} // end of calculate()
