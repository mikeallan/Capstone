#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//Define a pulsating function
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

// Pin assignment
#define Vz_pin A2

#define dds_RESET_pin 6 // dds reset pin - pin 22 on dds
#define dds_DATA_pin 10 // Serial bit sent to dds - pin 25 on dds
#define dds_W_CLK_pin 11 // Loads one bit, W_CLK - pin 7 on dds
#define dds_FQ_UD_pin 12 // Shifts register, FQ_UD - pin 8 on dds

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Variable declaration
const unsigned long starting_freq = 800000;  // Starting Frequency
const unsigned long max_freq = 2800000;       // Max Frequency
const unsigned long freq_step = 20000;       // Minimum Frequency
unsigned long freq = starting_freq;     // Frequency
const int arraySize = (max_freq - starting_freq) / (freq_step) + 1;
unsigned int counter = 0;

float Vz;
float Vz_values[arraySize];
float Vz_moving_avg[arraySize];
float data;     // Data to be sent to phone




// ***
// *** Setup function
// ***
void setup() {
  while (!Serial);  // make sure serial monitor is up
  delay(500);

  Serial.begin(115200);

  // Voltage reading pins are INPUTs
  pinMode(Vz_pin, INPUT);

  // dds pins are OUTPUTs
  pinMode(dds_RESET_pin, OUTPUT);
  pinMode(dds_DATA_pin, OUTPUT);
  pinMode(dds_W_CLK_pin, OUTPUT);
  pinMode(dds_FQ_UD_pin, OUTPUT);

  // Print the start of the table
  Serial.print(" Freq ");
  Serial.print("      ");
  Serial.println("Vz");

  init_dds();
} // end setup()



// ***
// *** Main loop function
// ***
void loop() {
  /***** For testing *****/
  
  for(freq = starting_freq; freq <= max_freq; freq += freq_step){
    if (counter == 0) {delay(5000);}
    
    writeddschip(freq);
    delay(1000);
    readVoltages();

    serialPrintTable();
  }

  Serial.println("\nMoving averages");
  calculateMovingAverage();

  for (int i = 0; i < arraySize; i++){
    Serial.println(Vz_moving_avg[i], 4);
  }
  
  
  //writeddschip(1000000);
  
  while(1);
  
} // end loop()




// ***
// *** initialize DDS chip
// ***
void init_dds(){
  pulseHigh(dds_RESET_pin);
  delay(10);
  pulseHigh(dds_W_CLK_pin);
  pulseHigh(dds_FQ_UD_pin);
} // end of init_dds()




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
    }
    else {
      digitalWrite(dds_DATA_pin, LOW);
    }

    // Toggle clock pin for dds to receive the data after every single bit
    digitalWrite(dds_W_CLK_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(dds_W_CLK_pin, LOW);
  }

  // Write last 8 values (constant everytime)
  for (bitMask8 = 1; bitMask8 > 0; bitMask8 <<= 1) { // iterate through 8 bits
    if (last8 & bitMask8) {
      digitalWrite(dds_DATA_pin, HIGH);
    }
    else {
      digitalWrite(dds_DATA_pin, LOW);
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
  float numberOfSamples = 2000;
  Vz = 0;
  
  for (unsigned int i = 0; i < numberOfSamples; i++){
    Vz += analogRead(Vz_pin);
  }
  
  Vz = Vz / numberOfSamples;
  Vz_values[counter] = Vz * 3.3 / 1023.0; // Store calculated values in an array
  
  counter++;
} // end of readVoltages()




// ***
// *** Function to calculate the moving average of Vz
// ***
void calculateMovingAverage(){
  Vz_moving_avg[0] = Vz_values[0];
  Vz_moving_avg[1] = Vz_values[1];
  Vz_moving_avg[arraySize-1] = Vz_values[arraySize-1];
  Vz_moving_avg[arraySize-2] = Vz_values[arraySize-2];
  for (int i = 2; i < arraySize - 2; i++){
    Vz_moving_avg[i] = (Vz_values[i+2] + Vz_values[i+1] + Vz_values[i-2] + Vz_values[i-1] + Vz_values[i]) / 5;
  }
} // end of calculateMovingAverage()




// ***
// *** Function to print the voltages
// ***
void serialPrintTable(){
  Serial.print(freq);
  Serial.print("   ");
  Serial.println(Vz_values[counter-1], 4);
} // end of printTable()




// ***
// *** Function to send information to phone
// ***
void sendEyePressureToPhone(){
  while(! ble.isConnected()){
    delay(200); //wait for a phone to be connected
  }
  ble.print("\n\nThe current eye pressure is: ");
  ble.println(data);
}
