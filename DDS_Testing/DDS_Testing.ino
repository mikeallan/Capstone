#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//Define a pulsating function
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

// Pin assignment
#define Vi_pin A1
#define Vs_pin A2
#define Vz_pin A3
#define Vr_pin A4

#define dds_RESET_pin 6 // dds reset pin - pin 22 on dds
#define dds_DATA_pin 10 // Serial bit sent to dds - pin 25 on dds
#define dds_W_CLK_pin 11 // Loads one bit, W_CLK - pin 7 on dds
#define dds_FQ_UD_pin 12 // Shifts register, FQ_UD - pin 8 on dds

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Variable declaration
const unsigned long starting_freq = 1000000;  // Starting Frequency
const unsigned long max_freq = 1200000;       // Max Frequency
const unsigned long freq_step = 10000;       // Minimum Frequency
unsigned long freq = starting_freq;     // Frequency
const int arraySize = (max_freq - starting_freq) / (freq_step);
unsigned int counter = 0;

float Vi;
float Vi_value;
float Vs;
float Vs_value;
float Vz;
float Vz_value[arraySize];
float Vr;
float Vr_value;
float rho;      // Reflection coefficient
float SWR;      // Standing wave ratio
float Zin_Mag;  // Magnitude of Zin
float Zin_Re;   // Real component of Zin
float data;     // Data to be sent to phone




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
  Serial.print("  Freq ");
  Serial.print("   ");
  Serial.print(" Vi");
  Serial.print("   ");
  Serial.print(" Vs");
  Serial.print("   ");
  Serial.print(" Vz");
  Serial.print("   ");
  Serial.print(" Vr");
  Serial.print("   ");
  Serial.println("Zin_Re");

  init_dds();
} // end setup()



// ***
// *** Main loop function
// ***
void loop() {
  /***** For testing *****/
  
  for(freq = starting_freq; freq <= max_freq; freq += freq_step){
    //writeddschip(freq);
    //delay(1000);
    
    readVoltages();
    //calculateImpedance();
    Vz_value[counter] = Vz;
    //serialPrintTable();
    printVzs();
    counter++;
  }

  /*for (int i = 0; i < arraySize; i++){
    Serial.println(Vz_value[i], 4);
  }*/

  Serial.println("moving averages");
  calculateMovingAverage();

  for (int i = 0; i < arraySize; i++){
    Serial.println(Vz_value[i], 4);
  }
  //writeddschip(1500000);
  
  while(1);
  
} // end loop()




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
  Vi = 0;
  Vs = 0;
  Vz = 0;
  Vr = 0;

  float numberOfSamples = 2000;
  
  for (unsigned int i = 0; i < numberOfSamples; i++){
    //Vi += analogRead(Vi_pin);
    //Vs += analogRead(Vs_pin);
    Vz += analogRead(Vz_pin);
    //Vr += analogRead(Vr_pin);
  }

  //Vi = Vi / numberOfSamples;
  //Vs = Vs / numberOfSamples;
  Vz = Vz / numberOfSamples;
  //Vr = Vr / numberOfSamples;

  //Vi = Vi * 3.3 / 1023.0;
  //Vs = Vs * 3.3 / 1023.0;
  Vz = Vz * 3.3 / 1023.0;
  //Vr = Vr * 3.3 / 1023.0;
  
} // end of readVoltages()




// ***
// *** Function to print the voltages
// ***
void calculateMovingAverage(){
  for (int i = 4; i < arraySize; i++){
    Vz_value[i] = (Vz_value[i-4] + Vz_value[i-3] + Vz_value[i-2] + Vz_value[i-1] + Vz_value[i]) / 5;
  }
} // end of calculateMovingAverage()





void printVzs(){
  Serial.print(freq);
  Serial.print("    ");
  Serial.print(Vz, 4);
  Serial.print("    ");
  Serial.println(Vz_value[counter], 4);
}







// ***
// *** Function to print the voltages
// ***
void serialPrintTable(){
  Serial.print(freq);
  Serial.print("   ");
  Serial.print(Vi, 4);
  Serial.print("   ");
  Serial.print(Vs, 4);
  Serial.print("   ");
  Serial.print(Vz, 4);
  Serial.print("   ");
  Serial.print(Vr, 4);
  Serial.print("   ");
  Serial.println(Zin_Re, 4);
} // end of printTable()




// ***
// *** Function to convert the voltages to an impedence
// ***
void calculateImpedance() {
  rho = Vr / (Vi / 2.0);                                          //reflection coeff
  //SWR = (abs(0.5*Vi) + abs(Vr)) / (abs((0.5*Vi) - abs(Vr))
  SWR = (0.5*Vi + Vr) / (0.5*Vi - Vr);
  //SWR = (1.0 + abs(rho)) / (1.0 - abs(rho));                        //standing wave ratio
  Zin_Mag = 50.0 * Vz / Vs;                                       //magnitude of Zin
  Zin_Re = ((pow(Zin_Mag,2.0) + pow(50.0,2.0)) * SWR) / (50.0 * (pow(SWR,2.0) + 1.0)); //real componend of Zin
} // end of calculateImpedance()




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
// *** Function to clear dds input registers - not used
// ***
void clearDDSReg() {
  for(int i=0;i<40;i++){  //writes 0 to all 40 input reg bits
    digitalWrite(dds_DATA_pin,LOW);
    pulseHigh(dds_W_CLK_pin);
  }
  pulseHigh(dds_FQ_UD_pin);
} // End of clearDDSReg()




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
