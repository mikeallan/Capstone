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
#define right_button_pin A3
#define left_button_pin A4
#define dds_RESET_pin 6 // dds reset pin - pin 22 on dds
#define dds_DATA_pin 10 // Serial bit sent to dds - pin 25 on dds
#define dds_W_CLK_pin 11 // Loads one bit, W_CLK - pin 7 on dds
#define dds_FQ_UD_pin 12 // Shifts register, FQ_UD - pin 8 on dds

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Variable declaration

// Need to be const for array size declaration
const unsigned int initial_starting_freq = 750000;  // Starting Frequency
const unsigned int initial_max_freq = 3000000;       // Max Frequency
const unsigned int initial_freq_step = 100000;       // Minimum Frequency
unsigned int final_freq_step = 5000;

unsigned int freq = initial_starting_freq;
const int arraySize = (initial_max_freq - initial_starting_freq) / (initial_freq_step) + 1;

unsigned int max_freq = initial_max_freq;
unsigned int starting_freq = initial_starting_freq;
unsigned int freq_step = initial_freq_step;
unsigned int min_Vz_freq;

int mode = LOW;
unsigned int myIndex;
unsigned int sweepNumber = 1;
const unsigned int numberOfSweeps = 5;

unsigned int freq_values[arraySize][numberOfSweeps + 1];
float Vz;
float Vz_values[arraySize][numberOfSweeps + 1];
float Vz_moving_avg[arraySize][numberOfSweeps + 1];
float min_Vz_value;
float data;     // Data to be sent to phone
unsigned int counter[numberOfSweeps + 1] = {0};
unsigned int sweepSize[numberOfSweeps + 1] = {0};


// ***
// *** Setup function
// ***
void setup() {
  while (!Serial);  // make sure serial monitor is up
  delay(500);

  // Set bluefruit to data mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println("Bluefruit in Data Mode");

  // Initialize bluefruit
  if ( !ble.begin(VERBOSE_MODE)){
    Serial.println("error");
  }
  Serial.println( "Bluefruit Initialized" );
  ble.verbose(false);

  Serial.begin(115200);

  // Voltage reading pins are INPUTs
  pinMode(Vz_pin, INPUT);
  pinMode(left_button_pin, INPUT_PULLUP);
  pinMode(right_button_pin, INPUT_PULLUP);

  // dds pins are OUTPUTs
  pinMode(dds_RESET_pin, OUTPUT);
  pinMode(dds_DATA_pin, OUTPUT);
  pinMode(dds_W_CLK_pin, OUTPUT);
  pinMode(dds_FQ_UD_pin, OUTPUT);

  /*// Print the start of the table
  Serial.print(" Freq ");
  Serial.print("      ");
  Serial.println("Vz");*/

  init_dds();
} // end setup()



// ***
// *** Main loop function
// ***
void loop() {
  /***** For testing *****/
  if(digitalRead(right_button_pin) == LOW) { 
    Serial.println("Right button engaged"); 
    pinMode(right_button_pin, INPUT);
    pinMode(left_button_pin, INPUT);
    mode = HIGH; 
  }
  //if(digitalRead(right_button_pin) == LOW) { sendDataToPhone(); }

  if(mode){
    Serial.print("\n\nSweep ");
    Serial.println(sweepNumber);
    for(freq = starting_freq; freq <= max_freq; freq += freq_step){
      writeddschip(freq);
      delay(50);
      readVoltages();
      storeValues();
      serialPrintTable();
    }
  
    //sweepSize[sweepNumber] = counter[sweepNumber - 1] -1;
  
    if (sweepNumber == 1){
      Serial.println("\nMoving averages");
      calculateMovingAverage();
  
      for (int i = 0; i < arraySize; i++){
        Serial.println(Vz_moving_avg[i][sweepNumber - 1], 4);
      }
    }
    
    Serial.print("\nMin freq: ");
    Serial.println(min_Vz_freq);
  
    if(sweepNumber < numberOfSweeps - 1){
      if(min_Vz_freq != 0) { 
        starting_freq = min_Vz_freq - 3*freq_step;
        max_freq = min_Vz_freq + 3*freq_step;
        freq_step = freq_step / 2;
      }
    }
  
    if(sweepNumber == numberOfSweeps - 1){
      freq_step = final_freq_step;
    }
    
    sweepNumber++;
    
    if(sweepNumber > numberOfSweeps) {
      Serial.println("Done");
      mode = LOW;
      pinMode(right_button_pin, INPUT_PULLUP);
      pinMode(left_button_pin, INPUT_PULLUP);
      sweepNumber = 1;
      for (int i = 0; i < numberOfSweeps; i++) {
        counter[i] = 0;
      }
    }
  }
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
  float numberOfSamples = 1000;
  Vz = 0;
  
  for (unsigned int i = 0; i < numberOfSamples; i++){
    Vz += analogRead(Vz_pin);
  }
  
  Vz = Vz / numberOfSamples;
} // end of readVoltages()



// ***
// *** Function to store necessary values in an array
// ***
void storeValues(){
  
  Vz_values[counter[sweepNumber - 1]][sweepNumber - 1] = Vz * 3.3 / 1023.0; // Store calculated values in an array
  freq_values[counter[sweepNumber - 1]][sweepNumber - 1] = freq;
  
  if (counter[sweepNumber - 1] == 0){
    min_Vz_value = Vz_values[counter[sweepNumber - 1]][sweepNumber - 1];
  }
  
  else if (Vz_values[counter[sweepNumber - 1]][sweepNumber - 1] < min_Vz_value) {
    min_Vz_value = Vz_values[counter[sweepNumber - 1]][sweepNumber - 1];
    min_Vz_freq = freq_values[counter[sweepNumber - 1]][sweepNumber - 1];
  }
  
  counter[sweepNumber - 1]++;
}




// ***
// *** Function to calculate the moving average of Vz
// ***
void calculateMovingAverage(){
  Vz_moving_avg[0][sweepNumber - 1] = Vz_values[0][sweepNumber - 1];
  Vz_moving_avg[1][sweepNumber - 1] = Vz_values[1][sweepNumber - 1];
  Vz_moving_avg[arraySize-1][sweepNumber - 1] = Vz_values[arraySize-1][sweepNumber - 1];
  Vz_moving_avg[arraySize-2][sweepNumber - 1] = Vz_values[arraySize-2][sweepNumber - 1];
  for (int i = 2; i < arraySize - 2; i++){
    Vz_moving_avg[i][sweepNumber - 1] = (Vz_values[i+2][sweepNumber - 1] + Vz_values[i+1][sweepNumber - 1] + 
      Vz_values[i-2][sweepNumber - 1] + Vz_values[i-1][sweepNumber - 1] + Vz_values[i][sweepNumber - 1]) / 5;
  }
} // end of calculateMovingAverage()



// ***
// *** Function to print the voltages
// ***
void serialPrintTable(){
  Serial.print(freq);
  Serial.print("   ");
  Serial.println(Vz_values[counter[sweepNumber - 1]-1][sweepNumber - 1], 4);
} // end of printTable()




// ***
// *** Function to send information to phone
// ***
void serialPrintAllSweeps(){
  /*for (int i = 0; i < numberOfSweeps; i++){
    for (int j = 0; j < sweepSize[i]; j++){
      Serial.print(freq);
      Serial.print("   ");
      Serial.println(Vz_values[counter[sweepNumber - 1]-1][sweepNumber - 1], 4);
    }
  }*/
}



// ***
// *** Function to send information to phone
// ***
void sendDataToPhone(){

  // Send graph of whole sweep
  Serial.println("Checking for connection");
  while(! ble.isConnected()){
    delay(200); //wait for a phone to be connected
  }

  Serial.println("Connected");
  
  while(1){
    ble.println(60);
  }
  
  
  
  /*
  for (int i = 0; i < arraySize; i++){
    ble.println(Vz_moving_avg[i], 4);
  }*/
  //ble.print("\n\nThe current eye pressure is: ");
  //ble.println(data);
}
