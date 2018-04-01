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

#define FACTORYRESET_ENABLE         1

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// Variable declaration

// Need to be const for array size declaration
const unsigned int initial_starting_freq = 800000;  // Starting Frequency
const unsigned int initial_max_freq = 3000000;       // Max Frequency
const unsigned int initial_freq_step = 100000;       // Minimum Frequency
unsigned int final_freq_step = 20000;

unsigned int freq = initial_starting_freq;
const int arraySize = 50; //(initial_max_freq - initial_starting_freq) / (initial_freq_step) + 1;

unsigned int max_freq = initial_max_freq;
unsigned int starting_freq = initial_starting_freq;
unsigned int freq_step = initial_freq_step;
unsigned int min_Vz_freq;

int mode = LOW;
unsigned int myIndex;
unsigned int sweepNumber = 1;
const unsigned int numberOfSweeps = 4;
unsigned int buttonPressNumber = 1;
bool flag = HIGH;
bool dataCanBeSent = LOW;

unsigned int freq_values[arraySize][numberOfSweeps + 1];
float Vz;
float Vz_values[arraySize][numberOfSweeps + 1];
float Vz_moving_avg[arraySize][numberOfSweeps + 1];
float min_Vz_value;
float data;     // Data to be sent to phone
unsigned int counter[numberOfSweeps + 1] = {0};
unsigned int lastCounter[numberOfSweeps + 1] = {0};
float interpolatedArray[arraySize*100];
float slope = 0;
unsigned int sweepToPrint;
unsigned long finishTime;


// ***
// *** Setup function
// ***
void setup() {
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  // Initialise the module
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    // Perform a factory reset to make sure everything is in a known state
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Finished set up");

  ble.verbose(false);  // debug info is a little annoying after this point!


  // Send graph of whole sweep
  Serial.println("\nChecking for connection");
  while(! ble.isConnected()){
    delay(200); //wait for a phone to be connected
  }
  Serial.println("Connected");

  // Set bluefruit to data mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println("Bluefruit in Data Mode");
  

  // Voltage reading pins are INPUTs
  pinMode(Vz_pin, INPUT);
  pinMode(left_button_pin, INPUT_PULLUP);
  pinMode(right_button_pin, INPUT_PULLUP);

  // dds pins are OUTPUTs
  pinMode(dds_RESET_pin, OUTPUT);
  pinMode(dds_DATA_pin, OUTPUT);
  pinMode(dds_W_CLK_pin, OUTPUT);
  pinMode(dds_FQ_UD_pin, OUTPUT);

  init_dds(); // Initialize dds chip
  writeddschip(freq);// Send it an initial value 
} // end setup()




// ***
// *** Main loop function
// ***
void loop() {

  //userInput();
  
  if(digitalRead(right_button_pin) == LOW && flag) { 
    ble.print("\nRight button engaged. \n\nRun number "); 
    ble.println(buttonPressNumber);
    ble.print("\n");
    pinMode(right_button_pin, INPUT);
    pinMode(left_button_pin, INPUT);
    flag = LOW;
    mode = HIGH; 
    dataCanBeSent = LOW;
  }
  if(digitalRead(left_button_pin) == LOW && dataCanBeSent) { 
    sweepToPrint = 0;

    // Give the user 3 seconds to input sweep number they want to see
    finishTime = millis() + 3000;
    
    while (finishTime > millis()){
      delay(200); // for debouncing
      if(digitalRead(left_button_pin) == LOW){
        if (sweepToPrint < numberOfSweeps){
          sweepToPrint++;
        }
      }
    }
    
    Serial.print("Printing sweep ");
    Serial.println(sweepToPrint);
    addDataPoints();
    sendDataToPhone(); 
  }
  
  if(mode){
    ble.print("Sweep ");
    ble.println(sweepNumber);
    ble.print("Starting Frequency: ");
    ble.println(starting_freq);
    ble.print("Max Frequency: ");
    ble.println(max_freq);
    ble.print("Frequency step: ");
    ble.println(freq_step);
    for(freq = starting_freq; freq <= max_freq; freq += freq_step){
      writeddschip(freq);
      delay(50);
      readVoltages();
      storeValues();
      //serialPrintTable();
    }
  
    ble.print("Resonance frequency = ");
    ble.println(min_Vz_freq);
    ble.print("\n");
  
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
      //serialPrintAllSweeps();
      ble.println("\n\nDone");
      flag = HIGH;
      mode = LOW;
      pinMode(right_button_pin, INPUT_PULLUP);
      pinMode(left_button_pin, INPUT_PULLUP);
      sweepNumber = 1;
      for (int i = 0; i < numberOfSweeps; i++) {
        counter[i] = 0;
      }
      max_freq = initial_max_freq;
      starting_freq = initial_starting_freq;
      freq_step = initial_freq_step;
      buttonPressNumber++;
      dataCanBeSent = HIGH;
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

  // Store calculated values in an array
  Vz_values[counter[sweepNumber - 1]][sweepNumber - 1] = Vz * 3.3 / 1023.0; 
  freq_values[counter[sweepNumber - 1]][sweepNumber - 1] = freq;
  
  if (counter[sweepNumber - 1] == 0){
    min_Vz_value = Vz_values[counter[sweepNumber - 1]][sweepNumber - 1];
  }
  
  else if (Vz_values[counter[sweepNumber - 1]][sweepNumber - 1] < min_Vz_value) {
    min_Vz_value = Vz_values[counter[sweepNumber - 1]][sweepNumber - 1];
    min_Vz_freq = freq_values[counter[sweepNumber - 1]][sweepNumber - 1];
  }
  
  counter[sweepNumber - 1]++;
  lastCounter[sweepNumber - 1] = counter[sweepNumber - 1];
}




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
  for (int i = 0; i < numberOfSweeps; i++){    
    for (int j = 0; j < counter[i]; j++){
      Serial.print(freq_values[j][i]);
      Serial.print("   ");
      Serial.println(Vz_values[j][i], 4);
    }
  }
}




// ***
// *** Function to linearly interpolate between values
// ***
void addDataPoints(){
  
  for (int i = 0; i < lastCounter[sweepToPrint] - 1; i++){
    slope = Vz_values[i+1][sweepToPrint] - Vz_values[i][sweepToPrint];
    for (int j = 0; j < 100; j++){
      interpolatedArray[100*i+j] = Vz_values[i][sweepToPrint] +
            slope*((float)j)/(100.0);
    }
  }
}




// ***
// *** Function to send information to phone
// ***
void sendDataToPhone(){
  //for (int i = 0; i < 1/*numberOfSweeps*/; i++){ 
/*  int i = 2;   
    for (int j = 0; j < lastCounter[i] * 100; j++){
      ble.println(interpolatedArray[j], 4);
      for (int k = 0; k < 25; k++){
        ble.println(Vz_values[j][i], 4);
      }
    }*/

  for (int i = 0; i < lastCounter[sweepToPrint] - 1; i++){
    for (int j = 0; j < 100; j++){
      ble.println(interpolatedArray[100*i+j], 4);
      delayMicroseconds(10);
    }
  }
  
  delay(300);
  //ble.setMode(BLUEFRUIT_MODE_COMMAND);
} // end sendDataToPhone()



/*
/// ***
/// *** function to send & receive data - stock function
/// ***
void userInput(){
    // Check for user input
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
  ble.waitForOK();
} // end userInputWait()




/// ***
/// *** Function to get the user input
/// ***
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}*/
