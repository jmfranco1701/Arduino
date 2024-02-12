//#define DEBUG  //Uncomment this line to enable serial monitor output to debug the code

/////////////////////////
// GENERAL DESCRIPTION //
/////////////////////////

/*  
  USB Multi Axis Controller that provides 9 analog input channels using the joystick controller.
  A separate switch is used to determine if the pedals should operate in Racing mode (clutch/brake/accelerator)
  or Flight mode (rudder/differential braking)
  Designed for Arduino Leonardo
  Requires Joystick library

  Work to go:
  Add switches for spare inputs
  See if there's a way to create differential brake axis that FS2020 recognises without conflict
  Remove thrust reverser LED (hardware function instead)
  Add trim wheel
*/

///////////////////////////////
// AXES AND PIN DESCRIPTIONS //
///////////////////////////////
/*

AXES DESCRIPTIONS [output axis name = output control]
   Racing mode
  Accel = accel
  Brake = brake
  Throttle = clutch
  
   Flight mode
  Accel = right brake (accelerator pedal)
  Brake = left brake (clutch pedal)
  Throttle = rudder (combination of accelerator and clutch pedals)

  xAxis = L Throttle
  yAxis = L Prop
  zAxis = L Mixture
  
  RxAxis = R Throttle
  RyAxis = R Prop
  RzAxis = R Mixture

PIN ASSIGNMENT
  A0 - Left Throttle
  A1 - Right Throttle 
  A2 - Left Prop 
  A3 - Right Prop
  A4 - Left Mixture
  A5 - Right Mixture
  A6 - Clutch      [Pin 4]
  A7 - Brake       [Pin 6]
  A8 - Accelerator [Pin 8]
  
  0 - [not used]
  1 - [not used]
  2 - Racing/Flight Mode
  3 - [not used]
  5 - [not used]
  7 - [not used]
  9 - Trim Up
  10 - Trim Down
  11 - Thrust Reverser LED
  12 - Input button
  13 - On board LED

*/

///////////////
// LIBRARIES //
///////////////

#include <EEPROM.h> //used to read/write cal tables

#include <Joystick.h>
// Create 2 joysticks, one for the pedals (all 3 for racing mode, or just brakes for flight mode), the other for all 6 levers plus rudder and additional switches
Joystick_ Joystick[2] = {
  Joystick_(0x03, JOYSTICK_TYPE_MULTI_AXIS, 0, 0, false, false, false, false, false, false, true, true, true, true, false), //throttle, brake, accelerator (Logitech pedals)
  Joystick_(0x04, JOYSTICK_TYPE_JOYSTICK, 5, 0,true,true,true,true,true,true,true,true,false,false,false) //6 axis, plus 1 button (Rudder, Throttle levers, plus 5 buttons)
};

/////////////////////////////////////////
// CONSTANT AND VARIABLE INITILISATION //
/////////////////////////////////////////

// CONSTANTS
const bool analogBrakes = false;      // True = using analog brake axis. False = using switches to toggle brakes on and off. If using switches, it's recommended to calibrate the pedals.
const bool initAutoSendState = true;  // Continuously sends joystick output if set to true
const int brakeSensitivity = 100;     // Minimum input required (out of 1024)on both clutch and accelerator to trigger differential braking (Flight mode only)
const int brakePeriod = 50;           // Toggle brake inputs every x milliseconds
const int brakeResolution = 10;       // Number of brake positions. Only used for modulating the left/right brake switches.
const int zeroThrotOffset = 10;       // Offset to ensure throttles to min result in 0 output (ie any value less than this will be read as 0). This may help with engaging thrust reversers.

// MISC
int inPin1 = 12;                    // Input pin 1 (used for thrust reverser toggle)
int inPin2 = 10;                    // Trim down switch
int inPin3 = 9;                     // Trim up switch
int revLed = 11;                    // Output pin for thrust reverser LED (illuminates thrust reverser switch when active)
int modeState = HIGH;               // Initialises the state (HIGH = racing pedals)
unsigned long previousMillis = 0;   // used for startup timer
unsigned long previousBMillis = 0;  // used for brake modulation
unsigned long prevDebMillis = 0;    // used for debug loop

// AXES CALIBRATION TABLE (This is the default table unless calibration mode is activated)
int minLThrottle = 0;
int maxLThrottle = 1023;
int minRThrottle = 0;
int maxRThrottle = 1023;
int minLProp = 0;
int maxLProp = 1023;
int minRProp = 0;
int maxRProp = 1023;
int minLMix = 0;
int maxLMix = 1023;
int minRMix = 0;
int maxRMix = 1023;
int minClutch = 0;
int maxClutch = 1023;
int minAccel = 0;
int maxAccel = 1023;
int minBrake = 0;
int maxBrake = 1023;
int calValid = 0;

// INPUTS
int clutch = 0;
int brake = 0;
int accel = 0;
int trimDown = 0;
int trimUp = 0;

int lThrottle = 0;
int lProp = 0;
int lMix = 0;
int rThrottle = 0;
int rProp = 0;
int rMix = 0;

// OUTPUTS
int rRud = 0;
int lRud = 0;
int rBrake = 0;
int lBrake = 0;
int rudder = 1023;
int button1State = LOW; // used for selecting thrust reversers
int lBrakeSW = LOW;     // used for modulating brake switch
int rBrakeSW = LOW;     // used for modulating brake switch
int brakeCount = 0;     // used to determine when the the brakes should be on


///////////
// SETUP //
///////////
void setup()
{
  #ifdef DEBUG
  Serial.begin(9600); // start serial port for debugging.
  while(!Serial); // wait for serial port to open.
  #endif
    
  // Setup joysticks
  Joystick[0].begin(); // racing pedals (racing mode) or differential brakes (flight mode)
  Joystick[1].begin(); // flight controls including rudder and differential braking via switches

  // Setup digital pins
  // pinMode(switchPin, INPUT_PULLUP); //Set switch as input to read desired mode [no longer used]
  pinMode(inPin1, INPUT_PULLUP);    //Set input pin 1 as an input with pull up (high means off)
  pinMode(inPin2, INPUT_PULLUP);    //Set input pin 2 as an input with pull up (high means off)
  pinMode(inPin3, INPUT_PULLUP);    //Set input pin 3 as an input with pull up (high means off)
  pinMode(LED_BUILTIN, OUTPUT);     //Set on board LED pin as output
  pinMode(revLed, OUTPUT);          //Set thrust reverse LED pin as output

  //Read mode switch (high = flight pedals)
  //modeState = digitalRead(switchPin);
  modeState = digitalRead(inPin1); //read inPin1 on startup to determine which mode for the pedals (off = flight)

  if (EEPROM.read(0) == 1) { //calibration table has been stored, therefore read it
    readCalTable();
  }
}


/////////////////
// CALIBRATION //
///////////////// 
  
void calibrate(){
  int start_state = digitalRead(inPin1); // get the mode switch position at the beginning. Terminate calibration once it's switched back
  int switch_state = digitalRead(inPin1); 

  //clear calibration table valid flag
  if (EEPROM.read(0) == 1) { //clear previous cal table in case cal fails part way through
    EEPROM.write(0,0);
  }

  //reset all calibration values. This assumes all axis are calibrated, with the exception of the brake pedal.
  minLThrottle = 511;
  maxLThrottle = 511;
  minRThrottle = 511;
  maxRThrottle = 511;
  minLProp = 511;
  maxLProp = 511;
  minRProp = 511;
  maxRProp = 511;
  minLMix = 511;
  maxLMix = 511;
  minRMix = 511;
  maxRMix = 511;
  minRMix = 511;
  maxRMix = 511;
  minClutch = 511;
  maxClutch = 511;
  minAccel = 511;
  maxAccel = 511;
  minBrake = 511;
  maxBrake = 511;

  do { // Loop through updating cal tables until the switch is moved
    // Read current values
    lThrottle = analogRead(A0);
    rThrottle = analogRead(A1);
    lProp = analogRead(A2);
    rProp = analogRead(A3);
    lMix = analogRead(A4);
    rMix = analogRead(A5);
    clutch = analogRead(A6);
    brake = analogRead(A7);
    accel = analogRead(A8);

    // Update calibration table based on readings
    minLThrottle = min(minLThrottle,lThrottle); //update the min value
    maxLThrottle = max(maxLThrottle,lThrottle); //update the max value
    minRThrottle = min(minRThrottle,rThrottle);
    maxRThrottle = max(maxRThrottle,rThrottle);
    minLProp = min(minLProp,lProp);
    maxLProp = max(maxLProp,lProp);
    minRProp = min(minRProp,rProp);
    maxRProp = max(maxRProp,rProp);
    minLMix = min(minLMix,lMix);
    maxLMix = max(maxLMix,lMix);
    minRMix = min(minRMix,rMix);
    maxRMix = max(maxRMix,rMix);
    minClutch = min(minClutch,clutch);
    maxClutch = max(maxClutch,clutch);
    minAccel = min(minAccel,accel);
    maxAccel = max(maxAccel,accel);
    minBrake = min(minBrake,brake);
    maxBrake = max(maxBrake,brake);

    // Read switch state to see if it's changed (cal complete)
    //switch_state = digitalRead(switchPin);
    switch_state = digitalRead(inPin1);
  } while (switch_state == start_state); // continue calibration until switch is moved

  calValid = 1; // calibration successful
  writeCalTable(); //calibration complete, write table

}


////////////////
// BRAKE PWM //
////////////////

void brakePWM(int lBrakeIn, int rBrakeIn) {
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousBMillis >= brakePeriod){ // Update brake values ever brakePeriod ms
    previousBMillis = currentMillis;
  
    if (map(lBrakeIn,0,1023,0,brakeResolution) > brakeCount) { // check if current brake value, remapped 0 to brake resolution (20 means full brakes), is greater than the current count
      lBrakeSW = HIGH;
    }
    else { // turn off the brake
      lBrakeSW = LOW;
    }

    if (map(rBrakeIn,0,1023,0,brakeResolution) > brakeCount) {
      rBrakeSW = HIGH;
    }
    else {
      rBrakeSW = LOW;
    }
  }
  brakeCount = (brakeCount + 1) % (brakeResolution - 1); // increment brake counter. the '-1' ensures the two top bins are for max braking, otherwise only an analog reading of 1023 would result in full brakes.

}
  
/////////////////////////////
// WRITE CALIBRATION TABLE //
/////////////////////////////

void writeCalTable(){
  int calTable[] = {calValid,minLThrottle,maxLThrottle,minRThrottle,maxRThrottle,minLProp,maxLProp,minRProp,maxRProp,minLMix,maxLMix,minRMix,maxRMix,minRMix,maxRMix,minClutch,maxClutch,minAccel,maxAccel,minBrake,maxBrake}; //create an array of calibration values. Order must match read function.
  int address = 1; //start at the first byte (save the successful cal flag at the end)

  #ifdef DEBUG
  Serial.print("Writing Cal Table. Size is: ");
  Serial.println(sizeof(calTable));
  #endif
    
  for (address = 1; address < (sizeof(calTable) / sizeof(calTable[0])); address++) { //start at the first cal value. Address 0 is the calibration valid flag.
    EEPROM.write(address * 2, calTable[address] >> 8);
    EEPROM.write(address * 2 + 1, calTable[address] & 0xFF);

    #ifdef DEBUG
    Serial.println(calTable[address]);
    Serial.println(calTable[address]>>8);
    Serial.println(calTable[address] & 0xFF);
    Serial.println();
    #endif
  }

  EEPROM.write(0,calValid); // store successful cal valid flag
  EEPROM.write(1,sizeof(calTable)); //store the size of the calibration table to help the read function. Assumes cal table will have no great than 255 entries.
}


////////////////////////////
// READ CALIBRATION TABLE //
////////////////////////////

void readCalTable(){
  int calTable[(EEPROM.read(1)/2)]; //initialise cal table. Byte 1 is the size of the array to create
  int address = 1; //start reading from the second entry
  byte byte1;
  byte byte2;

  #ifdef DEBUG
  Serial.print("Reading Cal Table. Size is: ");
  Serial.println(EEPROM.read(1));
  #endif
  
  for (address = 1; address < (sizeof(calTable) / sizeof(calTable[0])); address++) { //start at the first cal value. Address 0 is the calibration valid flag.
    calTable[address] = (EEPROM.read(address * 2) << 8) + EEPROM.read(address * 2 + 1); //store all the cal values into the calTable.

    #ifdef DEBUG
    Serial.println(calTable[address]); //debug cal value
    Serial.println(calTable[address] >> 8);
    Serial.println(calTable[address] & 0xFF);
    Serial.println();
    #endif
  }

  //assign all calibration values
  minLThrottle = calTable[1];
  maxLThrottle = calTable[2];
  minRThrottle = calTable[3];
  maxRThrottle = calTable[4];
  minLProp = calTable[5];
  maxLProp = calTable[6];
  minRProp = calTable[7];
  maxRProp = calTable[8];
  minLMix = calTable[9];
  maxLMix = calTable[10];
  minRMix = calTable[11];
  maxRMix = calTable[12];
  minRMix = calTable[13];
  maxRMix = calTable[14];
  minClutch = calTable[15];
  maxClutch = calTable[16];
  minAccel = calTable[17];
  maxAccel = calTable[18];
  minBrake = calTable[19];
  maxBrake = calTable[20];
  
}

///////////////////////////
// RACING PEDAL CONTROLS //
///////////////////////////

void racingPedals(){
    clutch = analogRead(A6);
    clutch = map(clutch,minClutch,maxClutch,0,1023); //Apply cal table and do not reverse axis
    Joystick[0].setThrottle(clutch);                
  
    brake = analogRead(A7);
    brake = map(brake,minBrake,maxBrake,0,1023); //Apply cal table and do not reverse axis
    Joystick[0].setBrake(brake);                
  
    accel = analogRead(A8);
    accel = map(accel,minAccel,maxAccel,0,1023); //Apply cal table and do not reverse axis
    Joystick[0].setAccelerator(accel);                
        
}

///////////////////////////
// FLIGHT PEDAL CONTROLS //
///////////////////////////

void flightPedals(){

  // Determine Rudder and Brake outputs  
  clutch = analogRead(A6);  
  clutch = map(clutch,minClutch,maxClutch,1023,0); //Apply cal table and reverse axis
  accel = analogRead(A8);
  accel = map(accel,minAccel,maxAccel,1023,0); //Apply cal table and reverse axis
  
  // Set Rudder based on difference between left and right pedals
  rudder = 1023 - clutch + accel;
  rudder = map(rudder,0,2047,0,1023);
  
  // Set Brakes
  if ((clutch > brakeSensitivity) && (accel > brakeSensitivity)) {
      lBrake = map(clutch,brakeSensitivity,1023,0,1023);
      rBrake = map(accel,brakeSensitivity,1023,0,1023);
      brakePWM(lBrake,rBrake);   //update brake switch controls
  }
  else { // No brakes required - reset values
      lBrake = 0;
      rBrake = 0;
      lBrakeSW = LOW;
      rBrakeSW = LOW;
      brakeCount = 0;
  }

  // Write joystick outputs

  Joystick[1].setThrottle(rudder);      // write rudder value

  if (analogBrakes) {                   // Set analog outputs if in use
    Joystick[0].setBrake(lBrake);
    Joystick[0].setAccelerator(rBrake);
  }
  else {                                // Set digital outputs if in use
    Joystick[1].setButton(1,lBrakeSW);  // Appears as switch 2
    Joystick[1].setButton(2,rBrakeSW);  // Appears as switch 3   
  }
}

////////////////////
// LEVER CONTROLS //
////////////////////

void leverControls(){
  lThrottle = analogRead(A0);
  if (lThrottle < zeroThrotOffset) {
    lThrottle = 0;
  }
//  lThrottle = constrain(lThrottle - zeroThrotOffset,0,1023);
  lThrottle = map(lThrottle,minLThrottle,maxLThrottle,0,1023); // apply calibration table
  Joystick[1].setXAxis(lThrottle);

  lProp = analogRead(A2);
  lProp = map(lProp,minLProp,maxLProp,0,1023); // apply calibration table
  Joystick[1].setYAxis(lProp);

  lMix = analogRead(A4);
  lMix = map(lMix,minLMix,maxLMix,0,1023); // apply calibration table
  Joystick[1].setZAxis(lMix);

  rThrottle = analogRead(A1);
  if (rThrottle < zeroThrotOffset) {
    rThrottle = 0;
  }
//  rThrottle = constrain(rThrottle - zeroThrotOffset,0,1023);
  rThrottle = map(rThrottle,minRThrottle,maxRThrottle,0,1023); // apply calibration table
  Joystick[1].setRxAxis(rThrottle);

  rProp = analogRead(A3);
  rProp = map(rProp,minRProp,maxRProp,0,1023); // apply calibration table
  Joystick[1].setRyAxis(rProp);

  rMix = analogRead(A5);
  rMix = map(rMix,minRMix,maxRMix,0,1023); // apply calibration table
  Joystick[1].setRzAxis(rMix);
  
}

/////////////////////
// BUTTON CONTROLS //
/////////////////////

void buttonControls(){
  button1State = !digitalRead(inPin1);    // Read thrust reverser button
  Joystick[1].setButton(0,button1State);  // Set thrust reverser state
  digitalWrite(revLed, button1State);     // Set thrust reverser LED state

  trimDown = !digitalRead(inPin2);        // Read trim down switch
  trimUp = !digitalRead(inPin3);          // Read trim up switch
  
  Joystick[1].setButton(3,trimDown);      // Set trim down state
  Joystick[1].setButton(4,trimUp);        // Set trim up state  
  
}

///////////////
// DEBUG LOOP //
///////////////
void debugLoop(){
  unsigned long currDebMillis = millis();
  
  if (currDebMillis - prevDebMillis >= 250){     //run every 250 milliseconds
    prevDebMillis = currDebMillis; // reset counter
    
    Serial.println(lThrottle); //debug cal value
    Serial.println(rThrottle); //debug cal value
    Serial.println(lProp); //debug cal value
    Serial.println(rProp); //debug cal value
    Serial.println(lMix); //debug cal value
    Serial.println(rMix); //debug cal value
    Serial.println();
  }
  
}

///////////////
// MAIN LOOP //
///////////////

void loop(){
  unsigned long currentMillis = millis(); //counter for toggling the LED at startup
  bool calComplete = false; //determine once calibration is complete
  
  // Check for a change to mode switch in the first 5 seconds, if so, calibrate
  if (millis() < 5000 && calComplete == false) {    // Check if switch mode has changed within first 5 seconds
    if (currentMillis - previousMillis >= 250){     //Blink LED to show calibration input is being checked. Toggles every 250ms.
      previousMillis = currentMillis;
      digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); //set onboard LED;
    }
    if (modeState != digitalRead(inPin1)) { // switch has moved, therefore claibration
      calibrate();        // Update cal table for each axis
      calComplete = true; // Calibration is complete
      digitalWrite(LED_BUILTIN,modeState); // Set onboard LED with mode
    }
  }

  if (modeState == LOW) {  // Operate as Racing Pedals
    racingPedals(); // Process racing pedal controls
  }
  else {  // Operate as Flight Controls
    flightPedals(); // Process flight pedal controls
  }
  leverControls(); // Process lever controls
  buttonControls(); // Process button controls

  #ifdef DEBUG
  debugLoop();  // call debug loop
  #endif
  
}
