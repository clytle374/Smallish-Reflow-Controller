#define version 38

/*Title: Smallish Reflow Controller
  Date: 04-26-2024
  Author: Cory Lytle  KO4SWI

  This code was taken from Tiny Reflow Controller and adapted and modified, only 
  some parts of the structure remain, and its exellent display code.
  Thermocouple controller was changed to a MAX6657 and buttons were 
  replaced with a encoder button to enable settings configuration at the panel.
  The origonal code wouldn't run from on a 328P due to, I believe, running out of memory.  
  So it was moved to a ATmega644 processor on a MightyCore board for development. 
  So the MightyCore board needs loaded to build with the Arduino IDE.
  A servo to open the door is being added.   
  https://github.com/clytle374/Smallish-Reflow-Controller


  V31 organizing and commenting    
  V32 fixing menus, some bugs(known) remain        
  V33 fixed menus minor tweeks and the modes only moving 1 way with encoder        
  V34 clean, fix menus still, removed dead code,  
  V35 deleting extra junk, a few misc bugs
  V36 more cleanup, removed hardcoded values, work on soak ramp option
  V37 test and fix changes from above.  Need to figure out servo parameters.
  //^^ closed degrees.  Max open degrees. Open for cool ramp for both lead and ROHS?
  V38  found several issues, learned new text strings method that fixed running
  //^^ out of ram and getting trash on the display.  Fixed horriable soak slope issues
  */


//*****************************************************
//********* watchdog is running  *******
//********* charge pump the SSR supply? ***************
//*****************************************************


// ***** INCLUDES *****
#include <SPI.h>   // pulled in by SSD1306?
#include <Wire.h>  // pulled in by SSD1306SOAK_MICRO_PERIOD_PB?
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX6675.h>        //https://github.com/RobTillaart/MAX6675
#include <PID_v1.h>         //https://github.com/br3ttb/Arduino-PID-Library
#include <EncoderButton.h>  //https://github.com/Stutchbury/EncoderButton
#include <avr/wdt.h>        // watchdog timer
#include <Servo.h>
//#include <Watchdog.h> //Nadav Matalon   https://github.com/nadavmatalon/WatchDog

#include "smallishReflowController.h"

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE {  //states for switch statments
  IDLE,
  PREHEAT,
  SOAK,
  REFLOW,
  COOL,
  COMPLETE,
  TOO_HOT,
  ERROR,
  PROGRAM
} reflowState_t;

//states for the reflow modes
typedef enum CURRENT_FUNCTION {  //functions for switch statments
  INTERCEPT,
  RAMP,
  COAST,
  SLOPE,
  MAINTAIN,
  RECOVER,
} currentFunction_t;

typedef enum REFLOW_STATUS {  //reflow yes or no, why not bool?
  OFF,
  ON
} reflowStatus_t;


typedef enum REFLOW_PROFILE {  //switch for ROHS, lead, Program mode
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED,
  PROGRAM_MODE
} reflowProfile_t;


// ***** PID CONTROL VARIABLES *****
double setpoint;    //target temp
double input = 21;  //zero will throw errors before first averaged sample
double outputMain;  //output from PID for main element

//unsorted parameters for whatnots
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;    // time keeping variable
unsigned long nextRead;     //next read of thermouple
unsigned long updateLcd;    //display update interval
unsigned long systemTimer;  //timer for running everything in run modes
unsigned long soakTimer;    //timer for soaking
unsigned long reflowTimer;  //timer for relow
unsigned long buzzerPeriod;
unsigned long recoverTimer;
double lastTemp;  //temp last scan

//these are used to pass the diffrent ROHS or lead parameters to the system
unsigned long soakTempHoldoff;  //cut before we get to temp
unsigned long soakTemp;
unsigned long soakStartTemp;
unsigned long soakTempRamp;
unsigned long soakTime;
unsigned long reflowTempHoldoff;
unsigned long reflowTemp;
unsigned long reflowStartTemp;
unsigned long reflowTime;
unsigned long doorOpen;


reflowState_t reflowState;          // Reflow oven controller state machine state variable
reflowStatus_t reflowStatus;        // Reflow oven controller status
reflowProfile_t reflowProfile;      // Reflow profile type
currentFunction_t currentFunction;  //functions inside each state

unsigned int timerSeconds;  // Seconds timer for serial data out, or more?
unsigned int fault;         // Thermocouple fault status variable DOESN'T WORK
unsigned int timerUpdate;
unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
unsigned char x;
int encoder;                 //stored encoder couunts
bool button = 0;             // is button pressed?
int programPointer = 0;      //what is the current position of the programming menu
int lastPointer;             //save state to catch change
int menuPointer = 10;        // move selector back to parameters
bool edit = 1;               // sw back to edit mode
int menuYes = 0;             // IDK, look into this
int currentBaudPointer = 0;  // pointer for selecting baudrates from list, should be local?
double deltaTemp;
double px[41];  //parameters



// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {  //text for display
  "Ready",
  "Pre",
  "Soak",
  "Reflow",
  "Cool",
  "Done!",
  "Hot!",
  "Error",
  "Prog"
};

// these are the texts for the menu.
const char menuOptions[][4] = { "NO", "YES" };  //display for ON/OFF menu options
const char* parameterNames[] = { "TEMPERATURE_ROOM",
                                 "LF_SOAK_TEMP_HOLDOFF",
                                 "LF_SOAK_TEMP",
                                 "LF_SOAK_TIMER_TEMP",
                                 "LF_SOAK_TIME",
                                 "LF_SOAK_RAMP_TEMP",
                                 "LF_REFLOW_TEMP_HOLFOF",
                                 "LF_REFLOW_TEMP",
                                 "LF_REFLOW_TIMER_TEMP",
                                 "LF_REFLOW TIME",
                                 "LF_DOOR_OPEN_DEG",
                                 "PB_SOAK_TEMP_HOLDOFF",
                                 "PB_SOAK_TEMP",
                                 "PB_SOAK_TIMER_TEMP",
                                 "PB_SOAK_TIME",
                                 "PB_SOAK_RAMP_TEMP",
                                 "PB_REFLOW_TEMP_HOLDOF",
                                 "PB_REFLOW_TEMP",
                                 "PB_REFLOW_TIMER_TEMP",
                                 "PB_REFLOW_TIME",
                                 "PB_DOOR_OPEN_DEG",
                                 "PID_KP_PREHEAT_MAIN ",
                                 "PID_KI_PREHEAT_MAIN",
                                 "PID_KD_PREHEAT_MAIN",
                                 "PID_KP_SOAK_MAIN",
                                 "PID_KI_SOAK_MAIN",
                                 "PID_KD_SOAK_MAIN",
                                 "PID_KP_REFLOW_MAIN",
                                 "PID_KI_REFLOW_MAIN",
                                 "PID_KD_REFLOW_MAIN",
                                 "MAX_ON   1 to 100%",
                                 "TEMPERATURE_COOL_MIN",
                                 "DOOR_CLOSED_DEG",
                                 "DOOR_FULL_OPEN_DEG",
                                 "SERIAL_SPEED",
                                 "EXIT",
                                 "DUMP_TO_SERIAL",
                                 "LOAD_DEFAULT_SETTINGS",
                                 "SAVE_TO_EEPROM" };



const char* currentFunctionText[] = {  //names for Functions
  "INTERCEPT",
  "RAMP",
  "COAST",
  "SLOPE",
  "MAINTAIN",
  "RECOVER"
};
//this will load the defaults parameters.. menu will use to load to defaults
const double defaultpx[41] = { TEMPERATURE_ROOM,
                               LF_SOAK_TEMP_HOLDOFF,
                               LF_SOAK_TEMP,
                               LF_SOAK_TIMER_TEMP,
                               LF_SOAK_TIME,
                               LF_SOAK_RAMP_TEMP,
                               LF_REFLOW_TEMP_HOLFOFF,
                               LF__REFLOW_TEMP,
                               LF_REFLOW_TIMER_TEMP,
                               LF_REFLOW_TIME,
                               LF_DOOR_OPEN_DEG,
                               PB_SOAK_TEMP_HOLDOFF,
                               PB_SOAK_TEMP,
                               PB_SOAK_TIMER_TEMP,
                               PB_SOAK_TIME,
                               PB_SOAK_RAMP_TEMP,
                               PB_REFLOW_TEMO_HOLDOFF,
                               PB_REFLOW_TEMP,
                               PB_REFLOW_TIMER_TEMP,
                               PB_REFLOW_TIME,
                               PB_DOOR_OPEN_DEG,
                               PID_KP_PREHEAT_MAIN,
                               PID_KI_PREHEAT_MAIN,
                               PID_KD_PREHEAT_MAIN,
                               PID_KP_SOAK_MAIN,
                               PID_KI_SOAK_MAIN,
                               PID_KD_SOAK_MAIN,
                               PID_KP_REFLOW_MAIN,
                               PID_KI_REFLOW_MAIN,
                               PID_KD_REFLOW_MAIN,
                               MAX_ON_MAIN,
                               TEMPERATURE_COOL_MIN,
                               DOOR_CLOSED_DEG,
                               DOOR_FULL_OPEN_DEG,
                               SERIAL_SPEED,
                               0,
                               0,
                               0,
                               0 };



//uncomment this when skipping loading the eeprom
// *******************   BROKEN    ****************
/*double px[31] = { TEMPERATURE_ROOM, LF_SOAK_TEMP_HOLDOFF, LF_SOAK_TEMP,
                  LF_SOAK_TIME, LF_SOAK_RAMP_TEMP, LF_REFLOW_TEMP_HOLFOFF, LF__REFLOW_TEMP,
                  LF_REFLOW_TIME, PB_SOAK_TEMP_HOLDOFF, PB_SOAK_TEMP, PB_SOAK_TIME,
                  PB_SOAK_RAMP_TEMP, PB_REFLOW_TEMO_HOLDOFF, PB_REFLOW_TEMP, PB_REFLOW_TIME,
                  PID_KP_PREHEAT_MAIN, PID_KI_PREHEAT_MAIN, PID_KD_PREHEAT_MAIN, PID_KP_SOAK_MAIN,
                  PID_KI_SOAK_MAIN, PID_KD_SOAK_MAIN, PID_KP_REFLOW_MAIN, PID_KI_REFLOW_MAIN,
                  PID_KD_REFLOW_MAIN, MAX_ON_MAIN, TEMPERATURE_COOL_MIN, SERIAL_SPEED };


*/

//availble baud rates, did I miss anything important?
const double baudRates[14] = { 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600,
                               74800, 115200, 230400, 250000, 500000, 1000000 };


// PID control initalize // PID values are garbage now since I can not load eeprom yet
PID reflowOvenPIDmain(&input, &outputMain, &setpoint, 100, .025, 20, DIRECT);

//initalize oled display
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MAX6675 thermocouple interface
MAX6675 thermoCouple(MAX6675_CS_PIN, &SPI);  //sets up the thermocouple chip

EncoderButton eb1(ROTARY_PIN1, ROTARY_PIN2, BUTTON_PIN);  //sets up the encoder buton

Servo doorServo;  // create servo object to control a servo

void setup() {
  for (int i = 0; i < EXIT; i++) {  //this loop loads eeprom into parameters

    //EEPROM.put(EEPROM_STORAGE_ADDRESS + (i*4) , px[i]);   //this loads eeprom from parameters
    EEPROM.get(EEPROM_STORAGE_ADDRESS + (i * 4), px[i]);  //this loads parameters from eeprom
  }
  for (int i = EXIT; i < SAVE_TO_EEPROM; i++) {  //clear the commands parameters
    px[i] = 0;
  }


  //Serial.begin(SERIAL_SPEED); //trash
  Serial.begin(px[SERIAL_BAUD]);  //set serial speed
  // Check current selected reflow profile
  unsigned char value;  // = EEPROM.read(PROFILE_TYPE_ADDRESS);
  EEPROM.get(PROFILE_TYPE_ADDRESS, value);
  if ((value == 0) || (value == 1)) {
    // Valid reflow profile value
    reflowProfile = value;
  } else {
    // Default to lead-free profile
    EEPROM.put(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(SSR_MAIN, LOW);
  pinMode(SSR_MAIN, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  // LED pins initialization and turn on upon start-up (active high)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //Rocoder switch events
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);

  SPI.begin();
  // Initialize thermocouple interface
  thermoCouple.begin();
  thermoCouple.setSPIspeed(2500000);
  //thermocouple.setThermocoupleType(MAX6675);

  // Start-up splash
  digitalWrite(BUZZER_PIN, HIGH);  //buzz on

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {     //setup oled set to internal power and it's address
    Serial.println(F("SSD1306 allocation failed"));  //if that didn't work then
    for (;;)
      ;  // Don't proceed, loop forever
  }


  oled.display();  //start display

  digitalWrite(BUZZER_PIN, LOW);  //buzz off
  delay(1000);

  oled.clearDisplay();  //bunch of display stuff
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println(F("   Smallish Reflow"));
  oled.println(F("     Controller"));
  oled.println();
  oled.print(F("       V."));
  oled.print(version);
  oled.println();
  oled.println(F("      04-21-24"));
  oled.display();
  delay(1000);
  oled.clearDisplay();

  digitalWrite(LED_PIN, LOW);  // Turn off LED

  windowSize = 1000;  // Set window size

  nextCheck = millis();  // Initialize time keeping variable

  nextRead = millis();  // Initialize thermocouple reading variable

  updateLcd = millis();        // Initialize LCD update timer
  wdt_enable(WDTO_2S);         // Enable WDT with 1 second timeout
  doorServo.attach(SERVOPIN);  //attach door servo
  doorServo.write(0);          //zero out door servo
}

void loop() {

  unsigned long now;  // Current time
  eb1.update();       //button update
  //thermocouple read *****************************************  START
  if (millis() > nextRead) {           // Time to read thermocouple?
    wdt_reset();                       //pet the dog
    nextRead += SENSOR_SAMPLING_TIME;  // nextRead  = nextRead + SENSOR_SAMPLING_TIME
    // Read current temperature
    static int samples = 0;
    static double inputRaw[4];
    int fault = thermoCouple.read();  //figure this function out!************************
    //input = thermoCouple.getTemperature();/*
    inputRaw[samples] = thermoCouple.getTemperature();  // get the temp from the thermocouple
    samples++;
    if (samples >= 4) {
      input = (inputRaw[0] + inputRaw[1] + inputRaw[2] + inputRaw[3]) / 4;
      samples = 0;
      deltaTemp = input - lastTemp;
      lastTemp = input;
    }

    // If any thermocouple fault is detected
    if ((fault || (input < 10)))  // i think thermoCouple.read() returns a non zero value if there is a fault, TEST THIS!!!!!
    {                             // also check to see if tempature returned was  < 10C
      // Illegal operation
      reflowState = ERROR;  //error state
      reflowStatus = OFF;   //heat off
      Serial.println(F("Error"));
    }
  }
  //thermocouple read *****************************************  END

  //output data to PC, turn led on/off, and increment timerSeconds START
  if (millis() > nextCheck) {
    // Check input in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME * 4;
    // If reflow process is on going
    if (reflowStatus == ON)  //is cycle running?
    {

      digitalWrite(LED_PIN, !(digitalRead(LED_PIN)));  // Toggle LED as system heart beat

      timerSeconds++;  // Increase seconds timer for reflow curve plot
      // Send temperature and time stamp to serial
      //Serial.print(F("Time"));
      Serial.print(timerSeconds);
      Serial.print(F(" "));
      Serial.print(setpoint);
      Serial.print(F(" "));
      Serial.print(input);
      Serial.print(F(" "));
      Serial.println(outputMain / 10); /*
      Serial.print(F(" "));
      Serial.println(outputBoost);*/
    } else {
      // Turn off red LED
      digitalWrite(LED_PIN, LOW);
    }
  }
  //output data to PC, turn led on/off, and increment timerSeconds ******************* END

  // update the the OLED display ***************************** START
  if (millis() > updateLcd) {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;

    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print(lcdMessagesReflowStatus[reflowState]);
    oled.setTextSize(1);
    if (reflowStatus == ON) {
      oled.setCursor(45, 55);
      oled.print(currentFunctionText[currentFunction]);
    }
    oled.setCursor(100, 0);  //oled.setCursor(115, 0);  shifted over to get room for prog

    if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
      oled.print(F("ROHS"));
    } else if (reflowProfile == REFLOW_PROFILE_LEADED) {
      oled.print(F("LEAD"));
    } else if (reflowProfile == PROGRAM_MODE) {
      oled.print(F("PROG"));
    }
    if (reflowState != PROGRAM) {  //if we are not in program mode run preexisting program *************************
                                   // Temperature markers
      oled.setCursor(0, 18);
      oled.print(F("250"));
      oled.setCursor(0, 36);
      oled.print(F("150"));
      oled.setCursor(0, 54);
      oled.print(F("50"));
      // Draw temperature and time axis
      oled.drawLine(18, 18, 18, 63, WHITE);
      oled.drawLine(18, 63, 127, 63, WHITE);
      oled.setCursor(115, 0);

      // If currently in error state
      if (reflowState == ERROR) {
        oled.setCursor(80, 9);
        oled.print(F("TC Error"));
      } else {
        // Right align temperature reading
        if (input < 10) oled.setCursor(91, 9);
        else if (input < 100) oled.setCursor(85, 9);
        else oled.setCursor(80, 9);
        // Display current temperature
        oled.print(input);
        oled.print((char)247);
        oled.print(F("C"));
        oled.setCursor(50, 46);
        oled.print((char)0x1e);
        //oled.print(F(" "));
        if (deltaTemp >= 0) oled.print(F(" "));
        //if (deltaTemp <= 10 && deltaTemp >= -10 ) oled.print(F(" "));
        oled.print(deltaTemp);
        oled.print((char)247);
        oled.print(F("C"));
      }


      if (reflowStatus == ON) {
        // We are updating the display faster than sensor reading
        if (timerSeconds > timerUpdate) {
          // Store temperature reading every 3 s
          if ((timerSeconds % 3) == 0) {
            timerUpdate = timerSeconds;
            unsigned char averageReading = map(input, 0, 250, 63, 19);
            if (x < (SCREEN_WIDTH - X_AXIS_START)) {
              temperature[x++] = averageReading;
            }
          }
        }
      }

      unsigned char timeAxis;
      for (timeAxis = 0; timeAxis < x; timeAxis++) {
        oled.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], WHITE);
      }
    } else {  //***************if not in a normal mode we are in
      //program by default so we will used the lower part of the screen from program mode.
      double X = px[programPointer];  //the following is to
      int shiftPlaces = 8;            //align the decimal point of all parameters in display
      if (X < 1) {
        shiftPlaces = 7;
      } else {
        for (; X >= 1; shiftPlaces--) {
          X = X * .1;
        }
      }
      oled.setTextSize(1);
      oled.setCursor(0, 18);
      oled.print(parameterNames[programPointer]);  //really bad name? lol
      oled.setCursor(30, 42);
      if (programPointer < EXIT) {
        while (shiftPlaces-- > 0)           //loop the number of time
          oled.print(F(" "));               //spaces for deciaml alignemnt
        oled.print(px[programPointer], 4);  //menuPointer programPointer edit
      } else {
        oled.print(F("   "));
        oled.print(menuOptions[menuYes]);
      }

      oled.setCursor(0, 30);
      if (edit && (menuPointer == 10)) {  //handle the pointer for parameter selection
        oled.print(F("^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^"));
      } else if (!edit && (menuPointer == 10)) {
        oled.print(F("* * * * * * * * * * "));  //now in move the pointer mode
      } else {
        oled.print(F(" "));
      }
      if (menuPointer != 10) {   //are we editing parameter values?
        oled.setCursor(30, 51);  //move cursor below target data value
        for (int i = 10; i > -1; i--) {
          if (i == 2) {
            oled.print(F(" "));
          }
          if ((i - menuPointer) == 0) {
            if (edit) {
              oled.print(F("^"));  //plave edit cursor ^ under proper digit.
            } else if (!edit) {    //place select cursor * under proper digit
              oled.print(F("*"));
            }
          } else {
            oled.print(F(" "));
          }
        }
      }

      oled.setCursor(0, 54);  //show what parameter # is being edited
      oled.print(F("P#"));
      oled.print(programPointer);  //mostly for programming debug, but nice.
    }

    // Update screen
    oled.display();
  }
  // update the the OLED display ***************************** STOP


  // Reflow oven controller state machine *************  START
  if (millis() > systemTimer) {
    systemTimer = millis() + 1000;
    //deltaTemp = input - lastTemp;
    //lastTemp = input;
    switch (reflowState) {
      case IDLE:  //****************************************IDLE
        if (input >= px[ROOM_TEMP]) {
          reflowState = TOO_HOT;
          break;
        }
        doorServo.write(0);  //servo  to door close
        // If switch is pressed to start reflow process
        if (button && (reflowProfile != PROGRAM_MODE)) {
          // Send header for CSV file
          //Serial.println(F("Time,Setpoint,Input,OutputMain,OuputBoost"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;  //starting reflow cycle time?
          timerUpdate = 0;
          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++) {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;
          //#endif

          // Initialize PID control window starting time
          windowStartTime = millis();

          unsigned char value = reflowProfile;      // save used profile to eeprom
          EEPROM.put(PROFILE_TYPE_ADDRESS, value);  //upon cycle start

          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
            soakTempHoldoff = px[ROHS_SOAK_TEMP_HOLDOFF];
            soakTemp = px[ROHS_SOAK_TEMP];
            soakStartTemp = px[ROHS_SOAK_TIMER_TEMP];
            soakTime = px[ROHS_SOAK_TIME];
            soakTempRamp = px[ROHS_SOAK_TEMP_RAMP];
            reflowTempHoldoff = px[ROHS_REFLOW_TEMP_HOLDOFF];
            reflowTemp = px[ROHS_REFLOW_TEMP];
            reflowStartTemp = px[ROHS_REFLOW_TIMER_TEMP];
            reflowTime = px[ROHS_REFLOW_TIME];
            doorOpen = px[ROHS_DOOR_OPEN_DEG];
          } else {
            soakTempHoldoff = px[LEAD_SOAK_TEMP_HOLDOFF];
            soakTemp = px[LEAD_SOAK_TEMP];
            soakStartTemp = px[ROHS_SOAK_TIMER_TEMP];
            soakTime = px[LEAD_SOAK_TIME];
            soakTempRamp = px[LEAD_SOAK_TEMP_RAMP];
            reflowTempHoldoff = px[LEAD_REFLOW_TEMP_HOLDOFF];
            reflowTemp = px[LEAD_REFLOW_TEMP];
            reflowStartTemp = px[LEAD_REFLOW_TIMER_TEMP];
            reflowTime = px[LEAD_REFLOW_TIME];
            doorOpen = px[LEAD_DOOR_OPEN_DEG];
          }
          reflowOvenPIDmain.SetTunings(px[PID_P_PREHEAT], px[PID_I_PREHEAT], px[PID_D_PREHEAT]);
          // Tell the PID to range between 0 and the full window size
          reflowOvenPIDmain.SetOutputLimits(0, windowSize * .01 * px[MAX_HEATER_ON]);
          reflowOvenPIDmain.SetSampleTime(PID_SAMPLE_TIME);

          // Turn the PID on
          reflowOvenPIDmain.SetMode(AUTOMATIC);
          button = 0;
          setpoint = input + 20;
          // Proceed to preheat stage
          soakTimer = 0;    //make zero for test to avoid repeating
          reflowTimer = 0;  //make zero for test to avoid repeating
          reflowState = PREHEAT;
          currentFunction = INTERCEPT;

        } else if (button && (reflowProfile == PROGRAM_MODE)) {
          reflowState = PROGRAM;
          button = 0;
          unsigned char value;
          EEPROM.get(PROFILE_TYPE_ADDRESS, value);
          reflowProfile = value;
          //updateProg = millis();  // time base for program mode
        }
        break;

      case PREHEAT:  //****************************************PREHEAT
        reflowStatus = ON;
        switch (currentFunction) {
          case INTERCEPT:             //find and intercept the ramp
            if (deltaTemp >= 1.5) {     //at 2C/s
              setpoint = input + 10;  //start 2C/s @4C higher
              currentFunction = RAMP;
            }
            break;

          case RAMP:
            setpoint += 2;
            if (input >= soakTemp - soakTempHoldoff) {
              reflowOvenPIDmain.SetTunings(px[PID_P_SOAK], px[PID_I_SOAK], px[PID_D_SOAK]);
              reflowState = SOAK;
              currentFunction = COAST;
              setpoint = input;
            }
            break;
        }
        break;

      case SOAK:  //****************************************SOAK

        if (input >= soakStartTemp && soakTimer == 0) {  //start timer if within soak limits
          soakTimer = millis();
        }
        switch (currentFunction) {
          case COAST:
            if (input >= soakTemp) {  //if we are at soak temp
              setpoint = soakTemp;    //
              if (soakTempRamp < 1) {
                currentFunction = MAINTAIN;
              } else {
                currentFunction = SLOPE;
              }
            } else if (deltaTemp <= .4) {  //catch if delta T not high enough
              if (input + 5 > soakTemp) {
                setpoint = soakTemp;
              } else {
                currentFunction = RECOVER;  //if so add a bit of setpoint.
                recoverTimer = millis();
                setpoint = input + 5;
              }
            }
            break;

          case RECOVER:
            if (millis() > recoverTimer + 2000) {
              setpoint = input - 3;
              if (millis() > recoverTimer + 5000) {
                currentFunction = COAST;
              }
            }
            break;



          case SLOPE:
            if (millis() > soakTimer + soakTime) {
              reflowOvenPIDmain.SetTunings(px[PID_P_REFLOW], px[PID_I_REFLOW], px[PID_D_REFLOW]);
              setpoint = input + 20;
              reflowState = REFLOW;
              currentFunction = INTERCEPT;
            } else {
              double A = soakTempRamp;
              double B = soakTime;
              setpoint += (A / (B / 1000));  //increment the setpoint for the amount each second.
            }
            break;

          case MAINTAIN:
            if (millis() > soakTimer + soakTime) {
              reflowOvenPIDmain.SetTunings(px[PID_P_REFLOW], px[PID_I_REFLOW], px[PID_D_REFLOW]);
              setpoint = input + 20;
              reflowState = REFLOW;
              currentFunction = INTERCEPT;
              break;
            }
        }
        break;

      case REFLOW:  //****************************************REFLOW


        if (input >= reflowStartTemp && reflowTimer == 0) {
          reflowTimer = millis();
        }

        switch (currentFunction) {
          case INTERCEPT:
            if (deltaTemp >= 1.5) {
              setpoint = input + 10;
              currentFunction = RAMP;
            }
            break;

          case RAMP:
            setpoint += 2;
            if (input >= reflowTemp - reflowTempHoldoff) {
              currentFunction = COAST;
              setpoint = input;
            }
            break;

          case COAST:
            if (input >= reflowTemp) {
              setpoint = reflowTemp;
              currentFunction = MAINTAIN;
            } else if (deltaTemp <= .4) {
              if (input + 5 > reflowTemp) {
                setpoint = reflowTemp;
              } else {
                currentFunction = RECOVER;  //if so add a bit of setpoint.
                recoverTimer = millis();
                setpoint = input + 5;
              }
            }
            break;

          case RECOVER:
            if (millis() > recoverTimer + 2000) {
              setpoint = input - 3 ;
              if (millis() > recoverTimer + 5000) {
                currentFunction = COAST;
              }
            }
            break;

          case MAINTAIN:
            if (millis() > reflowTimer + reflowTime) {
              setpoint = px[COOL_TEMP];
              reflowState = COOL;
              break;
            }
            break;
        }
        break;

      case COOL:  //****************************************COOL
        // If minimum cool temperature is achieve
        doorServo.write(180);  //open door servo
        if (input <= px[COOL_TEMP]) {
          // Retrieve current time for buzzer usage
          buzzerPeriod = millis() + 1000;
          // Turn on buzzer to indicate completion
          digitalWrite(BUZZER_PIN, HIGH);
          // Turn off reflow process
          reflowStatus = OFF;
          // Proceed to reflow Completion state
          reflowState = COMPLETE;
        }
        break;

      case COMPLETE:  //****************************************COMPLETE
        if (millis() > buzzerPeriod) {
          // Turn off buzzer
          digitalWrite(BUZZER_PIN, LOW);
          // Reflow process ended
          reflowState = IDLE;
        }
        break;

      case TOO_HOT:  //****************************************TOO HOT
        button = 0;  //clear button press to stop restarting once cool if pressed during cool
        // If oven temperature drops below room temperature
        if (input < px[ROOM_TEMP]) {
          // Ready to reflow
          reflowState = IDLE;
        }
        break;

      case ERROR:  //disabled until understood  for safety sake ***ERROR

        Serial.println(F("Thermocouple failed, Heaters shutdown, controll locked"));
        digitalWrite(SSR_MAIN, LOW);  //shut elements off
        reflowState = IDLE;           //write error to oled
        for (;;)
          ;
        break;
      //**********************Programming mode handeling
      case PROGRAM:
        systemTimer = millis() + 250;     //speed up loop if in menu for responce
        if (edit && menuPointer == 10) {  //are we selecting the parameters
          programPointer += encoder;      //change pointer of parameter by encoder counts
          if (programPointer < 0) {       //did we hit bottom of list?
            programPointer = 38;          // if so jump to top
          }
          if (programPointer > 38) {  //did we hit top of list?
            programPointer = 0;       //if so jump to bottom
          }
        }
        if (lastPointer != programPointer && programPointer > 34 && programPointer != 34) {
          if (px[programPointer] == 0) {
            menuYes = 0;
          } else {
            menuYes = 1;
          }
        }  //else if (programPointer < SAVE_TO_EEPROM) {
        //menuSpecial = 0;
        // }

        if (lastPointer != programPointer && programPointer == SERIAL_BAUD) {
          while (px[SERIAL_BAUD] != baudRates[currentBaudPointer]) {
            currentBaudPointer++;
            if (currentBaudPointer > 14) {
              currentBaudPointer = 0;
              break;
            }
          }
        }

        lastPointer = programPointer;

        if (!edit && programPointer < SERIAL_BAUD) {  //are we in edit paramter values mode?
          menuPointer -= encoder;                     //move pointer by encoder counts
          if (menuPointer > 10) {                     //next 4 handle upwards rollover
            menuPointer = 0;
          } else if (menuPointer < 0) {
            menuPointer = 10;
          }
        } else if (!edit && programPointer > DOOR_MAX) {  //is cursor in special mode?
          if (encoder != 0 && menuPointer == 10) {        //next 4 handle downwards  rollover
            menuPointer = 7;
          } else if (encoder != 0) {
            menuPointer = 10;
          }
        }

        if (programPointer < SERIAL_BAUD && edit && menuPointer != 10) {  //Is pointer inside the parameter value?                                      //not in special functions mode
          double Z = 0;
          switch (menuPointer) {        //load Z with proper value to add or
            case 0: Z = 1.0e-3; break;  //subtract from selected place
            case 1: Z = 1.0e-2; break;  //ie +/- .001 to 1000000
            case 2: Z = 1.0e-1; break;
            case 3: Z = 1.0e0; break;
            case 4: Z = 1.0e1; break;
            case 5: Z = 1.0e2; break;
            case 6: Z = 1.0e3; break;
            case 7: Z = 1.0e4; break;
            case 8: Z = 1.0e5; break;
            case 9: Z = 1.0e6; break;
            default: Z = 0;
          }
          px[programPointer] += (encoder * Z);  // +/- encoder counts from value
          if (px[programPointer] < 0) {         //catch negative parameter value
            px[programPointer] = 0;             //if so zero it out
          }
        } else if (programPointer > DOOR_MAX && edit && menuPointer != 10) {  //are we in the
          switch (programPointer) {                                           //figure out where we are and act accordanly

            case SERIAL_BAUD:                 //serial speed toggle through real values
              currentBaudPointer -= encoder;  //move pointer by encoder counts
              if (currentBaudPointer > 13) {  //  handle rollover
                currentBaudPointer = 0;
              } else if (currentBaudPointer < 0) {
                currentBaudPointer = 13;
              }
              px[SERIAL_BAUD] = baudRates[currentBaudPointer];
              if (button) {
                button = 0;
                menuPointer = 10;  // move selector back to parameters
                edit = 1;          // sw back to edit mode
                Serial.begin(px[SERIAL_BAUD]);
              };
              break;

            case EXIT:  //exit menu
              if (encoder != 0) {
                if (menuYes == 1) {
                  menuYes = 0;
                } else {
                  menuYes = 1;
                }
              }
              if (button) {
                px[EXIT] = menuYes;
                button = 0;
                edit = 0;
              };
              break;

            case DUMP_TO_SERIAL:  //dump to serial
              if (encoder != 0) {
                if (menuYes == 1) {
                  menuYes = 0;
                } else {
                  menuYes = 1;
                }
              }
              if (button) {
                px[DUMP_TO_SERIAL] = menuYes;
                button = 0;
                menuPointer = 10;  // move selector back to parameters
                edit = 1;          // sw back to edit mode
              };
              break;

            case LOAD_DEFAULTS:  //load default parameters
              if (encoder != 0) {
                if (menuYes == 1) {
                  menuYes = 0;
                } else {
                  menuYes = 1;
                }
              }
              if (button) {
                px[LOAD_DEFAULTS] = menuYes;
                button = 0;
                menuPointer = 10;  // move selector back to parameters
                edit = 1;          // sw back to edit mode
              };
              break;

            case SAVE_TO_EEPROM:  //save to eeprom
              if (encoder != 0) {
                if (menuYes == 1) {
                  menuYes = 0;
                } else {
                  menuYes = 1;
                }
              }
              if (button) {
                px[SAVE_TO_EEPROM] = menuYes;
                button = 0;
                menuPointer = 10;  // move selector back to parameters
                edit = 1;          // sw back to edit mode
              };
              break;
          }
        }



        if (button) {  //swap between moving pointer and editing values
          edit = !edit;
        }

        button = 0;   //clear button press
        encoder = 0;  //clear encoder count


        if (px[EXIT] != 0) {  //exit edit mode
          px[EXIT] = 0;
          reflowState = IDLE;  //reset to idle
          reflowStatus = OFF;
          menuPointer = 10;  // move selector back to parameters
          edit = 0;
          lastPointer = 55;  //set to out of range to trip rereading for YES/NO
          px[EXIT] = 0;
        }

        if (px[DUMP_TO_SERIAL] != 0) {
          double x;  //this dumps eemprom to serial port
          for (int i = 0; i <= 38; i++) {
            Serial.print("Parameter[ ");
            Serial.print(i);
            Serial.print("] Name is ");
            Serial.print(parameterNames[i]);
            Serial.print(" parameter data ");
            Serial.print(px[i], 3);  //was truncating .025 to .03 fix not tested
            Serial.print(" eeprom data ");
            EEPROM.get(EEPROM_STORAGE_ADDRESS + (i * 4), x);
            Serial.println(x, 3);  //was truncating .025 to .03 fix
          }
          px[DUMP_TO_SERIAL] = 0;
          lastPointer = 55;  //set to out of range to trip rereading for YES/NO
        }

        if (px[LOAD_DEFAULTS] != 0) {     //load default parameters
          for (int i = 0; i < 39; i++) {  //clear the commands parameters
            px[i] = defaultpx[i];
          }
          Serial.begin(px[SERIAL_BAUD]);
          px[LOAD_DEFAULTS] = 0;
        }

        if (px[SAVE_TO_EEPROM] != 0) {                            //write parameters to eeprom
          for (int i = 0; i < SAVE_TO_EEPROM; i++) {              //this loop loads eeprom
            EEPROM.put(EEPROM_STORAGE_ADDRESS + (i * 4), px[i]);  //this loads eeprom from parameters
          }
          px[SAVE_TO_EEPROM] = 0;
          lastPointer = 55;  //set to out of range to trip rereading for YES/NO
        }

        //some nonsense settings cause chaos, catch them here
        if(px[ROHS_SOAK_TIMER_TEMP] > px[ROHS_SOAK_TEMP]){
          px[ROHS_SOAK_TIMER_TEMP] = px[ROHS_SOAK_TEMP];
        }

        if(px[LEAD_SOAK_TIMER_TEMP] > px[LEAD_SOAK_TEMP]){
          px[LEAD_SOAK_TIMER_TEMP] = px[LEAD_SOAK_TEMP];
        }

        if(px[ROHS_REFLOW_TIMER_TEMP] > px[ROHS_REFLOW_TEMP]){
          px[ROHS_REFLOW_TIMER_TEMP] = px[ROHS_REFLOW_TEMP];
        }

        if(px[LEAD_REFLOW_TIMER_TEMP] > px[LEAD_REFLOW_TEMP]){
          px[LEAD_REFLOW_TIMER_TEMP] = px[LEAD_REFLOW_TEMP];
        }




        break;
    }
  }  // Reflow oven controller state machine ********** END

  // PID computation and SSR control  ********************************************************* START
  if (reflowStatus == ON) {  //here is the element control

    now = millis();

    reflowOvenPIDmain.Compute();
    if ((now - windowStartTime) > windowSize)  //window if past last window?
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;  //reset window
    }
    if (outputMain > (now - windowStartTime)) {  //calc time on/off for main element
      digitalWrite(SSR_MAIN, HIGH);              // turn on main
    } else {
      digitalWrite(SSR_MAIN, LOW);  //turn off main
    }
  }
  // Reflow oven process is off, ensure oven is off
  else {
    digitalWrite(SSR_MAIN, LOW);
  }
  // PID computation and SSR control  ********************************************************* END
}


void onEb1Clicked(EncoderButton& eb) {
  // If currently reflow process is on going
  if (reflowStatus == ON) {
    // Button press is for cancelling
    // Turn off reflow process
    digitalWrite(SSR_MAIN, LOW);
    reflowStatus = OFF;
    // Reinitialize state machine
    reflowState = IDLE;
  } else {
    button = 1;
    //if not running set switchStatus to button press for start condition
  }
}
// A function to handle the 'encoder' event
void onEb1Encoder(EncoderButton& eb) {
  // Only can switch reflow profile during idle
  encoder += eb.increment();
  if (reflowState == IDLE) {                     // only if in idle
    int profile = 10 + reflowProfile + encoder;  //add 10 to remove possable neg #
    if (profile > 12) {                          //roll over
      profile -= 3;
    }
    if (profile < 10) {  //roll under
      profile += 3;
    }
    reflowProfile = profile - 10;  //set profile, less the 10
    encoder = 0;                   //clear encoder valus to fix menu jumping when entering prog mode
  }
}


/*********MOST OF THE ORIGONAL TINY REFLOW OVEN HEADER** THANKS TO THE TRUE PROGRAMMERS ** MY APPOLIGIES FOR MY MINIMIAL SKILLS******************
  Title: Tiny Reflow Controller
  Version: 2.00
  Date: 03-03-2019
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  Lead-Free Reflow Curve
  ======================

  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ==================================

  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of SSD1306 libraries. Adafruit has been the source 
  of tonnes of tutorials, examples, and libraries for everyone to learn.

  ==========================================
  Spence Konde (www.drazzy.com/e/)
  ==========================================
  Maintainer of the ATtiny core for Arduino:
  https://github.com/SpenceKonde/ATTinyCore

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this Tiny Reflow Controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library

*/
