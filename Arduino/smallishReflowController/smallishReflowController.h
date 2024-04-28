
// ***** CONSTANTS *****
// ***** GENERAL *****

//********** screen data *********
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define X_AXIS_START 18   // X-axis starting position
#define UPDATE_RATE 100   //display

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0    //eeprom address for current profile
#define EEPROM_STORAGE_ADDRESS 1  //eeprom address for everything else
#define SENSOR_SAMPLING_TIME 250  // time for thermocouple reads, DON'T CHANGE
#define PID_SAMPLE_TIME 1000      //pid loop time

//********** begin paramters saved in eeprom *****************
#define TEMPERATURE_ROOM 50  //parameter 0
#define ROOM_TEMP 0


// ***** LEAD FREE PROFILE CONSTANTS *****
#define LF_SOAK_TEMP_HOLDOFF 25  //cut heaters before reaching reflow temp
#define ROHS_SOAK_TEMP_HOLDOFF 1
#define LF_SOAK_TEMP 150            //soak set temp
#define ROHS_SOAK_TEMP 2            //^^ parameter number
#define LF_SOAK_TIMER_TEMP 140      //start the timer here
#define ROHS_SOAK_TIMER_TEMP 3      //^^ parameter number
#define LF_SOAK_TIME 60000          //set soak time
#define ROHS_SOAK_TIME 4            //^^ parameter number
#define LF_SOAK_RAMP_TEMP 0         //do we want to ramp tem pduring soak?
#define ROHS_SOAK_TEMP_RAMP 5       //^^ parameter number
#define LF_REFLOW_TEMP_HOLFOFF 20   //cut heater off before reflow temp
#define ROHS_REFLOW_TEMP_HOLDOFF 6  //^^ parameter number
#define LF__REFLOW_TEMP 250         //reflow temp
#define ROHS_REFLOW_TEMP 7          //^^ parameter number
#define LF_REFLOW_TIMER_TEMP 240    //start the timer here
#define ROHS_REFLOW_TIMER_TEMP 8    //^^ parameter number
#define LF_REFLOW_TIME 60000        //reflow time
#define ROHS_REFLOW_TIME 9          //^^ parameter number
#define LF_DOOR_OPEN_DEG 90         //how far to open the door
#define ROHS_DOOR_OPEN_DEG 10       //

// ***** LEADED PROFILE CONSTANTS *****
#define PB_SOAK_TEMP_HOLDOFF 25      //cut heater before reaching soak
#define LEAD_SOAK_TEMP_HOLDOFF 11    //^^ parameter number
#define PB_SOAK_TEMP 150             //temp to soak
#define LEAD_SOAK_TEMP 12            //^^ parameter number
#define PB_SOAK_TIMER_TEMP 140       //start the timer here
#define LEAD_SOAK_TIMER_TEMP 13      //^^ parameter number
#define PB_SOAK_TIME 60000           //time to soak
#define LEAD_SOAK_TIME 14            //^^ parameter number
#define PB_SOAK_RAMP_TEMP 0          //do we want to ramp tem pduring soak?
#define LEAD_SOAK_TEMP_RAMP 15       //^^ parameter number
#define PB_REFLOW_TEMO_HOLDOFF 25    //cut heater before reflow
#define LEAD_REFLOW_TEMP_HOLDOFF 16  //^^ parameter number
#define PB_REFLOW_TEMP 224           //reflow temp
#define LEAD_REFLOW_TEMP 17          //^^ parameter number
#define PB_REFLOW_TIMER_TEMP 240     //start the timer here
#define LEAD_REFLOW_TIMER_TEMP 18    //^^ parameter number
#define PB_REFLOW_TIME 60000         //reflow time
#define LEAD_REFLOW_TIME 19          //^^ parameter number
#define PB_DOOR_OPEN_DEG 90          //how far to open the door
#define LEAD_DOOR_OPEN_DEG 20        //^^ parameter number


// ***** PID PARAMETERS *****
// ***** PRE-HEAT *****
#define PID_KP_PREHEAT_MAIN 125    //parameter10
#define PID_P_PREHEAT 21           //^^ parameter number
#define PID_KI_PREHEAT_MAIN 0.015  //parameter11
#define PID_I_PREHEAT 22           //^^ parameter number
#define PID_KD_PREHEAT_MAIN 150    //parameter12
#define PID_D_PREHEAT 23           //^^ parameter number

// ***** SOAK *****
#define PID_KP_SOAK_MAIN 125    //parameter16
#define PID_P_SOAK 24           //^^ parameter number
#define PID_KI_SOAK_MAIN 0.015  // parameter17
#define PID_I_SOAK 25           //^^ parameter number
#define PID_KD_SOAK_MAIN 150    //parameter18
#define PID_D_SOAK 26           //^^ parameter number

// ***** REFLOW STAGE *****
#define PID_KP_REFLOW_MAIN 125    // parameter22
#define PID_P_REFLOW 27           //^^ parameter number
#define PID_KI_REFLOW_MAIN 0.015  //parameter23
#define PID_I_REFLOW 28           //^^ parameter number
#define PID_KD_REFLOW_MAIN 150    //parameter24
#define PID_D_REFLOW 29           //^^ parameter number

//others
#define MAX_ON_MAIN 100           // % limit max ontime of element 0-100 // parameter28
#define MAX_HEATER_ON 30          //^^ parameter number
#define TEMPERATURE_COOL_MIN 100  //min temp to complete cool
#define COOL_TEMP 31              //^^ parameter number
#define DOOR_CLOSED_DEG 0         // what servo angle closes door
#define DOOR_CLOSE 32             //^^ parameter number
#define DOOR_FULL_OPEN_DEG 180    //max door opening.
#define DOOR_MAX 33               //^^ parameter number

// ****** serial speed ********
#define SERIAL_SPEED 115200  //serial com speed parameter 36
#define SERIAL_BAUD 34       //^^ parameter number
//********** end paramters saved in eeprom *****************
#define EXIT 35
#define DUMP_TO_SERIAL 36
#define LOAD_DEFAULTS 37
#define SAVE_TO_EEPROM 38


//Pin assignments
#define ROTARY_PIN1 10    //D16 PC0 PCINIT16 SCA encoder
#define ROTARY_PIN2 11    //D17 PC1 PCINIT17 SCL encoder
#define BUTTON_PIN A1     //D25 PA1 PCINIT1 ADC1 encoder button
#define SSR_MAIN 12       //SSR PIN
#define LED_PIN 0         //led pin
#define BUZZER_PIN 13     //buzzer pin
#define SERVOPIN A0       //servo PWM
#define MAX6675_CS_PIN 1  //chip select max6675 thermocouple
