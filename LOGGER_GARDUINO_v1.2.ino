//--------------------------------------------------------
// LOGGER_GARDUINO v1.1 (runs only on Arduino Uno boards)
// Rotary Encoder ISR based on some code from Simon Merrett, Oleg Mazurov, Nick Gammon and Steve Spence
// v1.1: single pump, rotary encoder with sw debounce, I2C RTC, SPI SD, I2C BM280 sensor
//--------------------------------------------------------

//#define SERIAL_DEBUG   // uncomment to get debug messages on serial monitor
//#define RANDOM_VALUES  // uncomment to make test cycle without sensors (random values assigned)

//-------------------------------------------------------
//                      LIBRARIES
//-------------------------------------------------------

#include <Wire.h>                // Wire library; used by LCD
#include <SPI.h>                 // SPI library, used by SD
#include <avr/wdt.h>             // AVR watch dog timer library; used for software reset function
#include <LiquidCrystal_I2C.h>   // I2C LCD library
#include <SdFat.h>               // SdFat library (https://github.com/greiman/SdFat), smaller than std SD library
#include <RTClib.h>              // RTC library
#include <Adafruit_Sensor.h>     // Adafruit Sensor library
#include <Adafruit_BME280.h>     // BME280 temp/hum/press sensor


// ---- define the I2C LCD object ----
LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (0x27 for test, 0x20 for prod)
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

// ---- define the BM280 sensor object ----
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// ---- define the Real Time Clock object ----
RTC_DS1307 rtc;

// ---- define SD objects ----
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)
const int chipSelect = 10;          // Adafruit SD shields and modules use pin 10
SdFat SD;                           // File system object

//-------------------------------------------------------
//            GLOBAL VARIABLES AND CONSTANTS
//-------------------------------------------------------

//-----------------
// PINs definition
//-----------------
const int moistureSensorPinA = A3;        // moisture sensor A reading pin (analog A0)
const int lightSensorPin = A2;            // light sensor pin (Analog A3)
const int pinA = 2;                       // Our first pin on Rotary Encoder is digital pin 2 (used as hardware interrupt)
const int pinB = 3;                       // Our second pin on Rotary Encoder is digital pin 3 (used as hardware interrupt)
const int buttonPin = 4;                  // this is the Arduino pin we are connecting the push button to
const int moistureSensorPowerA = 6;       // moisture sensor A activation pin
const int pumpPinA = 7;                   // pump A activation pin (relais control Pin)

//-----------------
//    variables
//-----------------
// system variables
int drycounterA = 0;                      // count number of sequential "dry soil" lectures for sensor A
int pumpcountA = 0;                       // count number of pump A activation with no effect
int readcountA = 0;                       // count number of sensor A reading
int adjustfactor = 1;                     // multiply pumping time if weather is hot; default is 1, max is 3
int drythresholdoffset = 0;               // dry soil threshold offset set if humidity is low; default is 0

// sensors variables
float tempSensorValue;                    // the analog reading from the temperature sensor
float humSensorValue;                     // the analog reading from the humidity sensor
int lightSensorValue = 0;                 // the analog reading from the light sensor divider
int moistSensorValue = 0;                 // the analog reading from the moisture sensor
int darknessthreshold = 115;              // under this reading threshold it is dark: OK to check soil
const int darklightreadings = 3;          // number of consecutive dark light readings to enter the soil moisture check
int darklightcount = 0;                   // count number of dark light reading

// variables that can be changed by using Rotary Encoder
int pumpingintervalA = 10;                // number of sec to keep pump A on when dry soil is read
int moistsoilthresholdA = 99;             // moist soil threshold for sensor A (0-255); below this value pump is not activated
int drysoilthresholdA = 149;              // dry soil threshold for sensor A (0-255); above this value pump is activated
unsigned int MoistureCheckDelay = 15;     // interval between consecutive soil readings (minutes). keep as unsigned
int maxfailedpumpactivations = 7;         // max number of consecutive pump activations
int drysoilreadings = 4;                  // number of consecutive dry soil readings to activate the pump
boolean reset=0;                          // Reset option, default value is 0 (=no)
boolean adaptive=1;                       // Adapt behavior according to hum and temp, default value is 1 (=yes)

// Rotary encoder variables
byte oldEncPos = 0;                       // stores the last encoder position value so we can compare to the current reading and see if it has changed
volatile byte encoderPos = 0;             // this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte aFlag = 0;                  // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;                  // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte reading = 0;                // somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Button reading, including debounce without delay function declarations
byte oldButtonState = HIGH;               // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;    // milliseconds
unsigned long buttonPressTime;            // when the switch last changed state
boolean buttonPressed = 0;                // a flag variable for Rotary Encoder PushButton

// Menu and submenu/setting declarations
byte Mode = 0;                            // This is which menu mode we are in at any given time (top level or one of the submenus)
const byte modeMax = 8;                   // This is the number of submenus/settings you want

// Log file base name.  Must be five characters long.
char logfile[] = "log00.csv";             // Base filename
SdFile myFile;                            // File object

// main loop control variables
#define EnvCheckDelay 1000                                    // Update external environment readings every 1 sec (1000 ms)
#define FileLogDelay 900000                                   // Log values in logfile every 15 min (90000 ms)
unsigned long LastEnvCheck  = EnvCheckDelay;                  // variable to keep time of last environment update
unsigned long LastFileLog = FileLogDelay;                     // variable to keep time of last file log event
unsigned long LastMoistureCheck = MoistureCheckDelay*60000;   // variable to keep time of last soil moisture reading
boolean printLightValue = true;                               // print light histogram on LCD when checking environment sensors?

//----------------------------------
//         System messages
//----------------------------------
const char* msg_string[] =
{
  // ----------------  (16 CHARS PATTERN)
  "                ", //0
  "Initializing... ", //1
  "SD init fail!   ", //2
  "RTC not running!", //3
  "Failed file open", //4
  "Failed temp read", //5
  "dry ",             //6
  "moist  ",          //7
  "soggy  ",          //8
  "Pumping ",         //9
  "Pump stopped!   "  //10
  "BM280 init fail!"  //11

};

const char* menu_string[] =
{
  // ----------------  (16 CHARS PATTERN)
  "                ", //0
  "pumping"         , //1
  "moist"           , //2
  "dry"             , //3
  "delay"           , //4
  "maxfail"         , //5
  "dryread"         , //6
  "AdaptiveMode"    , //7
  "Reset"           , //8
};



//-------------------------------------------------------
//                   INITIALIZATION
//-------------------------------------------------------
void setup() {
    // put your setup code here, to run once:

    // serial initialization (for debug)
    #if defined(SERIAL_DEBUG)
    Serial.begin(9600);
    #endif

    // LCD initialization
    initialize_LCD();

    // BM280 sensor initialization
    //dht.begin();
    if (!bme.begin(0x76)) {
    lcd.setCursor(0, 1);
      lcd.println(msg_string[11]);
      delay(2000);
    }

    //Rotary encoder section of setup
    pinMode(pinA, INPUT_PULLUP);                                // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
    pinMode(pinB, INPUT_PULLUP);                                // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
    pinMode (buttonPin, INPUT_PULLUP);                          // set the button pin as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
    attachInterrupt(digitalPinToInterrupt(pinA), PinA, RISING); // set an interrupt on pinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
    attachInterrupt(digitalPinToInterrupt(pinB), PinB, RISING); // set an interrupt on pinB, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)

    // Welcome message
    #if defined(SERIAL_DEBUG)
    Serial.println(msg_string[1]);
    #endif
    lcd.setCursor(0, 0);
    lcd.println(msg_string[1]);

    // Arduino pins initalization
    pinMode(pumpPinA, OUTPUT);
    digitalWrite(pumpPinA, LOW);
    pinMode(moistureSensorPowerA, OUTPUT);
    digitalWrite(moistureSensorPowerA, LOW);

    // If you want to set the aref to something other than 5v
    //analogReference(EXTERNAL);

    // RTC initialization
    if (!rtc.begin()) {
        lcd.setCursor(0, 1);
        lcd.println(msg_string[3]);
        delay(2000);
    } else {
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    // SD card initialization
    // Note that even if it's not used as the CS pin, the hardware SS pin
    // (10 on most Arduino boards, 53 on the Mega) must be left as an output
    // or the SD library functions will not work.
    pinMode(SS, OUTPUT);
    if (!SD.begin(chipSelect, SPI_SPEED)) {
        lcd.setCursor(0, 1);
        lcd.println(msg_string[2]);
        delay(2000);
    } else {
        // SD logfile: create a new file
        for (uint8_t i = 0; i < 100; i++) {
           logfile[3] = i/10 + '0';
           logfile[4] = i%10 + '0';
           if (! SD.exists(logfile)) {
              // only open a new file if it doesn't exist
              break;  // leave the loop!
           }
        }
        lcd.setCursor(0, 1);
        lcd.print("log: ");
        lcd.print(logfile);
        lcd.println(msg_string[0]);
        delay(2000);
        #if defined(SERIAL_DEBUG)
        Serial.print("Logging to: ");
        Serial.println(logfile);
        #endif
    }

    // Prepare Log file:
    // Print headers on first line
    if (myFile.open(logfile, O_CREAT | O_WRITE | O_APPEND)) {

      //print headers
      myFile.println(F("Timestamp;Temperature (*C);Humidity (%);Light;Pressure (hPa);Altitude (m);Soil"));
      //myFile.println(F("Time;Temp;Hum;Light;Press;Alt;Soil"));

      // close the file:
      myFile.close();

     } else {

      // if the file didn't open, print an error:
      //lcd.clear();
      lcd.setCursor(0,1);
      lcd.println(msg_string[4]);
      delay(2000);

    }

    delay(1000);

}

//-------------------------------------------------------
//                       LOOP
//-------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:
  byte light_threshold_offset = 0;

  // always: check rotary encoder pushbutton for menu request
    checkButtonStatus();                            // check if rotary encoder button is pressed
    if (buttonPressed) {
        rotaryMenu();                               // enter the setting menu
        printLightValue=true;                       // restore printing light histogram on LCD line 1
    }

  // every EnvCheckDelay ms: check environment sensors (light, hum, temp, etc)
  if ((unsigned long)(millis() - LastEnvCheck) > EnvCheckDelay) {
    EnvSensorsCheck();
    LastEnvCheck = millis();                        // Update Last event mark
  }

  // if dark enable soil moisture check
  if (lightSensorValue <= darknessthreshold+light_threshold_offset) {
      darklightcount++;                             // dark: increment dark readings counter
      if (darklightcount >= darklightreadings) {    // enter soil measurement loop
          darklightcount = 0;                       // reset dark readings counter
          light_threshold_offset = 30;              // temporarily increment the darkness threshold

          // every MoistureCheckDelay ms: check soil moisture
          if ((unsigned long)(millis() - LastMoistureCheck) > MoistureCheckDelay*60000) {
              MoistureSensorCheck();                // read and print moisture sensor / update counters / activate pump if needed
              LastMoistureCheck = millis();         // Update Last event mark
              printLightValue=false;                // temporarily stop printing light histogram on LCD line 1
          }
        }

  } else {                                          // not dark
      darklightcount = 0;                           // reset dark readings counter
      printLightValue = true;                       // restore printing light histogram on LCD line 1
      light_threshold_offset = 0;                   // restore the standard darkness threshold
  }

  // every FileLogDelay ms: Log sensor values on file
  if ((unsigned long)(millis() - LastFileLog) > FileLogDelay) {
    log_SensorValues();                             // log values on file
    LastFileLog = millis();                         // Update Last event mark
  }

}


//-------------------------------------------------------
//                     FUNCTIONS
//-------------------------------------------------------

// ---- initialize LCD ----
void initialize_LCD() {

  byte newChar[8];

  byte code[5] =   //icons for histogram bar
  {
    B10000,
    B11000,
    B11100,
    B11110,
    B11111
  };

  byte thermometer[8] = //icon for thermometer
  {
    B00100,
    B01010,
    B01010,
    B01110,
    B01110,
    B11111,
    B11111,
    B01110
  };

  byte droplet[8] = //icon for water droplet
  {
    B00100,
    B00100,
    B01010,
    B01010,
    B10001,
    B10001,
    B10001,
    B01110,
  };

  byte sun[8] = //icon for sunlight
  {
    B00100,
    B10101,
    B01110,
    B11111,
    B01110,
    B10101,
    B00100,
    B00000,
  };

  // LCD initialization
  lcd.begin(16, 2);
  lcd.home (); // go home

  // define bar graph icons (1 to 5)
  for (int i = 1; i < 6; i++) {
    for (int j = 0; j < 8; j++)
      newChar[j] = code[i - 1];
    lcd.createChar(i, newChar);
  };
  lcd.createChar(6,thermometer);  // thermometer icon is 6
  lcd.createChar(7,droplet);      // droplet icon is 7
  lcd.createChar(8,sun);          // sunlight icon is 8
}

// ---- print histogram ----
// full scale range is 0-1024
void print_histogram(int SensorValue) {
  int blocks;
  float histogram;

  // moistSensorValue : MAXVALUE = histogram : SIZE_BAR
  histogram = (SensorValue * 11.0 * 5.0) / 1024.0;
  histogram = histogram + 0.5;

  blocks = (int)histogram / 5;

  for (int i = 0; i < blocks; i++)
    lcd.write(5);

  if ((int)(histogram) % 5 > 0)
    lcd.write((int)(histogram) % 5);
}

// ---- read light (in FULL mode), humidity and temperature (always) ----
void EnvSensorsCheck() {

  //------------ READ SENSOR VALUES ------------
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  //force random values for sensors' readings if debugging
  #if defined(RANDOM_VALUES)
  humSensorValue=(float)random(80,100);
  tempSensorValue=(float)random(-10,50);
  lightSensorValue=random(darknessthreshold);
  #else
  //humSensorValue = dht.readHumidity();
  //tempSensorValue = dht.readTemperature(false);
  humSensorValue = bme.readHumidity();
  tempSensorValue = bme.readTemperature() - 7; // 7C offset
  lightSensorValue = analogRead(lightSensorPin);
  #endif

  //------------ PRINT SENSOR READINGS ------------
  if (isnan(tempSensorValue) || isnan(humSensorValue)) {
    lcd.clear();
    lcd.print(msg_string[5]);
  } else {
    if (printLightValue) {
        lcd.clear();
        lcd.setCursor(0,0);
        //lcd.print(get_timestamp("TIME"));
        //lcd.print(':');
        lcd.write(8);             //sun symbol
        lcd.print(lightSensorValue);
        lcd.println(msg_string[0]);
        // draw light intensity bar
        lcd.setCursor(5, 0);
        print_histogram(lightSensorValue);
    }

    lcd.setCursor(0,1);
    lcd.println(msg_string[0]);
    lcd.setCursor(0,1);
    lcd.write(6);             //thermometer symbol
    lcd.print((float)tempSensorValue, 1);
    lcd.print((char)223);     //degree sign
    lcd.print("C");
    lcd.setCursor(9, 1);
    lcd.write(7);             //droplet symbol
    lcd.print((float)humSensorValue,0);
    lcd.print("%");
    //lcd.println(msg_string[0]);

    //------------ ADJUST ADAPTIVE PARAMETERS ------------
    if (adaptive) {
        // adjust soil moisture threshold for dry soil according to external humidity
        if (humSensorValue>=95)
          drythresholdoffset = 25;     // increase standard threshold by 25 if humidity >=95%
        else
          drythresholdoffset = 0;      // reset standard threshold if humidity <95%

        // adjust pumping time according to external temp
        if (tempSensorValue>=25)
          adjustfactor = 3;           // multiply pumping time by 3 if temp >=25C
        else if (tempSensorValue<=10)
          adjustfactor = 1;           // keep standard timing for pumping if temp <25C
        else
          adjustfactor = 2;           // multiply pumping time by 2 if 10<=temp<=25C
    }
  }
}  // end function EnvSensorsCheck

// ---- check moisture value and activate pump cycle ----
void MoistureSensorCheck() {

  // read moisture sensor/set random value for sensor reading if debugging
  #if defined(RANDOM_VALUES)
  moistSensorValue = random(moistsoilthresholdA-50, drysoilthresholdA+100);
  #else
  digitalWrite(moistureSensorPowerA, HIGH);  // power up moisture sensor
  delay(50);
  moistSensorValue = map(analogRead(moistureSensorPinA), 0, 1023, 0, 255);
  digitalWrite(moistureSensorPowerA, LOW);  // power down moisture sensor
  #endif
  readcountA++;

  // print messages on serial monitor
  #if defined(SERIAL_DEBUG)
  Serial.print("reading");
  Serial.print("A");
  Serial.print(" #");
  Serial.print(readcountA);
  Serial.print(": ");
  Serial.print(moistSensorValue);
  Serial.print(" - ");
  #endif

  lcd.setCursor(0, 0);
  lcd.print("A");
  lcd.print(readcountA);
  lcd.print(": ");
  lcd.print(moistSensorValue);
  lcd.print(" - ");

  // check the moisture range
  if(moistSensorValue >= (drysoilthresholdA + drythresholdoffset)){
    // in case of dry soil: system messages
    drycounterA++;
    #if defined(SERIAL_DEBUG)
    Serial.print(msg_string[6]); // print messages on serial monitor if debug mode
    #endif
    lcd.print(msg_string[6]);
    lcd.print(drycounterA);
    lcd.println("   ");
    delay(1000);
  }
  if((moistSensorValue < (drysoilthresholdA + drythresholdoffset)) && (moistSensorValue >= moistsoilthresholdA)){
    // in case of moist soil: system messages
    drycounterA = 0;
    pumpcountA = 0;
    #if defined(SERIAL_DEBUG)
    Serial.print(msg_string[7]); // print messages on serial monitor if debug mode
    #endif
    lcd.println(msg_string[7]);
    delay(1000);
  }
  if(moistSensorValue < moistsoilthresholdA){
    // in case of soggy soil: system messages
    drycounterA=0;
    pumpcountA=0;
    #if defined(SERIAL_DEBUG)
    Serial.print(msg_string[8]); // print messages on serial monitor if debug mode
    #endif
    lcd.println(msg_string[8]);
    delay(1000);
  }

  // print messages on serial monitor if debug mode
  #if defined(SERIAL_DEBUG)
  Serial.print(" (drycounterA:");
  Serial.print(drycounterA);
  Serial.print(", pumpcounter:");
  Serial.print(pumpcountA);
  Serial.println(")");
  #endif

  // if the soil is dry and if it is the right time: turn the pump on
  if (pumpcountA < maxfailedpumpactivations) {
    if (((moistSensorValue >= drysoilthresholdA + drythresholdoffset)) && (drycounterA >= drysoilreadings)){

      pumpcountA++;

      // print messages on Serial monitor
      #if defined(SERIAL_DEBUG)
      Serial.println();
      Serial.print(msg_string[9]);
      Serial.print(" - retry #");
      Serial.println(pumpcountA);
      Serial.print("Temperature is ");
      Serial.print(tempSensorValue);
      Serial.println("C");
      Serial.print("pump");
      Serial.print("A");
      Serial.print(" active for ");
      if (adaptive)
            Serial.print(pumpingintervalA * adjustfactor);
      else
            Serial.print(pumpingintervalA);
      Serial.println("s");
      Serial.println();
      #endif

      // print messages on LCD
      lcd.setCursor(0, 0);
      lcd.print(msg_string[9]);
      if (adaptive)
            lcd.print((pumpingintervalA * adjustfactor));
      else
        lcd.print(pumpingintervalA);
      lcd.print("s: r#");
      lcd.print(pumpcountA);
      lcd.println("   ");

      // activate pump 1 times for (adjustfactor*pumpingintervalA) sec
      digitalWrite(pumpPinA,HIGH);
      if (adaptive)
            delay((pumpingintervalA*adjustfactor)*1000);  // keep pumping for adjusted interval
      else
            delay(pumpingintervalA*1000);           // keep pumping for predefined interval
      digitalWrite(pumpPinA,LOW);

      // reset counters
      drycounterA = 0;
      readcountA = 0;

    }

    // turn the pump off
    digitalWrite(pumpPinA,LOW);

  } else {

    // skip pump activation - too many failed retries
    #if defined(SERIAL_DEBUG)
    Serial.println();
    Serial.println(msg_string[10]);
    Serial.print("pump ");
    Serial.print("A");
    Serial.println(" exceeded max failed activations");
    #endif
    lcd.setCursor(0, 0);
    lcd.println(msg_string[10]);

  }

}  // end of function MoistureSensorCheck

// ---- get timestamp ----
String get_timestamp(const String& mode) {
  DateTime now = rtc.now();
  String TempString = String("");
  TempString += ('"');
  if (( mode == "DATETIME") || ( mode == "DATE")) {
    TempString += String(now.year(), DEC);
    TempString += '/';
    TempString += String(now.month(), DEC);
    TempString += '/';
    TempString += String(now.day(), DEC);
  }
  if ( mode == "DATETIME") {
    TempString += ' ';
  }
  if (( mode == "DATETIME")|| (mode == "TIME")) {
    TempString += String(now.hour(), DEC);
    TempString += ':';
    TempString += String(now.minute(), DEC);
    TempString += ':';
    TempString += String(now.second(), DEC);
  }
  TempString += ('"');
  return TempString;
}

//---- write entry to log file ----
void log_SensorValues() {

  String dataString = "";   // make a string for assembling the data to log

  // if the file opened okay, write to it:
  if (myFile.open(logfile, O_CREAT | O_WRITE | O_APPEND)) {

    //print timestamp
    myFile.print(get_timestamp("DATETIME"));
    myFile.write(';');

    // read sensors and append to the string:
    dataString += String(tempSensorValue);
    dataString += ";";
    dataString += String(humSensorValue);
    dataString += ";";
    dataString += String(lightSensorValue);
    dataString += ";";
    dataString += String(bme.readPressure() / 100.0F);
    dataString += ";";
    dataString += String(bme.readAltitude(SEALEVELPRESSURE_HPA));
    dataString += ";";
    dataString += String(moistSensorValue);

    //print string values
    myFile.println(dataString);

    // close the file:
    myFile.close();

    // print log entry to serial monitor
    #if defined(SERIAL_DEBUG)
    Serial.print("logging ");
    Serial.print(get_timestamp("DATETIME"));
    Serial.print(';');
    Serial.println(value);
    #endif

  }
  else {

    // if the file didn't open, print an error:
    //lcd.clear();
    lcd.setCursor(0,1);
    lcd.println(msg_string[4]);
    delay(2000);
  }
}

// ---- This handles the bulk of the menu functions without needing to install/include/compile a menu library ----
void rotaryMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setting Mode...");
  lcd.println("                ");
  encoderPos=0;
  buttonPressed = 0;
  Mode=0;

  while (true) {

      // Rotary encoder update display if turned
      if(oldEncPos != encoderPos) { // DEBUGGING
        if (Mode == 0 && encoderPos <= modeMax) { // menu mode; encoderPos is menu item
            if (encoderPos == 0) {
                lcd.setCursor(0, 0);
                lcd.println("Exit            ");
                lcd.setCursor(0, 1);
                lcd.println(menu_string[0]);
                #if defined(SERIAL_DEBUG)
                Serial.println("Exit");
                #endif
            } else {
                lcd.setCursor(0, 0);
                lcd.print("Change ");
                lcd.print(menu_string[encoderPos]);
                lcd.println(menu_string[0]);
                lcd.setCursor(0, 1);
        #if defined(SERIAL_DEBUG)
        Serial.print("Change ");
        Serial.println(menu_string[encoderPos]);// DEBUGGING. Sometimes the serial monitor may show a value just outside modeMax due to this function. The menu shouldn't be affected.
        #endif
                switch (encoderPos) {
                  case 1:
                    lcd.print(pumpingintervalA);
                    if (adjustfactor > 0 && adaptive) {
                      lcd.print("*");
                      lcd.print(adjustfactor);
                    }
                    lcd.print(" sec");
                    break;
                  case 2:
                    lcd.print(moistsoilthresholdA);
                    break;
                  case 3:
                    lcd.print(drysoilthresholdA);
                    if (drythresholdoffset > 0 && adaptive) {
                      lcd.print("+");
                      lcd.print(drythresholdoffset);
                    }
                    break;
                  case 4:
                    lcd.print(MoistureCheckDelay);
                    lcd.print(" min");
                    break;
                  case 5:
                    lcd.print(maxfailedpumpactivations);
                    lcd.print(" attempts");
                    break;
                  case 6:
                    lcd.print(drysoilreadings);
                    lcd.print(" readings");
                    break;
                  case 7:
                    lcd.setCursor(0, 0);
                    lcd.print(menu_string[encoderPos]);
                    lcd.print("?");
                    lcd.println(menu_string[0]);
                    lcd.setCursor(0, 1);
                    if(adaptive) lcd.print("yes"); else lcd.print("no");
                    break;
                  case 8:
                    lcd.setCursor(0, 0);
                    lcd.print(menu_string[encoderPos]);
                    lcd.print("?");
                    lcd.println(menu_string[0]);
                    lcd.setCursor(0, 1);
                    lcd.print("yes/no");
                    break;
                }
                lcd.println(menu_string[0]);
            }
        } else {    // set mode; encoderPos is value
              lcd.setCursor(0, 1);
              if (Mode == 7 || Mode == 8)
                if((boolean)(encoderPos%2)) lcd.print("yes"); else lcd.print("no");
              else
                lcd.print(encoderPos);
              lcd.println(menu_string[0]);
              #if defined(SERIAL_DEBUG)
              if (Mode == 7 || Mode == 8)
                  if((boolean)(encoderPos%2)) Serial.println("yes"); else Serial.println("no");
              else
                Serial.println(encoderPos);
              #endif
        }
        oldEncPos = encoderPos;
      }
      checkButtonStatus();

      //Main menu section
      if (Mode == 0) {
        if (encoderPos > (modeMax+10)) encoderPos = modeMax; // check we haven't gone out of bounds below 0 and correct if we have
        else if (encoderPos > modeMax) encoderPos = 0; // check we haven't gone out of bounds above modeMax and correct if we have
        if (buttonPressed){
          Mode = encoderPos; // set the Mode to the current value of input if button has been pressed
          buttonPressed = 0; // reset the button status so one press results in one action
          if (Mode == 0) {
            lcd.clear();
            #if defined(SERIAL_DEBUG)
            Serial.println("Exiting..."); //DEBUGGING: print which mode has been selected
            #endif
            break;
          }

          lcd.setCursor(0, 0);
          lcd.print("set ");
          lcd.print(menu_string[Mode]);
          lcd.println(menu_string[0]);
          lcd.setCursor(0, 1);
          switch (Mode) {
            case 1:
              lcd.print(pumpingintervalA);
              encoderPos = pumpingintervalA;    // start adjusting value from last set point
              break;
            case 2:
              lcd.print(moistsoilthresholdA);
              encoderPos = moistsoilthresholdA; // start adjusting value from last set point
              break;
            case 3:
              lcd.print(drysoilthresholdA);
              encoderPos = drysoilthresholdA;   // start adjusting value from last set point
              break;
            case 4:
              lcd.print(MoistureCheckDelay);
              encoderPos = MoistureCheckDelay;    // start adjusting value from last set point
              break;
            case 5:
              lcd.print(maxfailedpumpactivations);
              encoderPos = maxfailedpumpactivations; // start adjusting value from last set point
              break;
            case 6:
              lcd.print(drysoilreadings);
              encoderPos = drysoilreadings;     // start adjusting value from last set point
              break;
            case 7:
              lcd.setCursor(0, 0);
              lcd.print(menu_string[encoderPos]);
              lcd.print("?");
              lcd.println(menu_string[0]);
              lcd.setCursor(0,1);
              if(adaptive) lcd.print("yes"); else lcd.print("no");
              encoderPos = adaptive; // start adjusting value from last set point (0=NO)
              break;
            case 8:
              lcd.setCursor(0, 0);
              lcd.print(menu_string[encoderPos]);
              lcd.print("?");
              lcd.println(menu_string[0]);
              lcd.setCursor(0,1);
              lcd.print("no");
              encoderPos = 0; // start adjusting value from last set point (0=NO)
              break;
          }
          lcd.println(menu_string[0]);
          #if defined(SERIAL_DEBUG)
          Serial.print("Mode ");    //DEBUGGING: print which mode has been selected
          Serial.println(Mode);     //DEBUGGING: print which mode has been selected
          #endif
        }
      }

      if (buttonPressed) {

          lcd.setCursor(0, 1);
          lcd.print(menu_string[Mode]);
          lcd.print("=");
      #if defined(SERIAL_DEBUG)
          Serial.print("Setting ");             //DEBUGGING
          Serial.print(menu_string[Mode]);      //DEBUGGING
          Serial.print("=");                    //DEBUGGING
      #endif

          switch (Mode) {
            case 1:
              pumpingintervalA = encoderPos; // record whatever value your encoder has been turned to, to setting 1
              setAdmin();
              lcd.print(pumpingintervalA);
              lcd.print(" sec");
              break;
            case 2:
              moistsoilthresholdA = encoderPos; // record whatever value your encoder has been turned to, to setting 2
              setAdmin();
              lcd.print(moistsoilthresholdA);
              break;
            case 3:
              drysoilthresholdA = encoderPos; // record whatever value your encoder has been turned to, to setting 3
              setAdmin();
              lcd.print(drysoilthresholdA);
              break;
            case 4:
              MoistureCheckDelay = encoderPos; // record whatever value your encoder has been turned to, to setting 4
              setAdmin();
              lcd.print(MoistureCheckDelay);
              lcd.print(" min");
              break;
            case 5:
              maxfailedpumpactivations = encoderPos; // record whatever value your encoder has been turned to, to setting 5
              setAdmin();
              lcd.print(maxfailedpumpactivations);
              lcd.print(" attmpts");
              break;
            case 6:
              drysoilreadings = encoderPos; // record whatever value your encoder has been turned to, to setting 6
              setAdmin();
              lcd.print(drysoilreadings);
              lcd.print(" reads");
              break;
            case 7:
              adaptive = (boolean)(encoderPos%2); // record whatever value your encoder has been turned to, to setting 7 (0=no, 1=yes)
              setAdmin();
              if(adaptive) lcd.print("yes"); else lcd.print("no");
              break;
            case 8:
              reset = (boolean)(encoderPos%2);  // record whatever value your encoder has been turned to, to setting 8 (0=no, 1=yes)
              if (reset) {
                  lcd.print("yes");
                  lcd.println(menu_string[0]);
                  lcd.setCursor(0, 0);
                  lcd.print(menu_string[Mode]);
                  lcd.print("?");
                  lcd.println(menu_string[0]);
                  delay(1000);
                  if(reset) softwareReset(WDTO_60MS);
              } else {
                  setAdmin();
                  lcd.print("no");
                  break;
              }
          }

          lcd.println(menu_string[0]);
          delay(1000);
          lcd.clear();
      }
    }
}  // end RotaryMenu function

// ---- Carry out common activities each time a setting is changed ----
void setAdmin() {
  #if defined(SERIAL_DEBUG)
  if (Mode == 7 || Mode == 8)
      if((boolean)(encoderPos%2)) Serial.println("yes"); else Serial.println("no");
  else
      Serial.println(encoderPos);
  Serial.println("returning to Main Menu");
  #endif

  encoderPos = 0;       // reorientate the menu index - optional as we have overflow check code elsewhere
  if (encoderPos == 0) oldEncPos=1;
  buttonPressed = 0;    // reset the button status so one press results in one action
  Mode = 0;           // go back to top level of menu, now that we've set values
}

// ---- check Button status function ----
void checkButtonStatus() {
    // Button reading with non-delay() debounce - thank you Nick Gammon!
    byte buttonState = digitalRead (buttonPin);
    if (buttonState != oldButtonState){
      if (millis () - buttonPressTime >= debounceTime){ // debounce
        buttonPressTime = millis ();  // when we closed the switch
        oldButtonState =  buttonState;  // remember for next time
        if (buttonState == LOW){
            #if defined(SERIAL_DEBUG)
            Serial.println ("Button closed"); // DEBUGGING: print that button has been closed
            #endif
            buttonPressed = 1;
          } else {
            #if defined(SERIAL_DEBUG)
            Serial.println ("Button opened"); // DEBUGGING: print that button has been opened
            #endif
            buttonPressed = 0;
        }
      }  // end if debounce time up
    } // end of state change
}  // end of check Button status function

// ---- Rotary encoder interrupt service routine for one encoder pin ----
void PinA() {
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
}

// ---- Rotary encoder interrupt service routine for the other encoder pin ----
void PinB() {
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
}

// ---- Arduino software reset function ----
void softwareReset(uint8_t prescaller) {
  // start watchdog with the provided prescaller
  wdt_enable( prescaller);
  // wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while(1) {}
}  // end of software Reset function
