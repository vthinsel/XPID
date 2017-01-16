/*
        Data Format
        -----------
        10 bits (16 bits value / 64)
        XL~a01~CXR~a02~C
        XL<Axis1>CXR<Axis2>C
        XECCC - End

	Pin out of arduino for Sabertooth
        ---------------------------------
	Pin 3 - TX data to connect to S1 on Sabertooth
	Pin A1 - input of feedback pot positioning from motor 1. (wiper,5V,GND)
	Pin A2 - input of feedback pot positioning from motor 2. (wiper,5V,GND)

        Pin 2: - Switch to enable/disable LCD. Switch open => LCD ON. Switch closed to GND => LCD Off
        Pin 12: - Emergency switch to disable DC motors between. Switch open => Motors ON . Switch closed to GND => Motors to standby value

        LCD/Keypad Shield connection:
        ---------------
          Pin A0 - Keypad input
          Pin 10 - LCD backlight control
          Pin 8 - LCD RS pin 4
          Pin 9 - LCD Enable pin 6
          Pin 4 - LCD D4 pin 11
          Pin 5 - LCD D5 pin 12
          Pin 6 - LCD D6 pin 13
          Pin 7 - LCD D7 pin 14
          * LCD R/W pin 5 to ground
          * LCD VSS pin 1 to ground
          * LCD VCC pin 2 to 5V
          * 10K pot: between +5V and ground, wiper to LCD VEE pin 3
*/
//#define DEBUG
#define ST2x32
//#include <SabertoothSimplified.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <MenuBackend.h>
#include <EEPROMex.h>
#include <EEPROMVar.h>

#define FASTADC  1 //Hack to speed up the arduino analogue read function, comment out with // to disable this hack
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define LOWBYTE(v)   ((unsigned char) (v))				//Read
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))
#define BYTELOW(v)   (*(((unsigned char *) (&v) + 1)))			//Write
#define BYTEHIGH(v)  (*((unsigned char *) (&v)))

#define   GUARD_MOTOR_1_GAIN   100.0
#define   GUARD_MOTOR_2_GAIN   100.0

int XPIDVersion = 308;
int DeadZone    = 0;
int ReadAnalog  = 0 ;

int currentanalogue1 = 0;
int currentanalogue2 = 0;
int target1 = 512;
int target2 = 512;
int buffer = 0;
int buffercount = -1;
int commandbuffer[5] = {0};

// Pot feedback inputs
int FeedbackPin1			= A1;		// select the input pin for the potentiometer 1, PC0
int FeedbackPin2			= A2;		// select the input pin for the potentiometer 2, PC1

// LCD and emergency pins definition
int EmergencyPin			= 12;            // Emergency switch (GND)
int LCDPin			       =  2 ;          // LCD toggle switch (GND)

// Calculate position
int FeedbackMax1 = 0;
int FeedbackMin1 = 0;
int FeedbackMax2 = 0;
int FeedbackMin2 = 0;

int FeedbackPotDeadZone1 = 0;		// +/- of this value will not move the motor
int FeedbackPotDeadZone2 = 0;		// +/- of this value will not move the motor

//PID variables set to 0. Will be populated with the correct value from the EEPROM or from the init EEPROM function
int motordirection1	= 0;			// motor 1 move direction 0=brake, 1=forward, 2=reverse
int motordirection2	= 0;			// motor 2 move direction 0=brake, 1=forward, 2=reverse
int oldmotordirection1	= 0;
int oldmotordirection2	= 0;

double K_motor_1       = 0;
double proportional1   = 0;
double integral1       = 0;
double derivative1     = 0;
double K_motor_2       = 0;
double proportional2   = 0;
double integral2       = 0;
double derivative2     = 0;

double integrated_motor_1_error = 0;
double integrated_motor_2_error = 0;
float last_motor_1_error	= 0;
float last_motor_2_error	= 0;

//DC Motors Value sent to Sabertooth
int OutputM1 = 0;
int OutputM2 = 0;
int Motor1STDBY = 512;
int Motor2STDBY = 1024;

int disable    = 1; //Motor stop flag
int LCDruntime = 1; //Enable LCD output
int SabertoothType = 0;

//Variables pour la liaison serie de la sabertooth
int PowerM1 = 0;              //motor 1 power [-127 127]
int PowerM2 = 0;              //motor 2 power [-127 127]

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SimEngineSerialPort Serial
#define SabertoothSerialPort Serial1 //#define SabertoothTXPinSerial Serial1 in case of arduino Mega2560
#else //arduino UNO and others
#define SimEngineSerialPort Serial
#define SabertoothSerialPort SWSerial
SoftwareSerial SWSerial(NOT_A_PIN, 3); //Pin 3 used to communicate with sabertooth module
#endif
Sabertooth ST(128, SabertoothSerialPort); // Create Sabertooth object based on guessed serial port
//SabertoothSimplified ST(SabertoothSerialPort); // Create Sabertooth object based on guessed serial port

#define SafeBLon(pin) pinMode(pin, INPUT)
#define SafeBLoff(pin) pinMode(pin, OUTPUT)
const int pin_BL = 10;

/* Optional LCD
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K pot:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 5;
int lcd_oldkey  = 5;
int adc_key_in  = 0;
#define btnRIGHT  1
#define btnUP     2
#define btnDOWN   3
#define btnLEFT   4
#define btnSELECT 5
#define btnNONE   0

//Define menu items
MenuBackend menu = MenuBackend(menuUseEvent, menuChangeEvent);
MenuItem menu_runtime = MenuItem("Showtime !");
MenuItem menu_runtimeGO = MenuItem("runtime");
MenuItem menu_motor1 = MenuItem("Motor 1");
MenuItem menu_K_motor_1 = MenuItem("M1 K");
MenuItem menu_proportional1_motor_1 = MenuItem("M1 Prop");
MenuItem menu_integral1_motor_1 = MenuItem("M1 Int");
MenuItem menu_derivative1_motor_1 = MenuItem("M1 Der");
MenuItem menu_stdby_motor_1 = MenuItem("M1 Stdby");

MenuItem menu_motor2 = MenuItem("Motor 2");
MenuItem menu_K_motor_2 = MenuItem("M2 K");
MenuItem menu_proportional1_motor_2 = MenuItem("M2 Prop");
MenuItem menu_integral1_motor_2 = MenuItem("M2 Int");
MenuItem menu_derivative1_motor_2 = MenuItem("M2 Der");
MenuItem menu_stdby_motor_2 = MenuItem("M2 Stdby");

MenuItem menu_global = MenuItem("Global Settings");
MenuItem menu_deadzone = MenuItem("Dead Zone");
MenuItem menu_readanalog = MenuItem("Read Analog");
MenuItem menu_FeedbackMax1  = MenuItem("FeedbackMax1");
MenuItem menu_FeedbackMin1  = MenuItem("FeedbackMin1");
MenuItem menu_FeedbackMax2  = MenuItem("FeedbackMax2");
MenuItem menu_FeedbackMin2  = MenuItem("FeedbackMin2");


MenuItem menu_reset = MenuItem("Reset");
MenuItem menu_reset_EEPROM = MenuItem("Reset EEPROM");

//EEPROM addresses to store variables
int addrReserved = EEPROM.getAddress(sizeof(int)); //
int addrFeedbackMax1 = EEPROM.getAddress(sizeof(FeedbackMax1));
int addrFeedbackMin1 = EEPROM.getAddress(sizeof(FeedbackMin1));
int addrFeedbackMax2 = EEPROM.getAddress(sizeof(FeedbackMax2));
int addrFeedbackMin2 = EEPROM.getAddress(sizeof(FeedbackMin2));
int addrFeedbackPotDeadZone1 = EEPROM.getAddress(sizeof(FeedbackPotDeadZone1));
int addrFeedbackPotDeadZone2 = EEPROM.getAddress(sizeof(FeedbackPotDeadZone2));
int addrK_motor_1 = EEPROM.getAddress(sizeof(K_motor_1));
int addrproportional1 = EEPROM.getAddress(sizeof(proportional1));
int addrintegral1 = EEPROM.getAddress(sizeof(integral1));
int addrderivative1 = EEPROM.getAddress(sizeof(derivative1));
int addrK_motor_2 = EEPROM.getAddress(sizeof(K_motor_2));
int addrproportional2 = EEPROM.getAddress(sizeof(proportional2));
int addrintegral2 = EEPROM.getAddress(sizeof(integral2));
int addrderivative2 = EEPROM.getAddress(sizeof(derivative2));
int addrDeadZone = EEPROM.getAddress(sizeof(DeadZone));
int addrReadAnalog = EEPROM.getAddress(sizeof(ReadAnalog));
int addrSabertoothType = EEPROM.getAddress(sizeof(SabertoothType));
int addrXPIDVersion = EEPROM.getAddress(sizeof(XPIDVersion));
int addrMotor1STDBY = EEPROM.getAddress(sizeof(Motor1STDBY));
int addrMotor2STDBY = EEPROM.getAddress(sizeof(Motor2STDBY));

void setup()
{
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
  SimEngineSerialPort.begin(115200);// for connection to SimEngine through FTDI serial USB converter on arduino board
#else //arduino UNO and others
  SimEngineSerialPort.begin(19200);// for connection to SimEngine through FTDI serial USB converter on arduino board
#endif

#ifdef ST2x32
  SabertoothSerialPort.begin(115200);
  //ST.setBaudRate(115200); //Not needed as should be done using describe software which writes speed in EEPROM of sabertooth module.
#else
  SabertoothSerialPort.begin(38400);
  ST.autobaud(SabertoothSerialPort, true); //Needed so the sabertooth 2*25 is correctly configured
#endif

  menuSetup();
  ST.motor(1, 0);
  ST.motor(2, 0);
  disable = 1;

  //Safe backlight pin mode
  //http://forum.arduino.cc//index.php?topic=96747
  digitalWrite(pin_BL, LOW);
  pinMode(pin_BL, INPUT);
  pinMode(pin_BL, OUTPUT);
  SafeBLon(pin_BL);
  pinMode(LCDPin, INPUT_PULLUP);
  pinMode(EmergencyPin, INPUT_PULLUP);
  lcd.begin(16, 2);
  //  while (!Serial) {
  ; // wait for serial port to connect. Needed for Leonardo only
  //}
  ReadEEValues(); // Init vars from EEPROM. Will be kept in case of EEPROM upgrade

#ifdef DEBUG
  PrintEEValues();
  PrintValues();
  Serial.print(F("EEPROM XPIDVERSION:")); Serial.println(EEPROM.readInt(addrXPIDVersion));
  Serial.print(F("XPIDVERSION:")); Serial.println(XPIDVersion);
#endif
  if (EEPROM.readInt(addrXPIDVersion) != XPIDVersion) { // New XPIDVERSION detected. Align EEPROM config
#ifdef DEBUG
    Serial.println(F("EEPROM empty or layout changed. Updating it"));
#endif
    lcd.setCursor(0, 0);
    lcd.print(F("Upgrade EEPROM"));
    InitEEValues();
    delay(1000);
    lcd.setCursor(0, 1);
    lcd.print("Done.");
    delay(2000);
    lcd.clear();
  } else {
#ifdef DEBUG
    Serial.println(F("EEPROM OK."));
#endif
  }
  lcd.setCursor(0, 0);
  lcd.print(F("XPID_Simtools"));
  lcd.setCursor(0, 1);
  lcd.print(F("Version "));
  lcd.setCursor(8, 1);
  lcd.print(XPIDVersion);
  delay(2000);
  lcd.clear();
#if FASTADC
  // set analogue prescale to 16
  sbi(ADCSRA, ADPS2) ;
  cbi(ADCSRA, ADPS1) ;
  cbi(ADCSRA, ADPS0) ;
#endif
}


void loop()
{
#ifdef DEBUG
  Serial.println(F("Starting navigation:\r\nUp: u   Down: d   Left: l   Right: r   Use: e"));
#endif
  menu.moveDown();
  int rate = 0;
  int rateoverhead = 0;
  int oldrateoverhead;
  //Program loop
  while (1 == 1) //Important hack: Use this own real time loop code without arduino framework delays
  {
    rate = millis();
    //rateoverhead = millis();
    if (digitalRead(EmergencyPin) == LOW)
    {
      target1 = Motor1STDBY;
      target2 = Motor2STDBY;
      FeedbackPotWorker();
      CalculatePID();
      CalculateMotorDirection();
      SetPWM();
    }
    else {
      FeedbackPotWorker();
      SerialWorker();
      CalculatePID();
      CalculateMotorDirection();
      SetPWM();
    }
    rate = millis() - rate;
    if (!digitalRead(LCDPin) == LOW)
    {
      rateoverhead = millis();
      SafeBLon(pin_BL);
      lcd_key = read_LCD_buttons();
      //if (lcd_key != lcd_oldkey)   // if keypress is detected
      //{
      //delay(50);  // wait for debounce time
      //lcd_key = read_LCD_buttons();    // convert into key press
      if (lcd_key != lcd_oldkey)
      {
        lcd_oldkey = lcd_key;
        if (lcd_key > 0) {
          LCDruntime = 0;
          //            Serial.println(lcd_key);
        }
        {
          switch (lcd_key) {
            case btnUP:
              menu.moveUp();
              break;
            case btnDOWN:
              menu.moveDown();
              break;
            case btnRIGHT: menu.moveRight();
              break;
            case btnLEFT: menu.moveLeft();
              break;
            case btnSELECT: menu.use();
              break;
          }
        }
      }
      //}
      // print the number of seconds since reset:
      //lcd.print(millis() / 1000); lcd.print(" ");
      if ((LCDruntime == 1) && (millis() % 20 == 0)) {
        lcd.setCursor(0, 0);
        lcd.print(F("L:"));
        lcd.setCursor(2, 0);
        lcd.print(target1); lcd.print("/"); lcd.print(PowerM1); lcd.print("/"); lcd.print(analogRead(FeedbackPin1)); lcd.print(F(" "));
        lcd.print(oldrateoverhead); lcd.print(F(" "));
        lcd.setCursor(0, 1);
        lcd.print(F("R:"));
        lcd.setCursor(2, 1);
        lcd.print(target2); lcd.print("/"); lcd.print(PowerM2); lcd.print("/"); lcd.print(analogRead(FeedbackPin2)); lcd.print(F(" "));
        lcd.print(rate); lcd.print(F(" "));
        rateoverhead = millis() - rateoverhead;
        oldrateoverhead = rateoverhead;
      }
    }
    else
    {
      SafeBLoff(pin_BL);
    }
  }
}

double LCDChangeValDouble(double val, double maxVal, double minVal, double stepVal) {
  LCDruntime = 0;
  lcd.setCursor(0, 1);
  lcd.print(val);
  while (true) {
    lcd_key = read_LCD_buttons();
    if (lcd_key != lcd_oldkey)   // if keypress is detected
    {
      //if (lcd_key != lcd_oldkey)
      //{
      lcd_oldkey = lcd_key;
#ifdef DEBUG
      Serial.print(F("LCD ChangeValDouble Key changed:"));
      Serial.println(lcd_key);
#endif
      switch (lcd_key) {
        case btnUP: {
            if (val + stepVal <= maxVal) {
              val += stepVal;
              Serial.print(val);
            }
            break;
          }
        case btnDOWN: {
            if (val - stepVal >= minVal) {
              val -= stepVal;
            }
          }
          break;
        case btnRIGHT: {
            return val;
          }
        case btnLEFT: {
            return val;
          }
        case btnSELECT: {
            return val;
          }
      }
      lcd.setCursor(0, 1);
      lcd.print(val); lcd.print(F("   "));
      //}
    }
  }
}

int LCDChangeValInt(int val, int maxVal, int minVal, int stepVal) {
  LCDruntime = 0;
  lcd.setCursor(0, 1);
  lcd.print(val);
  while (true) {
    lcd_key = read_LCD_buttons();
    if (lcd_key != lcd_oldkey)   // if keypress is detected
    {
      //if (lcd_key != lcd_oldkey)
      //{
      lcd_oldkey = lcd_key;
#ifdef DEBUG
      Serial.print(F("LCD ChangeValInt Key changed:"));
      Serial.println(lcd_key);
#endif
      switch (lcd_key) {
        case btnUP: {
            if (val + stepVal <= maxVal) {
              val += stepVal;
              Serial.print(val);
            }
            break;
          }
        case btnDOWN: {
            if (val - stepVal >= minVal) {
              val -= stepVal;
            }
          }
          break;
        case btnRIGHT: {
            return val;
          }
        case btnLEFT: {
            return val;
          }
        case btnSELECT: {
            return val;
          }
      }
      lcd.setCursor(0, 1);
      lcd.print(val); lcd.print(F("   "));
      //}
    }
  }
}

void InitEEValues() //Used to initialize variables and EEPROM content in case of new arduino, or updated EEPROM schema
{
  //150Â°
  /*
   FeedbackMax1			= 888;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
   FeedbackMin1			= 136;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
   FeedbackMin2			= 136;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
  */
  // 180Â°
  if (FeedbackMax1 <= 0) {
    FeedbackMax1 = 962; // Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
  }
  if (FeedbackMin1 <= 0) {
    FeedbackMin1 = 62; // Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
  }
  if (FeedbackMax2 <= 0) {
    FeedbackMax2 = 963; // Maximum position of pot 2 to scale, do not use 1023 because it cannot control outside the pot range
  }
  if (FeedbackMin2 <= 0) {
    FeedbackMin2 = 63; // Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
  }
  if (FeedbackPotDeadZone1 <= 0) {
    FeedbackPotDeadZone1 = 0; // +/- of this value will not move the motor
  }
  if (FeedbackPotDeadZone2 <= 0) {
    FeedbackPotDeadZone2 = 0; // +/- of this value will not move the motor
  }
  if (K_motor_1	<= 0) {
    K_motor_1	= 1;
  }
  if (proportional1 <= 0) {
    proportional1 = 5.000;
  }
  if (integral1	<= 0) {
    integral1 = 0.200;
  }
  if (derivative1 <= 0) {
    derivative1	= 0.200;
  }
  if (K_motor_2	<= 0) {
    K_motor_2 = 1;
  }
  if (proportional2 <= 0) {
    proportional2 = 5.000;
  }
  if (integral2	<= 0) {
    integral2 = 0.200;
  }
  if (derivative2 <= 0) {
    derivative2 = 0.200;
  }
  if (DeadZone <= 0) {
    DeadZone = 4 ; //increase this value to reduce vibrations of motors
  }
  if (ReadAnalog <= 0) {
    ReadAnalog = 8 ;
  }
  if (Motor1STDBY <= 0) {
    Motor1STDBY = 512 ;
  }
  if (Motor2STDBY <= 0) {
    Motor2STDBY = 512 ;
  }
  if (SabertoothType <= 0) {
    addrSabertoothType = 1;// sabertooth 2*25 =>1  sabertooth 2*32 =>2
  }
  WriteEEValues();
}

int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  delay(10); //switch debounce delay. Increase this delay if incorrect switch selections are returned.
  int k = (analogRead(0) - adc_key_in); //gives the button a slight range to allow for a little contact resistance noise
  if (5 < abs(k)) return btnNONE;  // double checks the keypress. If the two readings are not equal +/-k value after debounce delay, it tries again. if (adc_key_in > 770)  return btnNONE;  // This is the 1st option for speed reasons since it will be the most likely result.
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 250)  return btnUP;
  if (adc_key_in < 450)  return btnDOWN;
  if (adc_key_in < 650)  return btnLEFT;
  if (adc_key_in < 850)  return btnSELECT;

  // For V1.0 comment the other threshold and use the one below:
  /*
   if (adc_key_in < 50)   return btnRIGHT;
   if (adc_key_in < 195)  return btnUP;
   if (adc_key_in < 380)  return btnDOWN;
   if (adc_key_in < 555)  return btnLEFT;
   if (adc_key_in < 790)  return btnSELECT;
  */
  return btnNONE;  // when all others fail, return this...
}

void FeedbackPotWorker()
{
  currentanalogue1 = 0;
  currentanalogue2 = 0;
  for (int z = 0; z < ReadAnalog; z++)
  {
    currentanalogue1 += analogRead(FeedbackPin1);
    currentanalogue2 += analogRead(FeedbackPin2);
  }
  currentanalogue1 /= ReadAnalog;
  currentanalogue2 /= ReadAnalog;
}

void SerialWorker()
{
  while (Serial.available())
  {
    if (buffercount == -1)
    {
      buffer = Serial.read();
      if (buffer != 'X') {
        buffercount = -1;
      } else {
        buffercount = 0;
      }
    }
    else
    {
      buffer = Serial.read();
      commandbuffer[buffercount] = buffer;
      buffercount++;
      if (buffercount > 3)
      {
        if (commandbuffer[3] == 'C') {
          ParseCommand();
        }
        buffercount = -1;
      }
    }
  }
}

void ParseCommand()
{
  if (commandbuffer[0] == 'L')			//Set motor 1 position to High and Low value 0 to 1023
  {
    target1 = (commandbuffer[1] * 256) + commandbuffer[2];
    target1 = map(target1, 0, 1023, FeedbackMin1, FeedbackMax1);
    disable = 0;
    return;
  }
  if (commandbuffer[0] == 'R')			//Set motor 2 position to High and Low value 0 to 1023
  {
    target2 = (commandbuffer[1] * 256) + commandbuffer[2];
    target2 = map(target2, 0, 1023, FeedbackMin2, FeedbackMax2);
    disable = 0;
    return;
  }
  if (commandbuffer[0] == 'E')		//Disable power on both motor
  {
    unsigned long start;
    unsigned long time;
    start = millis();

    target1 = 512;
    target2 = 512;

    time = millis();

    while (time < (start + 1000)) //1s
    {
      FeedbackPotWorker();
      CalculatePID();
      CalculateMotorDirection();
      SetPWM();
      time = millis();
    }
    ST.motor(1, 0);
    ST.motor(2, 0);
    ST.stop();
    disable = 1;
    return;
  }
}

void CalculateMotorDirection()
{
  if (target1 > (currentanalogue1 + (DeadZone + FeedbackPotDeadZone1)) || target1 < (currentanalogue1 - (DeadZone + FeedbackPotDeadZone1)))
  {
    if (OutputM1 >= 0)
    {
      motordirection1 = 1;				// drive motor 1 forward
    }
    else
    {
      motordirection1 = 2;				// drive motor 1 backward
      OutputM1 = abs(OutputM1);
    }
  }
  else
  {
    motordirection1 = 0;
  }

  if (target2 > (currentanalogue2 + DeadZone + FeedbackPotDeadZone2) || target2 < (currentanalogue2 - (DeadZone + FeedbackPotDeadZone2)))
  {
    if (OutputM2 >= 0)
    {
      motordirection2 = 1;				// drive motor 2 forward
    }
    else
    {
      motordirection2 = 2;				// drive motor 2 backward
      OutputM2 = abs(OutputM2);
    }
  }
  else
  {
    motordirection2 = 0;
  }

  OutputM1 = constrain(OutputM1, -255, 255);
  OutputM2 = constrain(OutputM2, -255, 255);
}

int updateMotor1Pid(int targetPosition, int currentPosition)
{
  float error = (float)targetPosition - (float)currentPosition;
  float pTerm_motor_R = proportional1 * error;
  integrated_motor_1_error += error;
  float iTerm_motor_R = integral1 * constrain(integrated_motor_1_error, -GUARD_MOTOR_1_GAIN, GUARD_MOTOR_1_GAIN);
  float dTerm_motor_R = derivative1 * (error - last_motor_1_error);
  last_motor_1_error = error;
  return constrain(K_motor_1 * (pTerm_motor_R + iTerm_motor_R + dTerm_motor_R), -255, 255);
}

int updateMotor2Pid(int targetPosition, int currentPosition)
{
  float error = (float)targetPosition - (float)currentPosition;
  float pTerm_motor_L = proportional2 * error;
  integrated_motor_2_error += error;
  float iTerm_motor_L = integral2 * constrain(integrated_motor_2_error, -GUARD_MOTOR_2_GAIN, GUARD_MOTOR_2_GAIN);
  float dTerm_motor_L = derivative2 * (error - last_motor_2_error);
  last_motor_2_error = error;

  return constrain(K_motor_2 * (pTerm_motor_L + iTerm_motor_L + dTerm_motor_L), -255, 255);
}

void CalculatePID()
{
  OutputM1 = updateMotor1Pid(target1, currentanalogue1);
  OutputM2 = updateMotor2Pid(target2, currentanalogue2);
}

void SetPWM()
{
  //Set hardware pwm
  //Motor 1
  if (motordirection1 != 0)
  {
    //on fait la conXPIDVERSION en cas positif pour PowerM1 (0 Ã  127)
    if (motordirection1 == 1) //forward
    {
      PowerM1 = int(OutputM1 / 2);
    }
    //on fait la conXPIDVERSION en cas negatif pour PowerM1 (-127 Ã  0)
    else //reverse
    {
      PowerM1 = -int(OutputM1 / 2);
    }
    ST.motor(1, PowerM1);
  }
  else //motor1 must stop
  {
    ST.motor(1, 0);
  }

  //Motor 2
  if (motordirection2 != 0)
  {
    if (motordirection2 == 1) //sens positif
    {
      PowerM2 = int(OutputM2 / 2);
    }
    else
    {
      PowerM2 = -int(OutputM2 / 2);
    }
    ST.motor(2, PowerM2);
  }
  else //motor2 must stop
  {
    ST.motor(2, 0);
  }
}

void menuSetup() //setup menu tree and transitions between menu items
{
  menu.getRoot().add(menu_runtime);

  menu_runtime.addAfter(menu_motor1).addAfter(menu_motor2).addAfter(menu_global).addAfter(menu_reset).addAfter(menu_runtime);
  menu_reset.addBefore(menu_global).addBefore(menu_motor2).addBefore(menu_motor1).addBefore(menu_runtime).addBefore(menu_reset);

  menu_motor1.addRight(menu_K_motor_1).addRight(menu_proportional1_motor_1).addRight(menu_integral1_motor_1).addRight(menu_derivative1_motor_1).addRight(menu_stdby_motor_1).addRight(menu_motor1);
  menu_stdby_motor_1.addLeft(menu_derivative1_motor_1).addLeft(menu_integral1_motor_1).addLeft(menu_proportional1_motor_1).addLeft(menu_K_motor_1).addLeft(menu_motor1).addLeft(menu_stdby_motor_1);

  menu_motor2.addRight(menu_K_motor_2).addRight(menu_proportional1_motor_2).addRight(menu_integral1_motor_2).addRight(menu_derivative1_motor_2).addRight(menu_stdby_motor_2).addRight(menu_motor2);
  menu_stdby_motor_2.addLeft(menu_derivative1_motor_2).addLeft(menu_integral1_motor_2).addLeft(menu_proportional1_motor_2).addLeft(menu_K_motor_2).addLeft(menu_motor2).addLeft(menu_stdby_motor_2);

  menu_global.addRight(menu_deadzone).addRight(menu_readanalog).addRight(menu_FeedbackMax1).addRight(menu_FeedbackMin1).addRight(menu_FeedbackMax2).addRight(menu_FeedbackMin2).addRight(menu_global);
  menu_FeedbackMin2.addLeft(menu_FeedbackMax2).addLeft(menu_FeedbackMin1).addLeft(menu_FeedbackMax1).addLeft(menu_readanalog).addLeft(menu_deadzone).addLeft(menu_global);

  menu_reset.addLeft(menu_reset_EEPROM).addLeft(menu_reset).addLeft(menu_reset_EEPROM);
  menu_reset_EEPROM.addRight(menu_reset).addRight(menu_reset_EEPROM);  //menu_derivative1_motor_1.addAfter(menu_motor1);
}

void menuUseEvent(MenuUseEvent used)
{
  //#ifdef DEBUG
  Serial.print(F("Menu use "));
  Serial.println(used.item.getName());
  //#endif
  if (used.item == menu_reset_EEPROM)
  {
    FeedbackMax1 = 1023;
    FeedbackMin1 = 0;
    FeedbackMax2 = 1023;
    FeedbackMin2 = 0;

    FeedbackPotDeadZone1 = 2;		// +/- of this value will not move the motor
    FeedbackPotDeadZone2 = 2;		// +/- of this value will not move the motor
    K_motor_1       = 1;
    proportional1   = 5;
    integral1       = 0.2;
    derivative1     = 0.2;
    K_motor_2       = 1;
    proportional2   = 5;
    integral2       = 0.2;
    derivative2     = 0.2;
    DeadZone        = 2 ; //increase this value to reduce vibrations of motors
    ReadAnalog = 8 ;
    Motor1STDBY = 512 ;
    Motor2STDBY = 512 ;
    SabertoothType = 1;// sabertooth 2*25 =>1  sabertooth 2*32 =>2
    WriteEEValues();
    PrintEEValues();
    PrintValues();
  }
  if (used.item == menu_K_motor_1)
  {
    K_motor_1 = LCDChangeValDouble(K_motor_1, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_K_motor_2)
  {
    K_motor_2 = LCDChangeValDouble(K_motor_2, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_proportional1_motor_1)
  {
    proportional1 = LCDChangeValDouble(proportional1, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_proportional1_motor_2)
  {
    proportional2 = LCDChangeValDouble(proportional2, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_integral1_motor_1)
  {
    integral2 = LCDChangeValDouble(integral2, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_integral1_motor_2)
  {
    integral2 = LCDChangeValDouble(integral2, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_derivative1_motor_1)
  {
    derivative1 = LCDChangeValDouble(derivative1, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_derivative1_motor_2)
  {
    derivative2 = LCDChangeValDouble(derivative2, 5, 0, 0.1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_readanalog)
  {
    ReadAnalog = LCDChangeValInt(ReadAnalog, 16, 1, 1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_deadzone)
  {
    DeadZone = LCDChangeValInt(DeadZone, 254, 0, 1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_FeedbackMax1)
  {
    FeedbackMax1   = LCDChangeValInt(FeedbackMax1  , 254, 0, 1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_FeedbackMin1)
  {
    FeedbackMin1   = LCDChangeValInt(FeedbackMin1  , 254, 0, 1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_FeedbackMax2)
  {
    FeedbackMax2   = LCDChangeValInt(FeedbackMax2  , 254, 0, 1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_FeedbackMin2)
  {
    FeedbackMin2  = LCDChangeValInt(FeedbackMin2 , 254, 0, 1);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_stdby_motor_1)
  {
    Motor1STDBY  = LCDChangeValInt(Motor1STDBY , 1000, 10, 50);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_stdby_motor_2)
  {
    Motor2STDBY  = LCDChangeValInt(Motor2STDBY , 1000, 10, 50);
    WriteEEValues();
    menu.moveRight();
  }
  if (used.item == menu_runtime) {
    LCDruntime = 1;
    delay(500);
    lcd.clear();
    menu.moveDown();
  }
}

/*
	This is an important function
	Here we get a notification whenever the user changes the menu
	That is, when the menu is navigated
*/
void menuChangeEvent(MenuChangeEvent changed)
{
#ifdef DEBUG
  Serial.print(F("Menu change \""));
  Serial.print(changed.from.getName());
  Serial.print(F("\" TO \""));
  Serial.print(changed.to.getName()); Serial.println("\"");
#endif
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(changed.to.getName());
}

void WriteEEValues() { // update EEPROM content with current variable
#ifdef DEBUG
  Serial.print("Updating EEPROM from version "); Serial.print(EEPROM.readInt(addrXPIDVersion));
#endif
  EEPROM.updateInt(addrXPIDVersion, XPIDVersion);
  EEPROM.updateInt(addrFeedbackMax1, FeedbackMax1);
  EEPROM.updateInt(addrFeedbackMax2, FeedbackMax2);
  EEPROM.updateInt(addrFeedbackMin1, FeedbackMin1);
  EEPROM.updateInt(addrFeedbackMin2, FeedbackMin2);
  EEPROM.updateInt(addrFeedbackPotDeadZone1, FeedbackPotDeadZone1);
  EEPROM.updateInt(addrFeedbackPotDeadZone2, FeedbackPotDeadZone2);
  EEPROM.updateDouble(addrK_motor_1, K_motor_1);
  EEPROM.updateDouble(addrproportional1, proportional1);
  EEPROM.updateDouble(addrintegral1, integral1);
  EEPROM.updateDouble(addrderivative1, derivative1);
  EEPROM.updateDouble(addrK_motor_2, K_motor_2);
  EEPROM.updateDouble(addrproportional2, proportional2);
  EEPROM.updateDouble(addrintegral2, integral2);
  EEPROM.updateDouble(addrderivative2, derivative2);
  EEPROM.updateInt(addrDeadZone, DeadZone);
  EEPROM.updateInt(addrReadAnalog, ReadAnalog);
  EEPROM.updateInt(addrSabertoothType, SabertoothType);
  EEPROM.updateInt(addrMotor1STDBY, Motor1STDBY);
  EEPROM.updateInt(addrMotor2STDBY, Motor2STDBY);
#ifdef DEBUG
  Serial.print(" to "); Serial.println(EEPROM.readInt(addrXPIDVersion));
#endif
}

void ReadEEValues() { // update variables with content stored in EEPROM
  FeedbackMax1 = EEPROM.readInt(addrFeedbackMax1);
  FeedbackMax2 = EEPROM.readInt(addrFeedbackMax2);
  FeedbackMin1 = EEPROM.readInt(addrFeedbackMin1);
  FeedbackMin2 = EEPROM.readInt(addrFeedbackMin2);
  FeedbackPotDeadZone1 = EEPROM.readInt(addrFeedbackPotDeadZone1);
  FeedbackPotDeadZone2 = EEPROM.readInt(addrFeedbackPotDeadZone2);
  K_motor_1 = EEPROM.readDouble(addrK_motor_1);
  proportional1 = EEPROM.readDouble(addrproportional1);
  integral1 = EEPROM.readDouble(addrintegral1);
  derivative1 = EEPROM.readDouble(addrderivative1);
  K_motor_2 = EEPROM.readDouble(addrK_motor_2);
  proportional2 = EEPROM.readDouble(addrproportional2);
  integral2 = EEPROM.readDouble(addrintegral2);
  derivative2 = EEPROM.readDouble(addrderivative2);
  DeadZone = EEPROM.readInt(addrDeadZone);
  ReadAnalog = EEPROM.readInt(addrReadAnalog);
  SabertoothType = EEPROM.readInt(addrSabertoothType);
}

void PrintEEValues() {
#ifdef DEBUG
  Serial.println(F("--------------"));
  Serial.println(F("EEPROM Content"));
  Serial.println(F("--------------"));
  Serial.println(F("Var\t\t\t Address\tsize\tval"));
  Serial.print(F("EEInit\t\t\t")); Serial.print(addrXPIDVersion); Serial.print("\t\t"); Serial.print(sizeof(XPIDVersion)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrXPIDVersion)); Serial.println("");
  Serial.print(F("FeedbackMax1\t\t")); Serial.print(addrFeedbackMax1); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMax1)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrFeedbackMax1)); Serial.println("");
  Serial.print(F("FeedbackMin1\t\t")); Serial.print(addrFeedbackMin1); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMin1)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrFeedbackMin1)); Serial.println("");
  Serial.print(F("FeedbackMax2\t\t")); Serial.print(addrFeedbackMax2); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMax2)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrFeedbackMax2)); Serial.println("");
  Serial.print(F("FeedbackMin2\t\t")); Serial.print(addrFeedbackMin2); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMin2)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrFeedbackMin2)); Serial.println("");
  Serial.print(F("FeedbackPotDeadZone1\t")); Serial.print(addrFeedbackPotDeadZone1); Serial.print("\t\t"); Serial.print(sizeof(FeedbackPotDeadZone1)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrFeedbackPotDeadZone1)); Serial.println("");
  Serial.print(F("FeedbackPotDeadZone2\t")); Serial.print(addrFeedbackPotDeadZone2); Serial.print("\t\t"); Serial.print(sizeof(FeedbackPotDeadZone2)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrFeedbackPotDeadZone2)); Serial.println("");
  Serial.print(F("K_motor_1\t\t")); Serial.print(addrK_motor_1); Serial.print("\t\t"); Serial.print(sizeof(K_motor_1)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrK_motor_1)); Serial.println("");
  Serial.print(F("proportional1\t\t")); Serial.print(addrproportional1); Serial.print("\t\t"); Serial.print(sizeof(proportional1)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrproportional1)); Serial.println("");
  Serial.print(F("integral1\t\t")); Serial.print(addrintegral1); Serial.print("\t\t"); Serial.print(sizeof(integral1)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrintegral1)); Serial.println("");
  Serial.print(F("derivative1\t\t")); Serial.print(addrderivative1); Serial.print("\t\t"); Serial.print(sizeof(derivative1)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrderivative1)); Serial.println("");
  Serial.print(F("M1 Stdby\t\t")); Serial.print(addrMotor1STDBY); Serial.print("\t\t"); Serial.print(sizeof(addrMotor1STDBY)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrMotor1STDBY)); Serial.println("");
  Serial.print(F("K_motor_2\t\t")); Serial.print(addrK_motor_2); Serial.print("\t\t"); Serial.print(sizeof(K_motor_2)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrK_motor_2)); Serial.println("");
  Serial.print(F("proportional2\t\t")); Serial.print(addrproportional2); Serial.print("\t\t"); Serial.print(sizeof(proportional2)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrproportional2)); Serial.println("");
  Serial.print(F("integral2\t\t")); Serial.print(addrintegral2); Serial.print("\t\t"); Serial.print(sizeof(integral2)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrintegral2)); Serial.println("");
  Serial.print(F("derivative2\t\t")); Serial.print(addrderivative2); Serial.print("\t\t"); Serial.print(sizeof(derivative2)); Serial.print("\t"); Serial.print(EEPROM.readDouble(addrderivative2)); Serial.println("");
  Serial.print(F("M2 Stdby\t\t")); Serial.print(addrMotor2STDBY); Serial.print("\t\t"); Serial.print(sizeof(addrMotor2STDBY)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrMotor2STDBY)); Serial.println("");
  Serial.print(F("DeadZone\t\t")); Serial.print(addrDeadZone); Serial.print("\t\t"); Serial.print(sizeof(DeadZone)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrDeadZone)); Serial.println("");
  Serial.print(F("ReadAnalog\t\t")); Serial.print(addrReadAnalog); Serial.print("\t\t"); Serial.print(sizeof(ReadAnalog)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrReadAnalog)); Serial.println("");
  Serial.print(F("SabertoothType\t\t")); Serial.print(addrSabertoothType); Serial.print("\t\t"); Serial.print(sizeof(SabertoothType)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrSabertoothType)); Serial.println("");
#endif
}

void PrintValues() {
#ifdef DEBUG
  Serial.println(F("------------"));
  Serial.println(F("Current Vars"));
  Serial.println(F("------------"));
  Serial.println(F("Var\t\t\t Address\tsize\tval"));
  Serial.print(F("EEInit\t\t\t")); Serial.print(addrXPIDVersion); Serial.print("\t\t"); Serial.print(sizeof(XPIDVersion)); Serial.print("\t"); Serial.print(EEPROM.readInt(addrXPIDVersion)); Serial.println("");
  Serial.print(F("FeedbackMax1\t\t")); Serial.print(addrFeedbackMax1); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMax1)); Serial.print("\t"); Serial.print(FeedbackMax1); Serial.println("");
  Serial.print(F("FeedbackMin1\t\t")); Serial.print(addrFeedbackMin1); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMin1)); Serial.print("\t"); Serial.print(FeedbackMin1); Serial.println("");
  Serial.print(F("FeedbackMax2\t\t")); Serial.print(addrFeedbackMax2); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMax2)); Serial.print("\t"); Serial.print(FeedbackMax2); Serial.println("");
  Serial.print(F("FeedbackMin2\t\t")); Serial.print(addrFeedbackMin2); Serial.print("\t\t"); Serial.print(sizeof(FeedbackMin2)); Serial.print("\t"); Serial.print(FeedbackMin2); Serial.println("");
  Serial.print(F("FeedbackPotDeadZone1\t")); Serial.print(addrFeedbackPotDeadZone1); Serial.print("\t\t"); Serial.print(sizeof(FeedbackPotDeadZone1)); Serial.print("\t"); Serial.print(FeedbackPotDeadZone1); Serial.println("");
  Serial.print(F("FeedbackPotDeadZone2\t")); Serial.print(addrFeedbackPotDeadZone2); Serial.print("\t\t"); Serial.print(sizeof(FeedbackPotDeadZone2)); Serial.print("\t"); Serial.print(FeedbackPotDeadZone2); Serial.println("");
  Serial.print(F("K_motor_1\t\t")); Serial.print(addrK_motor_1); Serial.print("\t\t"); Serial.print(sizeof(K_motor_1)); Serial.print("\t"); Serial.print(K_motor_1); Serial.println("");
  Serial.print(F("proportional1\t\t")); Serial.print(addrproportional1); Serial.print("\t\t"); Serial.print(sizeof(proportional1)); Serial.print("\t"); Serial.print(proportional1); Serial.println("");
  Serial.print(F("integral1\t\t")); Serial.print(addrintegral1); Serial.print("\t\t"); Serial.print(sizeof(integral1)); Serial.print("\t"); Serial.print(integral1); Serial.println("");
  Serial.print(F("derivative1\t\t")); Serial.print(addrderivative1); Serial.print("\t\t"); Serial.print(sizeof(derivative1)); Serial.print("\t"); Serial.print(derivative1); Serial.println("");
  Serial.print(F("K_motor_2\t\t")); Serial.print(addrK_motor_2); Serial.print("\t\t"); Serial.print(sizeof(K_motor_2)); Serial.print("\t"); Serial.print(K_motor_2); Serial.println("");
  Serial.print(F("proportional2\t\t")); Serial.print(addrproportional2); Serial.print("\t\t"); Serial.print(sizeof(proportional2)); Serial.print("\t"); Serial.print(proportional2); Serial.println("");
  Serial.print(F("integral2\t\t")); Serial.print(addrintegral2); Serial.print("\t\t"); Serial.print(sizeof(integral2)); Serial.print("\t"); Serial.print(integral2); Serial.println("");
  Serial.print(F("derivative2\t\t")); Serial.print(addrderivative2); Serial.print("\t\t"); Serial.print(sizeof(derivative2)); Serial.print("\t"); Serial.print(derivative2); Serial.println("");
  Serial.print(F("DeadZone\t\t")); Serial.print(addrDeadZone); Serial.print("\t\t"); Serial.print(sizeof(DeadZone)); Serial.print("\t"); Serial.print(DeadZone); Serial.println("");
  Serial.print(F("ReadAnalog\t\t")); Serial.print(addrReadAnalog); Serial.print("\t\t"); Serial.print(sizeof(ReadAnalog)); Serial.print("\t"); Serial.print(ReadAnalog); Serial.println("");
  Serial.print(F("SabertoothType\t\t")); Serial.print(addrSabertoothType); Serial.print("\t\t"); Serial.print(sizeof(SabertoothType)); Serial.print("\t"); Serial.print(SabertoothType); Serial.println("");
#endif
}

void safeBlink(int pin, int count)
{
  /*
   * Safely blink the backlight on LCD shields that have
   * broken BL hardware
   * Uses the SafeBL macros defined above.
   */

  while (count--)
  {
    delay(200);
    SafeBLoff(pin); // turn on the backlight (safe to use for all shields)
    delay(50);
    SafeBLon(pin); // turn off the backlight (safe to use for all shields)
  }
}



