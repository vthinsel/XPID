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

        Pin 11: - Switch to enable/disable LCD. Switch open => LCD ON. Switch closed to GND => LCD Off
        Pin 12: - Emergency switch to disable DC motors between. Switch open => Motors ON . Switch closed to GND => Motors Off

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

#include <SabertoothSimplified.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <EEPROMex.h>
#include <EEPROMVar.h>

//Some speed test switches for testers ;)
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

int DeadZone = 5; //increase this value to reduce vibrations of motors
int ReadAnalog = 8;

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
int LCDPin			        = 11;            // LCD toggle switch (GND)
int LCDBacklight		        = 10;            // LCD backlight switch (GND)

//180°

int FeedbackMax1;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin1;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
int FeedbackMax2;		// Maximum position of pot 2 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin2;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range

//150°
/*
int FeedbackMax1			= 888;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin1			= 136;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
int FeedbackMax2			= 888;		// Maximum position of pot 2 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin2			= 136;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
*/

int FeedbackPotDeadZone1;		// +/- of this value will not move the motor
int FeedbackPotDeadZone2;		// +/- of this value will not move the motor

//PID variables
int motordirection1	= 0;			// motor 1 move direction 0=brake, 1=forward, 2=reverse
int motordirection2	= 0;			// motor 2 move direction 0=brake, 1=forward, 2=reverse
int oldmotordirection1	= 0;
int oldmotordirection2	= 0;

double K_motor_1;
double proportional1;
double integral1;
double derivative1;
double K_motor_2;
double proportional2;
double integral2;
double derivative2;

int OutputM1			= 0;
int OutputM2			= 0;

double integrated_motor_1_error = 0;
double integrated_motor_2_error = 0;
float last_motor_1_error		= 0;
float last_motor_2_error		= 0;

int disable				= 1; //Motor stop flag

//Variables pour la liaison serie de la sabertooth
int PowerM1 = 0;              //puissance moteur 1 [-127 127]
int PowerM2 = 0;              //puissance moteur 2 [-127 127]

SoftwareSerial SWSerial(NOT_A_PIN, 3); //Pin 10 utilisé pour communiquer avec la Sabertooth
SabertoothSimplified ST(SWSerial); //nom de l'objet communication serie avec une pin differente de TX

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
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//EEPROM addresses to store variables
int addrEEInit = EEPROM.getAddress(sizeof(int)); // to detect when EEPROM is empty to force initial values in it
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

void WriteEEValues() { // update EEPROM content with current variable
  EEPROM.updateInt(addrEEInit, 1);
  EEPROM.updateInt(addrFeedbackMax1, FeedbackMax1);
  EEPROM.updateInt(addrFeedbackMax2, FeedbackMax2);
  EEPROM.updateInt(addrFeedbackMin2, FeedbackMin2);
  EEPROM.updateInt(addrFeedbackMin2, FeedbackMin2);
  EEPROM.updateInt(addrFeedbackPotDeadZone1, FeedbackPotDeadZone1);
  EEPROM.updateInt(addrFeedbackPotDeadZone1, FeedbackPotDeadZone2);
  EEPROM.updateDouble(addrK_motor_1, K_motor_1);
  EEPROM.updateDouble(addrproportional1, proportional1);
  EEPROM.updateDouble(addrintegral1, integral1);
  EEPROM.updateDouble(addrderivative1, derivative1);
  EEPROM.updateDouble(addrK_motor_2, K_motor_2);
  EEPROM.updateDouble(addrproportional2, proportional2);
  EEPROM.updateDouble(addrintegral2, integral2);
  EEPROM.updateDouble(addrderivative2, derivative2);
}

void ReadEEValues() { // update variables with content stored in EEPROM
  FeedbackMax1 = EEPROM.readInt(addrFeedbackMax1);
  FeedbackMax2 = EEPROM.readInt(addrFeedbackMax2);
  FeedbackMin2 =  EEPROM.readInt(addrFeedbackMin2);
  FeedbackMin2 =  EEPROM.readInt(addrFeedbackMin2);
  FeedbackPotDeadZone1 =  EEPROM.readInt(addrFeedbackPotDeadZone1);
  FeedbackPotDeadZone2 =  EEPROM.readInt(addrFeedbackPotDeadZone1);
  K_motor_1 =  EEPROM.readDouble(addrK_motor_1);
  proportional1 =  EEPROM.readDouble(addrproportional1);
  integral1 =  EEPROM.readDouble(addrintegral1);
  derivative1 =  EEPROM.readDouble(addrderivative1);
  K_motor_2 =  EEPROM.readDouble(addrK_motor_2);
  proportional2 =  EEPROM.readDouble(addrproportional2);
  integral2 =  EEPROM.readDouble(addrintegral2);
  derivative2 =  EEPROM.readDouble(addrderivative2);
}

void InitEEValues() //Used to initialize variables and EEPROM content
{

//150°
/*
 FeedbackMax1			= 888;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
 FeedbackMin1			= 136;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
 FeedbackMin2			= 136;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
*/
// 180°
  FeedbackMax1			= 962;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
  FeedbackMin1			= 62;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
  FeedbackMax2			= 962;		// Maximum position of pot 2 to scale, do not use 1023 because it cannot control outside the pot range
  FeedbackMin2			= 62;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
  FeedbackPotDeadZone1	= 0;		// +/- of this value will not move the motor
  FeedbackPotDeadZone2	= 0;		// +/- of this value will not move the motor
  K_motor_1		= 1;
  proportional1	        = 5.000;
  integral1		= 0.200;
  derivative1		= 0.200;
  K_motor_2		= 1;
  proportional2	        = 5.000;
  integral2		= 0.200;
  derivative2		= 0.200;
  WriteEEValues();
}


void PrintEEValues() {

  Serial.println("-----------------------------------");
  Serial.println("Following adresses have been issued");
  Serial.println("-----------------------------------");

  Serial.println("begin adress \t\t size");
  Serial.print(addrEEInit);      Serial.print(" \t\t\t "); Serial.print(sizeof(int)); Serial.println("");
  Serial.print(addrFeedbackMax1);      Serial.print(" \t\t\t "); Serial.print(sizeof(FeedbackMax1)); Serial.println("");
  Serial.print(addrFeedbackMin1);       Serial.print(" \t\t\t "); Serial.print(sizeof(FeedbackMin1));  Serial.println("");
  Serial.print(addrFeedbackMax2);      Serial.print(" \t\t\t "); Serial.print(sizeof(FeedbackMax2)); Serial.println("");
  Serial.print(addrFeedbackMin2);      Serial.print(" \t\t\t "); Serial.print(sizeof(FeedbackMin2)); Serial.println("");
  Serial.print(addrFeedbackPotDeadZone1);      Serial.print(" \t\t\t "); Serial.print(sizeof(FeedbackPotDeadZone1)); Serial.println("");
  Serial.print(addrFeedbackPotDeadZone2);      Serial.print(" \t\t\t "); Serial.print(sizeof(FeedbackPotDeadZone2)); Serial.println("");
  Serial.print(addrK_motor_1);      Serial.print(" \t\t\t "); Serial.print(sizeof(K_motor_1)); Serial.println("");
  Serial.print(addrproportional1);      Serial.print(" \t\t\t "); Serial.print(sizeof(proportional1)); Serial.println("");
  Serial.print(addrintegral1);      Serial.print(" \t\t\t "); Serial.print(sizeof(integral1)); Serial.println("");
  Serial.print(addrderivative1);      Serial.print(" \t\t\t "); Serial.print(sizeof(derivative1)); Serial.println("");
  Serial.print(addrK_motor_2);      Serial.print(" \t\t\t "); Serial.print(sizeof(K_motor_2)); Serial.println("");
  Serial.print(addrproportional2);      Serial.print(" \t\t\t "); Serial.print(sizeof(proportional2)); Serial.println("");
  Serial.print(addrintegral2);      Serial.print(" \t\t\t "); Serial.print(sizeof(integral2)); Serial.println("");
  Serial.print(addrderivative2);      Serial.print(" \t\t\t "); Serial.print(sizeof(derivative2)); Serial.println("");
  Serial.println("Current values:");
  Serial.print("EEInit:"); Serial.print(EEPROM.read(addrEEInit)); Serial.print(" ");
  Serial.print("FeedbackMax1:"); Serial.print(FeedbackMax1); Serial.print(" ");
  Serial.print("FeedbackMax2:"); Serial.print(FeedbackMax2); Serial.print(" ");
  Serial.print("FeedbackMin2:"); Serial.print(FeedbackMin2); Serial.print(" ");
  Serial.print("FeedbackPotDeadZone1:"); Serial.print(FeedbackPotDeadZone1); Serial.print(" ");
  Serial.print("FeedbackPotDeadZone2:"); Serial.print(FeedbackPotDeadZone2); Serial.print(" ");
  Serial.print("K_motor_1:"); Serial.print(K_motor_1); Serial.print(" ");
  Serial.print("proportional1:"); Serial.print(proportional1); Serial.print(" ");
  Serial.print("integral1:"); Serial.print(integral1); Serial.print(" ");
  Serial.print("derivative1:"); Serial.print(derivative1); Serial.print(" ");
  Serial.print("K_motor_2:"); Serial.print(K_motor_2); Serial.print(" ");
  Serial.print("proportional2:"); Serial.print(proportional2); Serial.print(" ");
  Serial.print("integral2:"); Serial.print(integral2); Serial.print(" ");
  Serial.print("derivative2:"); Serial.print(derivative2); Serial.print(" ");
}


void setup()
{
  Serial.begin(9600);
  //  while (!Serial) {
  ; // wait for serial port to connect. Needed for Leonardo only
  //}
  if (EEPROM.read(addrEEInit) != 1) {
    Serial.println("EEPROM empty. Initializing it");
    InitEEValues();
  } else {
    Serial.println("EEPROM OK. Reading it");
    ReadEEValues();
  }
  PrintEEValues();
  SWSerial.begin(38400); //boutons 2, 4 et 5 OFF
  ST.motor(1, 0);
  ST.motor(2, 0);
  disable = 1;
  pinMode(LCDPin, INPUT_PULLUP);
  pinMode(EmergencyPin, INPUT_PULLUP);
  lcd.begin(16, 2);

#if FASTADC
  // set analogue prescale to 16
  sbi(ADCSRA, ADPS2) ;
  cbi(ADCSRA, ADPS1) ;
  cbi(ADCSRA, ADPS0) ;
#endif
}


int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
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
    target1 = 512;
    target2 = 512;

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
    //on fait la conversion en cas positif pour PowerM1 (0 à 127)
    if (motordirection1 == 1) //forward
    {
      PowerM1 = int(OutputM1 / 2);
    }
    //on fait la conversion en cas negatif pour PowerM1 (-127 à 0)
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

void loop()
{
  //Program loop
  while (1 == 1) //Important hack: Use this own real time loop code without arduino framework delays
  {
    //
    /*if (digitalRead(EmergencyPin) == LOW)
    {
      target1 = 512;
      target2 = 512;
      ST.motor(1, 0);
      ST.motor(2, 0);
      ST.stop();
    }
    else {*/
    FeedbackPotWorker();
    SerialWorker();
    CalculatePID();
    CalculateMotorDirection();
    if (disable == 0)
    {
      SetPWM();
    }
    //    }
    if (!digitalRead(LCDPin) == LOW)
    {
      // print the number of seconds since reset:
      //lcd.print(millis() / 1000); lcd.print(" ");
      //lcd.print(digitalRead(EmergencyPin));
      lcd.setCursor(0, 0);
      lcd.print("L:");
      lcd.setCursor(2, 0);
      lcd.print(target1); lcd.print("/"); lcd.print(PowerM1); lcd.print("/"); lcd.print(analogRead(FeedbackPin1)); lcd.print("  ");
      lcd.setCursor(0, 1);
      lcd.print("R:");
      lcd.setCursor(2, 1);
      lcd.print(target2); lcd.print("/"); lcd.print(PowerM2); lcd.print("/"); lcd.print(analogRead(FeedbackPin2)); lcd.print("  ");
      //Serial.println(millis() / 1000);
    }
  }
}

