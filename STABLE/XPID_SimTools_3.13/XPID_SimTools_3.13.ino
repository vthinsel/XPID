/*
This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
Original code from sirnoname and Wanegain
*/

/*
	   Input Data Format
	   -----------------
	   10 bits
	   XL~a01~CXR~a02~C
	   XL<Axis1>CXR<Axis2>C
	   XECCC - End

	   Pin out of arduino UNO for Sabertooth
	   --------------------------------------
	   Pin 3   - Sabertooth S1
	   Pin 2   - Switch to enable/disable LCD. Switch open => LCD ON. Switch closed to GND => LCD Off and backlight Off
	   Pin 12  - Emergency switch to disable DC motors between. Switch open => Motors ON . Switch closed to GND => Motors to standby value
	   Pin A1  - input of feedback pot positioning from motor 1. (5V,wiper,GND)
	   Pin A2  - input of feedback pot positioning from motor 2. (5V,wiper,GND))

	   Pin out of arduino MEGA for Sabertooth
	   --------------------------------------
	   Pin TX1(18) - Sabertooth S1 at 115200bps (serial packet mode)
	   Pin TX2(17) - Debug output at 115200bps
	   Pin 53  - Switch to enable/disable LCD. Switch open => LCD ON. Switch closed to GND => LCD and backlight Off
	   Pin 52  - Emergency switch to disable DC motors between. Switch open => Motors ON . Switch closed to GND => Motors to standby value
	   Pin A14 - input of feedback pot positioning from motor 1. (5V,wiper,GND)
	   Pin A15  - input of feedback pot positioning from motor 2. (5V,wiper,GND)

	   LCD/Keypad Shield connection (for Arduino UNO and Arduino MEGA)
	   ---------------------------------------------------------------
		 Pin A0 - Keypad input
		 Pin 10 - LCD backlight control (controlled by software)
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
#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <MenuBackend.h>
#include "XPIDDCMotor.h"
#include <EEPROMex.h>
#undef _EEPROMEX_DEBUG //to prevent false alarms on EEPROM write execeeded
#define FASTADC  1 //Hack to speed up the arduino analogue read function, comment out with // to disable this hack
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

int XPIDVersion = 313;
int DeadZone = 0;
int ReadAnalog = 0;

int currentanalogue1 = 0;
int currentanalogue2 = 0;

int buffer = 0;
int buffercount = -1;
int commandbuffer[5] = { 0 };

// Pot, LCD and emergency pins definition
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
// Pot feedback inputs
int FeedbackPin1 = A14;		// select the input pin for the potentiometer 1
int FeedbackPin2 = A15;		// select the input pin for the potentiometer 2
int EmergencyPin = 52;           // Emergency switch (GND)
int LCDPin = 53;          // LCD toggle switch (GND)
#else //arduino UNO and others
// Pot feedback inputs
int FeedbackPin1 = A1;		// select the input pin for the potentiometer 1
int FeedbackPin2 = A2;		// select the input pin for the potentiometer 2
int EmergencyPin = 12;           // Emergency switch (GND)
int LCDPin = 2;           // LCD toggle switch (GND)
#endif

int disable = 1; //Motor stop flag
int LCDruntime = 1; //Enable LCD output
int SabertoothType = 0;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SimEngineSerialPort Serial
#define SabertoothSerialPort Serial1 //#define SabertoothTXPinSerial Serial1 in case of arduino Mega2560
#define DebugSerial Serial2 //Debug output on serial2 TX pin
//#define DebugSerial Serial //Debug output on serial TX pin
#else //arduino UNO and others
SoftwareSerial SWSerial(NOT_A_PIN, 3); //Pin 3 used to communicate with sabertooth module
#define SimEngineSerialPort Serial
#define DebugSerial Serial //Debug output on serial TX pin (will interfere with native ARDUINO UNO USB/serial communication
#define SabertoothSerialPort SWSerial
#endif

Sabertooth ST(128, SabertoothSerialPort); // Create Sabertooth object based on guessed serial port above
XPIDDCMotor M1;
XPIDDCMotor M2;

#define SafeBLon(pin) pinMode(pin, INPUT)
#define SafeBLoff(pin) pinMode(pin, OUTPUT)
const int pin_BL = 10; //Backlight control pin. Used directly on LCD/Keypad shield

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the LCD shield and buttons
int lcd_key = 5;
int lcd_oldkey = 5;
int adc_key_in = 0;
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
MenuItem menu_FeedbackPotDeadZone1 = MenuItem("M1 DeadZone");

MenuItem menu_motor2 = MenuItem("Motor 2");
MenuItem menu_K_motor_2 = MenuItem("M2 K");
MenuItem menu_proportional1_motor_2 = MenuItem("M2 Prop");
MenuItem menu_integral1_motor_2 = MenuItem("M2 Int");
MenuItem menu_derivative1_motor_2 = MenuItem("M2 Der");
MenuItem menu_stdby_motor_2 = MenuItem("M2 Stdby");
MenuItem menu_FeedbackPotDeadZone2 = MenuItem("M2 DeadZone");

MenuItem menu_global = MenuItem("Global Settings");
MenuItem menu_deadzone = MenuItem("Dead Zone");
MenuItem menu_readanalog = MenuItem("Read Analog");
MenuItem menu_FeedbackMax1 = MenuItem("FeedbackMax1");
MenuItem menu_FeedbackMin1 = MenuItem("FeedbackMin1");
MenuItem menu_FeedbackMax2 = MenuItem("FeedbackMax2");
MenuItem menu_FeedbackMin2 = MenuItem("FeedbackMin2");
MenuItem menu_Sabertooth = MenuItem("ST 1=2.25 2=2.32");

MenuItem menu_reset = MenuItem("Reset");
MenuItem menu_reset_EEPROM = MenuItem("Reset EEPROM");

//EEPROM addresses to store variables
//Do not change order, otherwise offset will be wrong, requiring to reset EEPROM content to default values through LCD menu
int addrReserved = EEPROM.getAddress(sizeof(int));
int addrFeedbackMax1 = EEPROM.getAddress(sizeof(M1.getMax()));
int addrFeedbackMin1 = EEPROM.getAddress(sizeof(M1.getMin()));
int addrFeedbackMax2 = EEPROM.getAddress(sizeof(M2.getMax()));
int addrFeedbackMin2 = EEPROM.getAddress(sizeof(M2.getMin()));
int addrFeedbackPotDeadZone1 = EEPROM.getAddress(sizeof(M1.getDeadzone()));
int addrFeedbackPotDeadZone2 = EEPROM.getAddress(sizeof(M2.getDeadzone()));
int addrK_motor_1 = EEPROM.getAddress(sizeof(M1.getK()));
int addrproportional1 = EEPROM.getAddress(sizeof(M1.getP()));
int addrintegral1 = EEPROM.getAddress(sizeof(M1.getI()));
int addrderivative1 = EEPROM.getAddress(sizeof(M1.getD()));
int addrK_motor_2 = EEPROM.getAddress(sizeof(M2.getK()));
int addrproportional2 = EEPROM.getAddress(sizeof(M2.getP()));
int addrintegral2 = EEPROM.getAddress(sizeof(M2.getI()));
int addrderivative2 = EEPROM.getAddress(sizeof(M2.getD()));
int addrDeadZone = EEPROM.getAddress(sizeof(DeadZone));
int addrReadAnalog = EEPROM.getAddress(sizeof(ReadAnalog));
int addrSabertoothType = EEPROM.getAddress(sizeof(SabertoothType));
int addrXPIDVersion = EEPROM.getAddress(sizeof(XPIDVersion));
int addrMotor1STDBY = EEPROM.getAddress(sizeof(M1.getStandby()));
int addrMotor2STDBY = EEPROM.getAddress(sizeof(M2.getStandby()));

void setup()
{
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
	SimEngineSerialPort.begin(115200);// for connection to SimEngine through FTDI serial USB converter on arduino MEGA board at 115200
	DebugSerial.begin(115200);// Start debug on TX3 pin
#else //arduino UNO and others.
	SimEngineSerialPort.begin(19200);// for connection to SimEngine through FTDI serial USB converter on arduino UNO board at 19200
#endif
  //  EEPROM.setMaxAllowedWrites(5);
	ReadEEValues(); // Init vars from EEPROM. Will be kept in case of EEPROM upgrade
	if (SabertoothType == 2) { //We are managing a sabertooth 2*32 which allows 115200 bps
		SabertoothSerialPort.begin(115200);
	}
	//ST.setBaudRate(115200); //Not needed as should be done using describe software which writes speed in EEPROM of sabertooth module.
	else { //We are managing a sabertooth 2*25 or lower which allows only 38400 bps
		SabertoothSerialPort.begin(38400);
		ST.autobaud(SabertoothSerialPort, true); //Needed so the sabertooth 2*25 is correctly configured
	}
	menuSetup(); //Initialize menu structure
	ST.motor(1, 0);
	ST.motor(2, 0);
	disable = 1;
	// Safe backlight pin mode
	// http://forum.arduino.cc//index.php?topic=96747
	digitalWrite(pin_BL, LOW);
	pinMode(pin_BL, INPUT);
	pinMode(pin_BL, OUTPUT);
	SafeBLon(pin_BL);
	pinMode(LCDPin, INPUT_PULLUP);
	pinMode(EmergencyPin, INPUT_PULLUP);
	lcd.begin(16, 2);
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	PrintEEValues();
	PrintValues();
	DebugSerial.print(F("EEPROM XPIDVERSION:")); DebugSerial.println(EEPROM.readInt(addrXPIDVersion));
	DebugSerial.print(F("XPIDVERSION:")); DebugSerial.println(XPIDVersion);
#endif
	if (EEPROM.readInt(addrXPIDVersion) != XPIDVersion) { // New XPIDVERSION detected. Align EEPROM config
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("EEPROM empty or layout changed. Updating it"));
#endif
		lcd.setCursor(0, 0);
		lcd.print(F("Upgrade EEPROM"));
		InitEEValues();
		delay(1000);
		lcd.setCursor(0, 1);
		lcd.print("Done.");
		delay(2000);
		lcd.clear();
	}
	else {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("EEPROM OK."));
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
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	cbi(ADCSRA, ADPS0);
#endif
}

void loop()
{
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.println(F("Starting navigation:\r\nUp: u   Down: d   Left: l   Right: r   Use: e"));
#endif
	menu.moveDown();
	unsigned long LCDrefresh = 100;
	unsigned long rate = 0;
	unsigned long LCDtimer = 0;
	//Program loop
	while (1 == 1) //Important hack: Use this own real time loop code without arduino framework delays
	{
		rate = millis();
		//delay(2);
		//delayMicroseconds(300);
		FeedbackPotWorker();
		if ((digitalRead(EmergencyPin) == LOW))
		{
			M1.setTarget(M1.getStandby());
			M2.setTarget(M2.getStandby());
		}
		else {
			SerialWorker();
		}
		SetPWM();
		rate = millis() - rate;
		if (!digitalRead(LCDPin) == LOW)
		{
			SafeBLon(pin_BL);
			lcd_key = read_LCD_buttons();
			if (lcd_key != lcd_oldkey)
			{
				lcd_oldkey = lcd_key;
				if (lcd_key > 0) {
					LCDruntime = 0;
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
			if ((LCDruntime == 1) && (millis() - LCDtimer > LCDrefresh)) {
				LCDtimer = millis();
				lcd.setCursor(0, 0);
				lcd.print(F("1:"));
				lcd.setCursor(2, 0);
				lcd.print(M1.getTarget ()); lcd.print("/"); lcd.print(M1.getPower()); lcd.print("/"); lcd.print(analogRead(FeedbackPin1)); lcd.print(F(" "));
				lcd.setCursor(0, 1);
				lcd.print(F("2:"));
				lcd.setCursor(2, 1);
				lcd.print(M2.getTarget()); lcd.print("/"); lcd.print(M2.getPower()); lcd.print("/"); lcd.print(analogRead(FeedbackPin2)); lcd.print(F(" "));
				lcd.print(rate); lcd.print(F(" "));
			}
		}
		else
		{
			SafeBLoff(pin_BL);
		}
		/*#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
			DebugSerial.print(rate); DebugSerial.println(F("ms "));
			DebugSerial.print(rateoverhead); DebugSerial.println(F(" loop ms "));
		#endif
		*/
	}
}
// Read current pot position and update motor accordingly so next PID calculation takes the new value in account
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
	M1.setCurrent(currentanalogue1);
	M2.setCurrent(currentanalogue2);
}

// Calculate PID, and set motor accordingly
void SetPWM()
{
	M1.PIDUpdate();
	M2.PIDUpdate();
	ST.motor(1, M1.getPower());
	ST.motor(2, M2.getPower());
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
			}
			else {
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
		M1.setTarget ( (commandbuffer[1] * 256) + commandbuffer[2]);
		M1.setTarget(map(M1.getTarget(), 0, 1023, M1.getMin(), M1.getMax()));
		disable = 0;
		return;
	}
	if (commandbuffer[0] == 'R')			//Set motor 2 position to High and Low value 0 to 1023
	{
		M2.setTarget((commandbuffer[1] * 256) + commandbuffer[2]);
		M2.setTarget(map(M2.getTarget(), 0, 1023, M2.getMin(), M2.getMax()));
		disable = 0;
		return;
	}
	if (commandbuffer[0] == 'E')		//Disable power on both motor, go to standby value
	{
		unsigned long start;
		unsigned long time;
		start = millis();
		M1.setTarget(M1.getStandby());
		M2.setTarget(M2.getStandby());
		time = millis();
		while (time < (start + 3000)) //3s
		{
			FeedbackPotWorker();
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

double LCDChangeValDouble(double val, double maxVal, double minVal, double stepVal) {
	LCDruntime = 0;
	lcd.setCursor(0, 1);
	lcd.print(val);
	while (true) {
		lcd_key = read_LCD_buttons();
		if (lcd_key != lcd_oldkey)   // if keypress is detected
		{
			lcd_oldkey = lcd_key;
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
			DebugSerial.print(F("LCD ChangeValDouble Key changed:"));
			DebugSerial.println(lcd_key);
#endif
			switch (lcd_key) {
			case btnUP: {
				if (val + stepVal <= maxVal) {
					val += stepVal;
				}
				break;
			}
			case btnDOWN: {
				if (val - stepVal >= minVal) {
					val -= stepVal;
				}
				break;
			}
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
			lcd_oldkey = lcd_key;
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
			DebugSerial.print(F("LCD ChangeValInt Key changed:"));
			DebugSerial.println(lcd_key);
#endif
			switch (lcd_key) {
			case btnUP: {
				if (val + stepVal <= maxVal) {
					val += stepVal;
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
	if (M1.getMax()  <= 0) {
		M1.setMax (1021); // Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
	}
	if (M1.getMin() <= 0) {
		M1.setMin(1); // Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
	}
	if (M2.getMax() <= 0) {
		M2.setMax(1021); // Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
	}
	if (M2.getMin() < 1) {
		M2.setMin(1); // Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
	}
	if (M1.getDeadzone() < 0) {
		M1.setDeadZone(0); // +/- of this value will not move the motor
	}
	if (M2.getDeadzone() < 0) {
		M2.setDeadZone(0); // +/- of this value will not move the motor
	}
	if (M1.getK() <= 0) {
		M1.setK(1);
	}
	if (M1.getP() <= 0) {
		M1.setP(4.000);
	}
	if (M1.getI() <= 0) {
		M1.setI(0.500);
	}
	if (M1.getD() <= 0) {
		M1.setD(0.300);
	}
	if (M2.getK() <= 0) {
		M2.setK(1);
	}
	if (M2.getP() <= 0) {
		M2.setP(4.000);
	}
	if (M2.getI() <= 0) {
		M2.setI(0.500);
	}
	if (M2.getD() <= 0) {
		M2.setD(0.300);
	}
	if (DeadZone <= 0) {
		DeadZone = 4; //increase this value to reduce vibrations of motors
	}
	if (ReadAnalog <= 0) {
		ReadAnalog = 8;
	}
	if (M1.getStandby() <= 0) {
		M1.setStandby(512);
	}
	if (M2.getStandby() <= 0) {
		M2.setStandby(512);
	}
	if (SabertoothType <= 0) {
		addrSabertoothType = 2;// sabertooth 2*25 =>1  sabertooth 2*32 =>2
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

void menuSetup() //setup menu tree and transitions between menu items
{
	menu.getRoot().add(menu_runtime);

	menu_runtime.addAfter(menu_motor1).addAfter(menu_motor2).addAfter(menu_global).addAfter(menu_reset).addAfter(menu_runtime);
	menu_reset.addBefore(menu_global).addBefore(menu_motor2).addBefore(menu_motor1).addBefore(menu_runtime).addBefore(menu_reset);

	menu_motor1.addRight(menu_K_motor_1).addRight(menu_proportional1_motor_1).addRight(menu_integral1_motor_1).addRight(menu_derivative1_motor_1).addRight(menu_stdby_motor_1).addRight(menu_FeedbackPotDeadZone1).addRight(menu_motor1);
	menu_FeedbackPotDeadZone1.addLeft(menu_stdby_motor_1).addLeft(menu_derivative1_motor_1).addLeft(menu_integral1_motor_1).addLeft(menu_proportional1_motor_1).addLeft(menu_K_motor_1).addLeft(menu_motor1).addLeft(menu_FeedbackPotDeadZone1);

	menu_motor2.addRight(menu_K_motor_2).addRight(menu_proportional1_motor_2).addRight(menu_integral1_motor_2).addRight(menu_derivative1_motor_2).addRight(menu_stdby_motor_2).addRight(menu_FeedbackPotDeadZone2).addRight(menu_motor2);
	menu_FeedbackPotDeadZone2.addLeft(menu_stdby_motor_2).addLeft(menu_derivative1_motor_2).addLeft(menu_integral1_motor_2).addLeft(menu_proportional1_motor_2).addLeft(menu_K_motor_2).addLeft(menu_motor2).addLeft(menu_FeedbackPotDeadZone2);

	menu_global.addRight(menu_deadzone).addRight(menu_readanalog).addRight(menu_FeedbackMax1).addRight(menu_FeedbackMin1).addRight(menu_FeedbackMax2).addRight(menu_FeedbackMin2).addRight(menu_Sabertooth).addRight(menu_global);
	menu_Sabertooth.addLeft(menu_FeedbackMin2).addLeft(menu_FeedbackMax2).addLeft(menu_FeedbackMin1).addLeft(menu_FeedbackMax1).addLeft(menu_readanalog).addLeft(menu_deadzone).addLeft(menu_global);

	menu_reset.addLeft(menu_reset_EEPROM).addLeft(menu_reset).addLeft(menu_reset_EEPROM);
	menu_reset_EEPROM.addRight(menu_reset).addRight(menu_reset_EEPROM);  //menu_derivative1_motor_1.addAfter(menu_motor1);
}

void menuUseEvent(MenuUseEvent used)
{
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print(F("Menu use "));
	DebugSerial.println(used.item.getName());
#endif
	if (used.item == menu_reset_EEPROM)
	{
		M1.setK(1);
		M1.setP(4);
		M1.setI(0.5);
		M1.setD(0.3);
		M1.setMax (1023);
		M1.setMin(1);
		M1.setDeadZone(0);		// +/- of this value will not move the motor
		M2.setK(1);
		M2.setP(4);
		M2.setI(0.5);
		M2.setD(0.3);
		M2.setMax(1023);
		M2.setMin(1);
		M2.setDeadZone(0);		// +/- of this value will not move the motor
		DeadZone = 4; //increase this value to reduce vibrations of motors
		ReadAnalog = 8;
		M1.setStandby(512);
		M2.setStandby(512);
		SabertoothType = 2;// sabertooth 2*25 =>1  sabertooth 2*32 =>2
		WriteEEValues();
		PrintEEValues();
		PrintValues();
	}
	if (used.item == menu_Sabertooth)
	{
		SabertoothType = LCDChangeValInt(SabertoothType, 2, 1, 1); //1=2.25 2=2.32
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_K_motor_1)
	{
		M1.setK(LCDChangeValDouble(M1.getK(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_K_motor_2)
	{
		M2.setK(LCDChangeValDouble(M2.getK(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_proportional1_motor_1)
	{
		M1.setP(LCDChangeValDouble(M1.getP(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_proportional1_motor_2)
	{
		M2.setP(LCDChangeValDouble(M2.getP(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_integral1_motor_1)
	{
		M1.setI(LCDChangeValDouble(M1.getI(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_integral1_motor_2)
	{
		M2.setI(LCDChangeValDouble(M2.getI(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_derivative1_motor_1)
	{
		M1.setD(LCDChangeValDouble(M1.getD(), 5, 0, 0.1));
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_derivative1_motor_2)
	{
		M2.setD(LCDChangeValDouble(M2.getD(), 5, 0, 0.1));
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
		DeadZone = LCDChangeValInt(DeadZone, 50, 0, 1);
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_FeedbackMax1)
	{
		M1.setMax(LCDChangeValInt(M1.getMax(), 1023, 0, 1));
		WriteEEValues();
		SetMotor(1, M1.getMax());
		menu.moveRight();
	}
	if (used.item == menu_FeedbackMin1)
	{
		M1.setMin(LCDChangeValInt(M1.getMin(), 1023, 0, 1));
		WriteEEValues();
		SetMotor(1, M1.getMin());
		menu.moveRight();
	}
	if (used.item == menu_FeedbackMax2)
	{
		M2.setMax(LCDChangeValInt(M2.getMax(), 1023, 0, 1));
		WriteEEValues();
		SetMotor(2, M2.getMax());
		menu.moveRight();
	}
	if (used.item == menu_FeedbackMin2)
	{
		M2.setMin(LCDChangeValInt(M2.getMin(), 1023, 0, 1));
		WriteEEValues();
		SetMotor(2, M2.getMin());
		menu.moveRight();
	}
	if (used.item == menu_stdby_motor_1)
	{
		M1.setStandby(LCDChangeValInt(M1.getStandby(), 1023, 0, 10));
		WriteEEValues();
		SetMotor(1, M1.getStandby());
		menu.moveRight();
	}
	if (used.item == menu_stdby_motor_2)
	{
		M2.setStandby(LCDChangeValInt(M2.getStandby(), 1023, 0, 10));
		WriteEEValues();
		SetMotor(2, M2.getStandby());
		menu.moveRight();
	}
	if (used.item == menu_FeedbackPotDeadZone1)
	{
		M1.setDeadZone (LCDChangeValInt(M1.getDeadzone(), 100, 0, 1)+ DeadZone);
		WriteEEValues();
		menu.moveRight();
	}
	if (used.item == menu_FeedbackPotDeadZone2)
	{
		M2.setDeadZone (LCDChangeValInt(M2.getDeadzone(), 100, 0, 1)+ DeadZone);
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


// Move a motor to a given temporary position for 3 seconds
void SetMotor(int motor, int position) {
	int old_target;
	unsigned long time = 0;
	if (motor == 1) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.print(F("SetMotor1 pos: ")); DebugSerial.println(position);
#endif
		old_target = M1.getTarget ();// backup current target
		M1.setTarget(position);
	}
	else {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.print(F("SetMotor2 pos: ")); DebugSerial.println(position);
#endif
		old_target = M2.getTarget ();// backup current target
		M2.setTarget(position);
	}
	time = millis();
	while (millis() - time < 3000) { //Set motor position for 3 seconds to test value
		FeedbackPotWorker();
		SetPWM();
		//    DebugSerial.print(millis()); DebugSerial.print(F("/")); DebugSerial.println(millis() - time);
	}
	//now restore values
	if (motor == 1) {
		M1.setTarget(old_target);
	}
	else {
		M2.setTarget(old_target);
	}
}

void menuChangeEvent(MenuChangeEvent changed)
{
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print(F("Menu change \""));
	DebugSerial.print(changed.from.getName());
	DebugSerial.print(F("\" TO \""));
	DebugSerial.print(changed.to.getName()); DebugSerial.println("\"");
#endif
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(changed.to.getName());
}

void WriteEEValues() { // update EEPROM content with current variable
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print("Updating EEPROM from version "); DebugSerial.print(EEPROM.readInt(addrXPIDVersion));
#endif
	EEPROM.updateInt(addrXPIDVersion, XPIDVersion);
	EEPROM.updateInt(addrFeedbackMax1, M1.getMax());
	EEPROM.updateInt(addrFeedbackMax2, M2.getMax());
	EEPROM.updateInt(addrFeedbackMin1, M1.getMin());
	EEPROM.updateInt(addrFeedbackMin2, M2.getMin());
	EEPROM.updateInt(addrFeedbackPotDeadZone1, M1.getDeadzone());
	EEPROM.updateInt(addrFeedbackPotDeadZone2, M2.getDeadzone());
	EEPROM.updateDouble(addrK_motor_1, M1.getK());
	EEPROM.updateDouble(addrproportional1, M1.getP());
	EEPROM.updateDouble(addrintegral1, M1.getI());
	EEPROM.updateDouble(addrderivative1, M1.getD());
	EEPROM.updateDouble(addrK_motor_2, M2.getK());
	EEPROM.updateDouble(addrproportional2, M2.getP());
	EEPROM.updateDouble(addrintegral2, M2.getI());
	EEPROM.updateDouble(addrderivative2, M2.getD());
	EEPROM.updateInt(addrDeadZone, DeadZone);
	EEPROM.updateInt(addrReadAnalog, ReadAnalog);
	EEPROM.updateInt(addrSabertoothType, SabertoothType);
	EEPROM.updateInt(addrMotor1STDBY, M1.getStandby());
	EEPROM.updateInt(addrMotor2STDBY, M2.getStandby());
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print(" to "); DebugSerial.println(EEPROM.readInt(addrXPIDVersion));
#endif
}

void ReadEEValues() { // update variables with content stored in EEPROM
	M1.setMax(EEPROM.readInt(addrFeedbackMax1));
	M2.setMax(EEPROM.readInt(addrFeedbackMax2));
	M1.setMin(EEPROM.readInt(addrFeedbackMin1));
	M2.setMin(EEPROM.readInt(addrFeedbackMin2));
	M1.setDeadZone(EEPROM.readInt(addrFeedbackPotDeadZone1));
	M2.setDeadZone(EEPROM.readInt(addrFeedbackPotDeadZone2));
	M1.setK(EEPROM.readDouble(addrK_motor_1));
	M1.setP(EEPROM.readDouble(addrproportional1));
	M1.setI(EEPROM.readDouble(addrintegral1));
	M1.setD(EEPROM.readDouble(addrderivative1));
	M2.setK(EEPROM.readDouble(addrK_motor_2));
	M2.setP(EEPROM.readDouble(addrproportional2));
	M2.setI(EEPROM.readDouble(addrintegral2));
	M2.setD(EEPROM.readDouble(addrderivative2));
	M1.setStandby(EEPROM.readInt(addrMotor1STDBY));
	M2.setStandby(EEPROM.readInt(addrMotor2STDBY));
	M1.setTarget(EEPROM.readInt(addrMotor1STDBY));
	M2.setTarget(EEPROM.readInt(addrMotor2STDBY));
	DeadZone = EEPROM.readInt(addrDeadZone);
	ReadAnalog = EEPROM.readInt(addrReadAnalog);
	SabertoothType = EEPROM.readInt(addrSabertoothType);
}

void PrintEEValues() {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	
	DebugSerial.println(F("--------------"));
	DebugSerial.println(F("EEPROM Content"));
	DebugSerial.println(F("--------------"));
	DebugSerial.println(F("Var\t\t\t Address\tsize\tval"));
	DebugSerial.print(F("EEInit\t\t\t")); DebugSerial.print(addrXPIDVersion); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(XPIDVersion)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrXPIDVersion)); DebugSerial.println("");
	DebugSerial.print(F("M1.getMax()\t\t")); DebugSerial.print(addrFeedbackMax1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getMax())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMax1)); DebugSerial.println("");
	DebugSerial.print(F("M1.getMin()\t\t")); DebugSerial.print(addrFeedbackMin1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getMin())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMin1)); DebugSerial.println("");
	DebugSerial.print(F("M2.getMax()\t\t")); DebugSerial.print(addrFeedbackMax2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getMax())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMax2)); DebugSerial.println("");
	DebugSerial.print(F("M2.getMin()\t\t")); DebugSerial.print(addrFeedbackMin2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getMin())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMin2)); DebugSerial.println("");
	DebugSerial.print(F("FeedbackPotDeadZone1\t")); DebugSerial.print(addrFeedbackPotDeadZone1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getDeadzone())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackPotDeadZone1)); DebugSerial.println("");
	DebugSerial.print(F("FeedbackPotDeadZone2\t")); DebugSerial.print(addrFeedbackPotDeadZone2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getDeadzone())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackPotDeadZone2)); DebugSerial.println("");
	DebugSerial.print(F("M1.getK()\t\t")); DebugSerial.print(addrK_motor_1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getK())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrK_motor_1)); DebugSerial.println("");
	DebugSerial.print(F("proportional1\t\t")); DebugSerial.print(addrproportional1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getP())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrproportional1)); DebugSerial.println("");
	DebugSerial.print(F("integral1\t\t")); DebugSerial.print(addrintegral1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getI())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrintegral1)); DebugSerial.println("");
	DebugSerial.print(F("derivative1\t\t")); DebugSerial.print(addrderivative1); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M1.getD())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrderivative1)); DebugSerial.println("");
	DebugSerial.print(F("M1 Stdby\t\t")); DebugSerial.print(addrMotor1STDBY); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(addrMotor1STDBY)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrMotor1STDBY)); DebugSerial.println("");
	DebugSerial.print(F("M2.getK()\t\t")); DebugSerial.print(addrK_motor_2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getK())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrK_motor_1)); DebugSerial.println("");
	DebugSerial.print(F("proportional2\t\t")); DebugSerial.print(addrproportional2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getP())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrproportional2)); DebugSerial.println("");
	DebugSerial.print(F("integral2\t\t")); DebugSerial.print(addrintegral2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getI())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrintegral2)); DebugSerial.println("");
	DebugSerial.print(F("derivative2\t\t")); DebugSerial.print(addrderivative2); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(M2.getD())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrderivative2)); DebugSerial.println("");
	DebugSerial.print(F("M2 Stdby\t\t")); DebugSerial.print(addrMotor2STDBY); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(addrMotor2STDBY)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrMotor2STDBY)); DebugSerial.println("");
	DebugSerial.print(F("DeadZone\t\t")); DebugSerial.print(addrDeadZone); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(DeadZone)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrDeadZone)); DebugSerial.println("");
	DebugSerial.print(F("ReadAnalog\t\t")); DebugSerial.print(addrReadAnalog); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(ReadAnalog)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrReadAnalog)); DebugSerial.println("");
	DebugSerial.print(F("SabertoothType\t\t")); DebugSerial.print(addrSabertoothType); DebugSerial.print("\t\t"); DebugSerial.print(sizeof(SabertoothType)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrSabertoothType)); DebugSerial.println("");

#endif
}

void PrintValues() {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.println(F("------------"));
	DebugSerial.println(F("Current Vars"));
	DebugSerial.println(F("------------"));
	DebugSerial.println(F("Var\t\t\tval"));
	DebugSerial.print(F("EEInit\t\t\t")); DebugSerial.print(EEPROM.readInt(addrXPIDVersion)); DebugSerial.println("");
	DebugSerial.print(F("M1.getMax()\t\t")); DebugSerial.print(M1.getMax()); DebugSerial.println("");
	DebugSerial.print(F("M1.getMin()\t\t")); DebugSerial.print(M1.getMin()); DebugSerial.println("");
	DebugSerial.print(F("M2.getMax()\t\t")); DebugSerial.print(M2.getMax()); DebugSerial.println("");
	DebugSerial.print(F("M2.getMin()\t\t")); DebugSerial.print(M2.getMin()); DebugSerial.println("");
	DebugSerial.print(F("FeedbackPotDeadZone1\t")); DebugSerial.print(M1.getDeadzone ()); DebugSerial.println("");
	DebugSerial.print(F("FeedbackPotDeadZone2\t")); DebugSerial.print(M2.getDeadzone ()); DebugSerial.println("");
	DebugSerial.print(F("M1.getK()\t\t")); DebugSerial.print(M1.getK()); DebugSerial.println("");
	DebugSerial.print(F("proportional1\t\t")); DebugSerial.print(M1.getP()); DebugSerial.println("");
	DebugSerial.print(F("integral1\t\t")); DebugSerial.print(M1.getI()); DebugSerial.println("");
	DebugSerial.print(F("derivative1\t\t")); DebugSerial.print(M1.getD()); DebugSerial.println("");
	DebugSerial.print(F("M2.getK()\t\t")); DebugSerial.print(M2.getK()); DebugSerial.println("");
	DebugSerial.print(F("proportional2\t\t")); DebugSerial.print(M2.getP()); DebugSerial.println("");
	DebugSerial.print(F("integral2\t\t")); DebugSerial.print(M2.getI()); DebugSerial.println("");
	DebugSerial.print(F("derivative2\t\t")); DebugSerial.print(M2.getD()); DebugSerial.println("");
	DebugSerial.print(F("DeadZone\t\t")); DebugSerial.print(DeadZone); DebugSerial.println("");
	DebugSerial.print(F("ReadAnalog\t\t")); DebugSerial.print(ReadAnalog); DebugSerial.println("");
	DebugSerial.print(F("SabertoothType\t\t")); DebugSerial.print(SabertoothType); DebugSerial.println("");
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
