/*
This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
Original code from sirnoname and Wanegain
*/

/*
	   Input Data Format (SimTools 2.0)
	   -----------------
	   10 bits - binary
	   XL<Axis1a>CXR<Axis2a>C
	   XECCC - End

	   Pin out of arduino MEGA for Sabertooth
	   --------------------------------------
	   Pin TX1(18) - Sabertooth S1 at 115200bps (serial packet mode)
	   Pin TX2(17) - Debug output at 115200bps
	   Pin TX3(14) - VisualMicro live debugging (not needed unless debugging with visualstudio + visual micro if needed)
	   Pin RX3(15) - VisualMicro live debugging (not needed unless debugging with visualstudio + visual micro if needed)
	   Pin 53  - Switch to enable/disable TFT. Switch open => TFT ON. Switch closed to GND => TFT and backlight Off
	   Pin 52  - Emergency switch to disable DC motors between. Switch open => Motors ON . Switch closed to GND => Motors to standby value
	   Pin A14 - input of feedback pot positioning from motor 1. (5V,wiper,GND)
	   Pin A15  - input of feedback pot positioning from motor 2. (5V,wiper,GND)

	   
	   	 +-----+
		 +----[PWR]-------------------| USB |--+
		 |                            +-----+  |
		 |           GND/RST2  [ ] [ ]         |
		 |         MOSI2/SCK2  [ ] [ ]  SCL[ ] |   D0
		 |            5V/MISO2 [ ] [ ]  SDA[ ] |   D1
		 |                             AREF[ ] |
		 |                              GND[ ] |
		 | [ ]N/C                    SCK/13[ ]~|   B7
		 | [ ]v.ref                 MISO/12[ ]~|   B6
		 | [ ]RST                   MOSI/11[ ]~|   B5
		 | [ ]3V3      +----------+      10[ ]~|   B4
		 | [ ]5v       | ARDUINO  |       9[ ]~|   H6
		 | [ ]GND      |   MEGA   |       8[ ]~|   H5
		 | [ ]GND      +----------+            |
		 | [ ]Vin                         7[ ]~|   H4
		 |                                6[ ]~|   H3
		 | [ ]A0                          5[ ]~|   E3
		 | [ ]A1                          4[ ]~|   G5
		 | [ ]A2                     INT5/3[ ]~|   E5
		 | [ ]A3                     INT4/2[ ]~|   E4
		 | [ ]A4                       TX>1[ ]~|   E1
		 | [ ]A5                       RX<0[ ]~|   E0
		 | [ ]A6                               |
		 | [ ]A7                     TX3/14[ ] |   J1
		 |                           RX3/15[ ] |   J0
		 | [ ]A8                     TX2/16[ ] |   H1 -- DEBUG OUTPUT 115200/8/N/1
		 | [ ]A9                     RX2/17[ ] |   H0
		 | [ ]A10               TX1/INT3/18[ ] |   D3 -- SABERTOOTH
		 | [ ]A11               RX1/INT2/19[ ] |   D2
		 | [ ]A12           I2C-SDA/INT1/20[ ] |   D1
		 | [ ]A13           I2C-SCL/INT0/21[ ] |   D0
POT1 --	 | [ ]A14                              |
POT2 --	 | [ ]A15                              |   Ports:
		 |                RST SCK MISO         |    22=A0  23=A1
		 |         ICSP   [ ] [ ] [ ]          |    24=A2  25=A3
		 |                [ ] [ ] [ ]          |    26=A4  27=A5
		 |                GND MOSI 5V          |    28=A6  29=A7
		 | G                                   |    30=C7  31=C6
		 | N 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    32=C5  33=C4
		 | D 2 0 8 6 4 2 0 8 6 4 2 0 8 6 4 2 V |    34=C3  35=C2
		 |         ~ ~                         |    36=C1  37=C0
		 | @ # # # # # # # # # # # # # # # # @ |    38=D7  39=G2
		 | @ # # # # # # # # # # # # # # # # @ |    40=G1  41=G0
		 |           ~                         |    42=L7  43=L6
		 | G 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    44=L5  45=L4
		 | N 3 1 9 7 5 3 1 9 7 5 3 1 9 7 5 3 V |    46=L3  47=L2
		 | D                                   |    48=L1  49=L0    SPI:
		 |                                     |    50=B3  51=B2     50=MISO 51=MOSI
		 |     2560                ____________/    52=B1  53=B0     52=SCK  53=SS
		 \_______________________/

		 http://busyducks.com/ascii-art-arduinos

*/

//#define DEBUG
//#define ADAFRUITTFTTS // Uncomment this in case you are using Adafruit TFT
#include <Sabertooth.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>
#include "XPIDDCMotor.h"
#include <EEPROMex.h>
#undef _EEPROMEX_DEBUG //to prevent false alarms on EEPROM write execeeded
#define FASTADC 1 //Hack to speed up the arduino analogue read function, comment out with // to disable this hack
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

int XPIDVersion = 320;
int DeadZone = 0;
int ReadAnalog = 0;

int currentanalogue1 = 0;
int currentanalogue2 = 0;

int buffer = 0;
int buffercount = -1;
int commandbuffer[5] = { 0 };

// Pot, LCD and emergency pins definition
// Pot feedback inputs
int FeedbackPin1 = A14;		// select the input pin for the potentiometer 1
int FeedbackPin2 = A15;		// select the input pin for the potentiometer 2
int EmergencyPin = 52;           // Emergency switch (GND)
int TFTPin = 53;          // TFT toggle switch (GND)


// These are the pins for the TFT Touchscreen shield
#ifdef ADAFRUITTFTTS
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin
#else
#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin
#endif

#define MINPRESSURE 10
#define MAXPRESSURE 1000
int ts_xmin;
int ts_xmax;
int ts_ymin;
int ts_ymax;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint touch;

//TFT-LCD definitions
// Assign human-readable names to some common 16-bit color values:
#define BLACK		0x0000
#define BLUE		0x001F
#define NAVY		0x000F 
#define RED			0xF800
#define GREEN		0x07E0
#define DARKGREEN	0x03E0 
#define CYAN		0x07FF
#define C_PINK		0xF81F
#define PURPLE		0x780F
#define YELLOW		0xFFE0
#define WHITE		0xFFFF
#define LIGHTGREY	0xC618
#define DARKGREY	0x7BEF
#define ROZ			0xFBE0
#define GRI			0xBDF7
#define BUTTONSIZE 40
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4
int page = 1;
int oldcolor, currentcolor;
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

int disable; //Motor stop flag
int LCDruntime = 1; //Enable LCD output
int SabertoothType = 0;

#define SimEngineSerialPort Serial
#define SabertoothSerialPort Serial1 //#define SabertoothTXPinSerial Serial1 in case of arduino Mega2560
//#define DebugSerial Serial2 //Debug output on serial2 TX pin 
#define DebugSerial Serial //Debug output on serial TX pin

Sabertooth ST(128, SabertoothSerialPort); // Create Sabertooth object based on guessed serial port above
XPIDDCMotor M1;
XPIDDCMotor M2;

struct Button {            // struct for holding states of our buttons
	int X;
	int Y;
	int width;
	int height;
	int bgcol;
	int textcol;
	String Label;
	boolean State;
	boolean PrevState;
};

struct EditZone {
	int X;
	int Y;
	int width;
	int height;
	String Label;
};

Button Buttons[] = {
	{5,255,50,50,C_PINK,WHITE,"+",false,false},
	{65,255,50,50,C_PINK,WHITE,"-",false,false},
	{125,255,50,50,GREEN,WHITE,"OK",false,false},
	{185,255,50,50,ROZ,WHITE,"p2",false,false},
	{ 60,28,60,18,BLACK,WHITE,"K1",false,false},
	{ 60,46,60,18,BLACK,WHITE,"P1",false,false},
	{ 60,64,60,18,BLACK,WHITE,"I1",false,false},
	{ 60,82,60,18,BLACK,WHITE,"D1",false,false},
	{ 60,100,60,18,BLACK,WHITE,"DZ1",false,false},
	{ 60,118,60,18,BLACK,WHITE,"MAX1",false,false},
	{ 60,136,60,18,BLACK,WHITE,"MIN1",false,false},
	{ 60,154,60,18,BLACK,WHITE,"STB1",false,false },
	{ 170,28,60,18,BLACK,WHITE,"K2",false,false},
	{ 170,46,60,18,BLACK,WHITE,"P2",false,false },
	{ 170,64,60,18,BLACK,WHITE,"I2",false,false },
	{ 170,82,60,18,BLACK,WHITE,"D2",false,false },
	{ 170,100,60,18,BLACK,WHITE,"DZ2",false,false },
	{ 170,118,60,18,BLACK,WHITE,"MAX2",false,false },
	{ 170,136,60,18,BLACK,WHITE,"MIN2",false,false },
	{ 170,154,60,18,BLACK,WHITE,"STB2",false,false }

};

#define NUMBUTTONS 20

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
int addrDeadZone = EEPROM.getAddress(sizeof(DeadZone)); // TO BE REUSED. Do not uncomment as EEPROM allocation is deterministic.
int addrReadAnalog = EEPROM.getAddress(sizeof(ReadAnalog));
int addrSabertoothType = EEPROM.getAddress(sizeof(SabertoothType));
int addrXPIDVersion = EEPROM.getAddress(sizeof(XPIDVersion));
int addrMotor1STDBY = EEPROM.getAddress(sizeof(M1.getStandby()));
int addrMotor2STDBY = EEPROM.getAddress(sizeof(M2.getStandby()));
int addrts_xmin = EEPROM.getAddress(sizeof(ts_xmin));
int addrts_xmax = EEPROM.getAddress(sizeof(ts_xmax));
int addrts_ymin = EEPROM.getAddress(sizeof(ts_ymin));
int addrts_ymax = EEPROM.getAddress(sizeof(ts_ymax));
/*
void DrawMotorRealTime(XPIDDCMotor M,int X,int Y) {
	int step = 18;
	tft.setTextSize(2);
	tft.setCursor(X, Y + 9 * step); tft.setTextColor(BLUE);tft.print(F("Tget:"));tft.setTextColor(GREEN, RED);tft.print(F("    "));
	tft.setCursor(X, Y + 10 * step); tft.setTextColor(BLUE);tft.print(F("Curr:"));tft.setTextColor(GREEN, RED);tft.print(F("    "));
	tft.setCursor(X, Y + 11 * step); tft.setTextColor(BLUE);tft.print(F("Powe:"));tft.setTextColor(GREEN, RED);tft.print(F("    "));
	tft.setCursor(X+60, Y+9*step);tft.setTextColor(GREEN,RED);tft.print(M.getTarget());tft.print(F(" "));
	tft.setCursor(X+60, Y+10*step);tft.setTextColor(GREEN,RED);tft.print(M.getCurrent());tft.print(F(" "));
	tft.setCursor(X+60, Y+11*step);tft.setTextColor(GREEN,RED);tft.print(M.getPower());tft.print(F(" "));
}
*/

void setup()
{
	unsigned long start;
	SimEngineSerialPort.begin(115200);// for connection to SimEngine through FTDI serial USB converter on arduino MEGA board at 115200
	DebugSerial.begin(115200);// Start debug on Serial2 TX pin
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
	start = millis();
	while (millis() - start < 2000) {
	}
	ST.motor(1, 0);
	ST.motor(2, 0);
	disable = 1;
	pinMode(TFTPin, INPUT_PULLUP);
	pinMode(EmergencyPin, INPUT_PULLUP);
	PrintEEValues();
	PrintValues();
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print(F("EEPROM XPIDVERSION:")); DebugSerial.println(EEPROM.readInt(addrXPIDVersion));
	DebugSerial.print(F("XPIDVERSION:")); DebugSerial.println(XPIDVersion);
#endif
	tft.reset();
	uint16_t identifier = tft.readID();
	if (identifier == 0x9325) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("Found ILI9325 LCD driver"));
#endif
	}
	else if (identifier == 0x9328) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("Found ILI9328 LCD driver"));
#endif
	}
	else if (identifier == 0x7575) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("Found HX8347G LCD driver"));
#endif
	}
	else if (identifier == 0x9341) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("Found ILI9341 LCD driver"));
#endif
	}
	else if (identifier == 0x8357) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("Found HX8357D LCD driver"));
#endif
	}
	else {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.print(F("Unknown LCD driver chip: "));
		DebugSerial.println(identifier, HEX);
		DebugSerial.print(F("I try use ILI9341 LCD driver "));
		DebugSerial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
		DebugSerial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
		DebugSerial.println(F("should appear in the library header (Adafruit_TFT.h)."));
#endif
		identifier = 0x9341;
	}
	tft.begin(identifier);
	tft.setRotation(0);
	tft.fillScreen(BLACK);
	tft.fillRect(0, 0, 320, 240, BLACK);
	if (EEPROM.readInt(addrXPIDVersion) != XPIDVersion) { // New XPIDVERSION detected. Align EEPROM config
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
		DebugSerial.println(F("EEPROM empty or layout changed. Updating it"));
#endif
		tft.setCursor(0, 10);
		tft.setTextSize(2);
		tft.print(F("Upgrade EEPROM"));
		InitEEValues();
		PrintEEValues();
		delay(1500);
		tft.setCursor(0, 40);
		tft.print("Done.");
		delay(1500);
		tft.fillScreen(BLACK);
		tft.fillRect(0, 0, 320, 240, BLACK);
	}
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.println(F("EEPROM OK. Touch TFT to calibrate"));
#endif
	UpdateButtonPressed();
	start = millis();
	tft.setCursor(0, 10);
	tft.setTextSize(2);
	tft.print(F("EEPROM OK"));
	tft.setCursor(0, 30);
	tft.setTextSize(1);
	tft.print(F("Touch TFT to calibrate within 2 seconds"));
	while (millis() - start < 2000) {
		touch = ts.getPoint();
		if (touch.z > MINPRESSURE && touch.z < MAXPRESSURE) {
			TFTcalibrate();
			WriteEEValues();
		}
	}
	UpdateButtonPressed();
	tft.setRotation(0);
	tft.fillScreen(BLACK);
	tft.fillRect(0, 0, 320, 240, BLACK);
	tft.setTextSize(1);tft.setCursor(220, 1);tft.print(XPIDVersion);
#if FASTADC
	// set analogue prescale to 16
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	cbi(ADCSRA, ADPS0);
#endif
	DrawButtons();
	DrawMotorParams("Motor 1", M1, 10, 10);
	DrawMotorParams("Motor 2", M2, 120, 10);
	//DrawMotorRealTime(M1, 10, 10);
	//DrawMotorRealTime(M2, 120, 10);
}


void loop()
{
	unsigned long rate = 0;
	unsigned long pwmrate = 0;
	int signal = 0;
	page = 1;
	//Program loop
	while (1 == 1) //Important hack: Use this own real time loop code without arduino framework delays
	{
		pwmrate = millis();
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
		//pwmrate = millis() - pwmrate;
		//rate = millis();
		//rate = millis() - rate;
		//Serial.println(pwmrate);
		if (page == 1){
			if (UpdateButtonPressed()==true) {
				for (int i = 0;i < NUMBUTTONS;i++) {
					if (Buttons[i].State == true) {
						if (Buttons[i].Label == "p2") { // we switch context to page 2
							tft.fillScreen(BLACK);
							tft.fillRect(0, 0, 320, 240, BLACK);
							//tft.setRotation(1);
							signal = 0; //to start graphing from the origin
							page = 2;
						}
						if (Buttons[i].Label == "K1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);
							tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setK(TFTChangeValDouble(M1.getK(), 5, 0, 0.1,60,200,YELLOW,RED));
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "P1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setP(TFTChangeValDouble(M1.getP(), 5, 0, 0.1, 60, 200, YELLOW, RED));
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "I1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setI(TFTChangeValDouble(M1.getI(), 5, 0, 0.1 ,60, 200, YELLOW, RED));
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "D1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setD(TFTChangeValDouble(M1.getD(), 5, 0, 0.1, 60, 200, YELLOW, RED));
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "DZ1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setDeadZone(TFTChangeValInt(M1.getDeadzone(), 100, 0, 1, 60, 200, YELLOW, RED));
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "MAX1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setMax(TFTChangeValInt(M1.getMax(), 1023, 0, 1, 60, 200, YELLOW, RED));
							SetMotor(1, M1.getMax());
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "MIN1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setMin(TFTChangeValInt(M1.getMin(), 1023, 0, 1, 60, 200, YELLOW, RED));
							SetMotor(1, M1.getMin());
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "STB1") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M1.setStandby(TFTChangeValInt(M1.getStandby(), 1023, 0, 5, 60, 200, YELLOW, RED));
							SetMotor(1, M1.getStandby());
							TFTRefreshM1();
						}
						if (Buttons[i].Label == "K2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);
							tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setK(TFTChangeValDouble(M2.getK(), 5, 0, 0.1, 60, 200, YELLOW, RED));
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "P2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setP(TFTChangeValDouble(M2.getP(), 5, 0, 0.1, 60, 200, YELLOW, RED));
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "I2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setI(TFTChangeValDouble(M2.getI(), 5, 0, 0.1, 60, 200, YELLOW, RED));
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "D2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setD(TFTChangeValDouble(M2.getD(), 5, 0, 0.1, 60, 200, YELLOW, RED));
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "DZ2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setDeadZone(TFTChangeValInt(M2.getDeadzone(), 100, 0, 1, 60, 200, YELLOW, RED));
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "MAX2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setMax(TFTChangeValInt(M2.getMax(), 1023, 0, 1, 60, 200, YELLOW, RED));
							SetMotor(1, M2.getMax());
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "MIN2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setMin(TFTChangeValInt(M2.getMin(), 1023, 0, 1, 60, 200, YELLOW, RED));
							SetMotor(1, M2.getMin());
							TFTRefreshM2();
						}
						if (Buttons[i].Label == "STB2") {
							tft.setCursor(5, 200);
							tft.setTextColor(Buttons[i].textcol, Buttons[i].bgcol);tft.print(Buttons[i].Label);tft.print(F(":"));
							M2.setStandby(TFTChangeValInt(M2.getStandby(), 1023, 0, 5, 60, 200, YELLOW, RED));
							SetMotor(2, M2.getStandby());
							TFTRefreshM2();
						}
						/*
						if (used.item == menu_readanalog)
						{
							ReadAnalog = LCDChangeValInt(ReadAnalog, 16, 1, 1);
							WriteEEValues();
							menu.moveRight();
						}
						if (used.item == menu_reset_EEPROM)
						{
							M1.setK(1);
							M1.setP(2);
							M1.setI(0.5);
							M1.setD(0.3);
							M1.setMax (1023);
							M1.setMin(1);
							M1.setDeadZone(0);		// +/- of this value will not move the motor
							M2.setK(1);
							M2.setP(2);
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
						*/
					}
				}
			ClearButtonPressed();
			}
		}
		if (page == 2) {
			//rate = millis();
			signal = (signal + 1) % 240;
			int16_t p1, p2, p3, p4, p5, p6;
			p1 = map(M1.getCurrent(), M1.getMin(), M1.getMax(), 0, 159);
			p2 = map(M2.getCurrent(), M2.getMin(), M2.getMax(), 161, 320);
			p3 = map(M1.getTarget(), M1.getMin(), M1.getMax(), 0, 159);
			p4 = map(M2.getTarget(), M1.getMin(), M1.getMax(), 161, 320);
			p5 = map(M1.getPower(), -127, 127, 0, 159);
			p6 = map(M2.getPower(), -127, 127, 161, 320);
			//DebugSerial.print(signal);Serial.print(" ");Serial.print(p1);Serial.print(" ");Serial.print(p2);Serial.print(" ");Serial.print(p3);Serial.print(" ");Serial.print(p4);Serial.print(" ");Serial.print(p5);Serial.print(" ");Serial.println(p6);
			tft.drawFastVLine(signal + 1, 0, 320, WHITE); //to clean previous values quickly
			tft.drawPixel(signal, 160, YELLOW);
			/*			
			signal = (signal + 1) % 320;
			int16_t p1, p2, p3, p4,p5,p6;
			p1 = map(M1.getCurrent(), M1.getMin(),M1.getMax(), 5, 235);
			p2 = map(M2.getCurrent(), M2.getMin(), M2.getMax(), 5, 235);
			p3 = map(M1.getTarget() ,0, 1024, 5, 235);
			p4 = map(M2.getTarget(), 0, 1024, 5, 235);
			p5 = map(M1.getPower(), -127, 127, 90, 150);
			p6 = map(M2.getPower(), -127, 127, 90, 150);
			//DebugSerial.print(signal);Serial.print(" ");Serial.print(p1);Serial.print(" ");Serial.print(p2);Serial.print(" ");Serial.print(p3);Serial.print(" ");Serial.print(p4);Serial.print(" ");Serial.print(p5);Serial.print(" ");Serial.println(p6);
			tft.drawFastVLine(signal + 1, 1, 239, WHITE); //to clean previous values quickly
			*/
			tft.drawPixel(signal, p1, GREEN);
			tft.drawPixel(signal, p2, GREEN);
			tft.drawPixel(signal, p3, BLACK);
			tft.drawPixel(signal, p4, BLACK);
			tft.drawPixel(signal, p5, RED);
			tft.drawPixel(signal, p6, RED);
			//rate = millis() - rate;
			//Serial.println(rate);
			if (UpdateButtonPressed()==true){
				ClearButtonPressed();
				page = 1;
				tft.fillScreen(BLACK);
				tft.fillRect(0, 0, 320, 240, BLACK);
				tft.setTextColor(RED);
				tft.setTextSize(1);tft.setCursor(220, 1);tft.print(XPIDVersion);
				DrawButtons();
				DrawMotorParams("Motor 1", M1, 10, 10);
				DrawMotorParams("Motor 2", M2, 120, 10);
			}
		}
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
	if (disable == 0) {
		ST.motor(1, M1.getPower());
		ST.motor(2, M2.getPower());
	}
	else {
		ST.motor(1, 0);
		ST.motor(2, 0);
	}
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
		//M1.setTarget(constrain(M1.getTarget(), M1.getMin(), M1.getMax()));
		disable = 0;
		return;
	}
	if (commandbuffer[0] == 'R')			//Set motor 2 position to High and Low value 0 to 1023
	{
		M2.setTarget((commandbuffer[1] * 256) + commandbuffer[2]);
		M2.setTarget(map(M2.getTarget(), 0, 1023, M2.getMin(), M2.getMax()));
		//M2.setTarget(constrain(M2.getTarget(), M2.getMin(), M2.getMax()));
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
		disable = 0;
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

void TFTcalibrate() {
	unsigned long start;
	UpdateButtonPressed();// to restore PIN states
	tft.setTextSize(2);
	tft.setCursor(0, 130);tft.print(F("Press TFT corners"));
	tft.setCursor(0, 150);tft.print(F("within 10 seconds"));
	start = millis();
	ts_xmax = ts_xmin = ts_ymax = ts_ymin = 512;
	while (millis() - start < 10000) {
		touch = ts.getPoint();
		if (touch.z > MINPRESSURE && touch.z < MAXPRESSURE) {
			if (touch.x > ts_xmax) { ts_xmax = touch.x; }
			if (touch.y > ts_ymax) { ts_ymax = touch.y; }
			if (ts_xmin > touch.x) { ts_xmin = touch.x; }
			if (ts_ymin > touch.y) { ts_ymin = touch.y; }
		}
	}
	UpdateButtonPressed();// to restore PIN states
	tft.setTextSize(2);
	tft.setCursor(0, 200);tft.print(F("xmax:"));tft.print(ts_xmax);tft.print(F(" xmin:"));tft.print(ts_xmin);
	tft.setCursor(0, 220);tft.print(F("ymax:"));tft.print(ts_ymax);tft.print(F(" ymin:"));tft.print(ts_ymin);
	delay(4000);
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print("ts_xmax= ");DebugSerial.println(ts_xmax);
	DebugSerial.print("ts_xmin= ");DebugSerial.println(ts_xmin);
	DebugSerial.print("ts_ymax= ");DebugSerial.println(ts_ymax);
	DebugSerial.print("ts_ymin= ");DebugSerial.println(ts_ymin);
#endif
}

boolean UpdateButtonPressed() {
	touch = ts.getPoint();
	pinMode(XM, OUTPUT);
	digitalWrite(XM, LOW);
	pinMode(YP, OUTPUT);
	digitalWrite(YP, HIGH);
	pinMode(YM, OUTPUT);
	digitalWrite(YM, LOW);
	pinMode(XP, OUTPUT);
	digitalWrite(XP, HIGH);
	if (touch.z > MINPRESSURE && touch.z < MAXPRESSURE) {
		touch.x = map(touch.x, ts_xmin, ts_xmax, 0, 240);
#ifdef ADAFRUITTFT
		touch.y = map(touch.y, ts_ymin, ts_ymax, 320, 0);
#else
		touch.y = map(touch.y, ts_ymin, ts_ymax, 0, 320);
#endif
		//find button pressed
		for (int i = 0; i < NUMBUTTONS;i++) {
			if ((touch.x > Buttons[i].X) && (touch.x < Buttons[i].X + Buttons[i].width) && (touch.y > Buttons[i].Y) && (touch.y < Buttons[i].Y + Buttons[i].height)) {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
				DebugSerial.print("Touched button "); Serial.println(Buttons[i].Label);
#endif
				Buttons[i].State = true;
			}
		}
		return true;
	}
	else {
		return false;
	}
}

void ClearButtonPressed() {
	for (int i = 0;i < NUMBUTTONS;i++) {
		Buttons[i].PrevState = Buttons[i].State;
		Buttons[i].State = false;
	}
}

void DrawButtons() {
	for (int i = 4; i < NUMBUTTONS;i++) {//RED background for editable zone
		//DebugSerial.print(F("Draw bckground button "));DebugSerial.println(i);
		tft.fillRect(Buttons[i].X, Buttons[i].Y, Buttons[i].width, Buttons[i].height, RED);

	}
	for (int i = 0; i <4;i++) {
		//DebugSerial.print(F("Draw button text "));DebugSerial.println(i);
		tft.fillRoundRect(Buttons[i].X, Buttons[i].Y, Buttons[i].width, Buttons[i].height, 5, Buttons[i].bgcol);
		//We center the text on the button, assuming a car is 20x20
		tft.setTextSize(4);
		tft.setCursor((Buttons[i].X) + ((Buttons[i].width) / 2) - (Buttons[i].Label.length() * 12), Buttons[i].Y + 10);
		tft.setTextColor(Buttons[i].textcol);
		tft.print(Buttons[i].Label);
	}
}

void DrawMotorParams(String Label, XPIDDCMotor M, int X, int Y) {
	int step = 18;
	tft.setTextSize(2);
	tft.setCursor(X, Y);tft.setTextColor(RED);tft.print(Label);
	tft.setCursor(X, Y + step); tft.setTextColor(WHITE);tft.print(F("K   :"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getK());
	tft.setCursor(X, Y + 2 * step);tft.setTextColor(WHITE);tft.print(F("P   :"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getP());
	tft.setCursor(X, Y + 3 * step);tft.setTextColor(WHITE);tft.print(F("I   :"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getI());
	tft.setCursor(X, Y + 4 * step);tft.setTextColor(WHITE);tft.print(F("D   :"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getD());
	tft.setCursor(X, Y + 5 * step);tft.setTextColor(WHITE);tft.print(F("DZon:"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getDeadzone());
	tft.setCursor(X, Y + 6 * step);tft.setTextColor(WHITE);tft.print(F("Max :"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getMax());
	tft.setCursor(X, Y + 7 * step);tft.setTextColor(WHITE);tft.print(F("Min :"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getMin());
	tft.setCursor(X, Y + 8 * step);tft.setTextColor(WHITE);tft.print(F("Sdby:"));tft.setTextColor(YELLOW, BLACK);tft.print(M.getStandby());
}

void TFTRefreshM1() {
	DrawMotorParams("Motor 1", M1, 10, 10);
	tft.fillRect(5, 200, 200, 16, BLACK);
	tft.setCursor(120,200);
	tft.setTextColor(GREEN,BLACK);tft.print(F("Save"));
	WriteEEValues();
	tft.fillRect(5, 200, 200, 16, BLACK);
}

void TFTRefreshM2() {
	DrawMotorParams("Motor 2", M2, 120, 10);
	tft.fillRect(5, 200, 200, 16, BLACK);
	tft.setCursor(120, 200);
	tft.setTextColor(GREEN, BLACK);tft.print(F("Save"));
	WriteEEValues();
	tft.fillRect(5, 200, 200, 16, BLACK);
}

double TFTChangeValDouble(double val, double maxVal, double minVal, double stepVal,int X,int Y,int textcol,int bgcol) {
	tft.setCursor(X, Y);
	tft.setTextColor(textcol, bgcol);
	tft.print(val);
	ClearButtonPressed();
	while (1==1) {
		if (UpdateButtonPressed()) {
			for (int i = 0;i < 3;i++) {
				if ((Buttons[i].Label == "+") && (Buttons[i].State == true)) {
					if (val + stepVal <= maxVal) {
						val += stepVal;
					}
					ClearButtonPressed();
				}
				if ((Buttons[i].Label == "-") && (Buttons[i].State == true)) {
					if (val - stepVal >= minVal) {
						val -= stepVal;
					}
					ClearButtonPressed();
				}
				if ((Buttons[i].Label == "OK") && (Buttons[i].State == true)) {
					ClearButtonPressed();
					tft.setCursor(120, Y);
					tft.setTextColor(GREEN,BLACK);tft.print(F("Done"));
					return val;
				}
				tft.setCursor(X, Y);
				tft.setTextColor(textcol, bgcol);
				tft.print(val);tft.print(" ");
			}
		}
	}
}

int TFTChangeValInt(int val, int maxVal, int minVal, int stepVal, int X, int Y, int textcol, int bgcol) {
	tft.setCursor(X, Y);
	tft.setTextColor(textcol, bgcol);
	tft.print(val);
	ClearButtonPressed();
	while (1 == 1) {
		if (UpdateButtonPressed()) {
			for (int i = 0;i < 3;i++) {
				if ((Buttons[i].Label == "+") && (Buttons[i].State == true)) {
					if (val + stepVal <= maxVal) {
						val += stepVal;
					}
					ClearButtonPressed();
				}
				if ((Buttons[i].Label == "-") && (Buttons[i].State == true)) {
					if (val - stepVal >= minVal) {
						val -= stepVal;
					}
					ClearButtonPressed();
				}
				if ((Buttons[i].Label == "OK") && (Buttons[i].State == true)) {
					ClearButtonPressed();
					tft.setCursor(120,Y);
					tft.setTextColor(GREEN,BLACK);tft.print(F("Done"));
					return val;
				}
				tft.setCursor(X, Y);
				tft.setTextColor(textcol, bgcol);
				tft.print(val);tft.print(" ");
			}
		}
	}
}

void InitEEValues() //Used to initialize variables and EEPROM content in case of new arduino, or updated EEPROM schema
{
	if (M1.getMax() <= 0 || M1.getMax() >1000 || isnan(M1.getMax())) {
		M1.setMax (1021); // Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
	}
	if (M1.getMin() <= 0 || M1.getMin() >1000 || isnan(M1.getMin())) {
		M1.setMin(1); // Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
	}
	if (M2.getMax() <= 0 || M2.getMax() >1000 || isnan(M2.getMax())) {
		M2.setMax(1021); // Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
	}
	if (M2.getMin() < 1 || M2.getMin() >1000 || isnan(M2.getMin())) {
		M2.setMin(1); // Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
	}
	if (M1.getDeadzone() < 0 || M1.getDeadzone() > 100 || isnan(M1.getDeadzone())) {
		M1.setDeadZone(5); // +/- of this value will not move the motor
	}
	if (M2.getDeadzone() < 0 || M2.getDeadzone() > 100 || isnan(M2.getDeadzone())) {
		M2.setDeadZone(5); // +/- of this value will not move the motor
	}
	if (M1.getK() <= 0 || M1.getK() >= 5 || isnan(M1.getK()) ) {
		M1.setK(1);
	}
	if (M1.getP() <= 0 || M1.getP() >= 5 || isnan(M1.getP())) {
		M1.setP(1.000);
	}
	if (M1.getI() <= 0 || M1.getI() >= 5 || isnan(M1.getI()) ) {
		M1.setI(1.500);
	}
	if (M1.getD() <= 0 || M1.getD() >= 5 || isnan(M1.getD())) {
		M1.setD(0.300);
	}
	if (M2.getK() <= 0 || M2.getK() >= 5 || isnan(M2.getK())) {
		M2.setK(1);
	}
	if (M2.getP() <= 0 || M2.getP() >= 5 || isnan(M2.getP())) {
		M2.setP(1.000);
	}
	if (M2.getI() <= 0 || M2.getI() >= 10 || isnan(M2.getI())) {
		M2.setI(0.500);
	}
	if (M2.getD() <= 0 || M2.getD() >= 10 || isnan(M2.getD())) {
		M2.setD(0.300);
	}
	if (DeadZone <= 0 || DeadZone >= 50 || isnan(DeadZone)) {
		DeadZone = 4; //increase this value to reduce vibrations of motors
	}
	if (ReadAnalog <= 0 || ReadAnalog >= 8 || isnan(ReadAnalog)) {
		ReadAnalog = 8;
	}
	if (M1.getStandby() <= 0 || M1.getStandby() || isnan(M1.getStandby()) >= 1024 ) {
		M1.setStandby(512);
	}
	if (M2.getStandby() <= 0 || M2.getStandby() || isnan(M2.getStandby()) >= 1024) {
		M2.setStandby(512);
	}
	if (SabertoothType <= 0 || SabertoothType > 2) {
		addrSabertoothType = 2;// sabertooth 2*25 =>1  sabertooth 2*32 =>2
	}
	if (ts_xmax <= 0 || ts_xmax >= 1024) {
		ts_xmax = 1000;
	}
	if (ts_xmin <= 0 || ts_xmin >= 1024) {
		ts_xmin = 0;
	}
	if (ts_ymax <= 0 || ts_ymax >= 1024) {
		ts_ymax = 1000;
	}
	if (ts_ymin <= 0 || ts_ymin >= 1024) {
		ts_ymin = 0;
	}
	WriteEEValues();
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
	}
	//now restore values
	if (motor == 1) {
		M1.setTarget(old_target);
	}
	else {
		M2.setTarget(old_target);
	}
}


void WriteEEValues() { // update EEPROM content with current variable
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.print("Updating EEPROM data version "); DebugSerial.println(EEPROM.readInt(addrXPIDVersion));
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
	EEPROM.updateInt(addrts_xmax, ts_xmax);
	EEPROM.updateInt(addrts_xmin, ts_xmin);
	EEPROM.updateInt(addrts_ymax, ts_ymax);
	EEPROM.updateInt(addrts_ymin, ts_ymin);
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
	ts_xmax = EEPROM.readInt(addrts_xmax);
	ts_xmin = EEPROM.readInt(addrts_xmin);
	ts_ymax = EEPROM.readInt(addrts_ymax);
	ts_ymin = EEPROM.readInt(addrts_ymin);
}

void PrintEEValues() {
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) || defined DEBUG
	DebugSerial.println(F("--------------"));
	DebugSerial.println(F("EEPROM Content"));
	DebugSerial.println(F("--------------"));
	DebugSerial.println(F("Var\t\tAddress\tsize\tval"));
	DebugSerial.print(F("EEInit\t\t")); DebugSerial.print(addrXPIDVersion); DebugSerial.print("\t"); DebugSerial.print(sizeof(XPIDVersion)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrXPIDVersion)); DebugSerial.println("");
	DebugSerial.print(F("M1.getMax\t")); DebugSerial.print(addrFeedbackMax1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getMax())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMax1)); DebugSerial.println("");
	DebugSerial.print(F("M1.getMin\t")); DebugSerial.print(addrFeedbackMin1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getMin())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMin1)); DebugSerial.println("");
	DebugSerial.print(F("M1.DeadZone\t")); DebugSerial.print(addrFeedbackPotDeadZone1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getDeadzone())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackPotDeadZone1)); DebugSerial.println("");
	DebugSerial.print(F("M2.getMax\t")); DebugSerial.print(addrFeedbackMax2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getMax())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMax2)); DebugSerial.println("");
	DebugSerial.print(F("M2.getMin\t")); DebugSerial.print(addrFeedbackMin2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getMin())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackMin2)); DebugSerial.println("");
	DebugSerial.print(F("M2.DeadZone\t")); DebugSerial.print(addrFeedbackPotDeadZone2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getDeadzone())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrFeedbackPotDeadZone2)); DebugSerial.println("");
	DebugSerial.print(F("M1.K\t\t")); DebugSerial.print(addrK_motor_1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getK())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrK_motor_1)); DebugSerial.println("");
	DebugSerial.print(F("M1.P\t\t")); DebugSerial.print(addrproportional1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getP())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrproportional1)); DebugSerial.println("");
	DebugSerial.print(F("M1.I\t\t")); DebugSerial.print(addrintegral1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getI())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrintegral1)); DebugSerial.println("");
	DebugSerial.print(F("M1.D\t\t")); DebugSerial.print(addrderivative1); DebugSerial.print("\t"); DebugSerial.print(sizeof(M1.getD())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrderivative1)); DebugSerial.println("");
	DebugSerial.print(F("M1.Stdby\t")); DebugSerial.print(addrMotor1STDBY); DebugSerial.print("\t"); DebugSerial.print(sizeof(addrMotor1STDBY)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrMotor1STDBY)); DebugSerial.println("");
	DebugSerial.print(F("M2.K\t\t")); DebugSerial.print(addrK_motor_2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getK())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrK_motor_1)); DebugSerial.println("");
	DebugSerial.print(F("M2.P\t\t")); DebugSerial.print(addrproportional2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getP())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrproportional2)); DebugSerial.println("");
	DebugSerial.print(F("M2.I\t\t")); DebugSerial.print(addrintegral2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getI())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrintegral2)); DebugSerial.println("");
	DebugSerial.print(F("M2.D\t\t")); DebugSerial.print(addrderivative2); DebugSerial.print("\t"); DebugSerial.print(sizeof(M2.getD())); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readDouble(addrderivative2)); DebugSerial.println("");
	DebugSerial.print(F("M2.Stdby\t")); DebugSerial.print(addrMotor2STDBY); DebugSerial.print("\t"); DebugSerial.print(sizeof(addrMotor2STDBY)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrMotor2STDBY)); DebugSerial.println("");
	DebugSerial.print(F("DeadZone\t")); DebugSerial.print(addrDeadZone); DebugSerial.print("\t"); DebugSerial.print(sizeof(DeadZone)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrDeadZone)); DebugSerial.println("");
	DebugSerial.print(F("ReadAnalog\t")); DebugSerial.print(addrReadAnalog); DebugSerial.print("\t"); DebugSerial.print(sizeof(ReadAnalog)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrReadAnalog)); DebugSerial.println("");
	DebugSerial.print(F("SabertoothType\t")); DebugSerial.print(addrSabertoothType); DebugSerial.print("\t"); DebugSerial.print(sizeof(SabertoothType)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrSabertoothType)); DebugSerial.println("");
	DebugSerial.print(F("Touch Xmax\t")); DebugSerial.print(addrts_xmax); DebugSerial.print("\t"); DebugSerial.print(sizeof(ts_xmax)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrts_xmax)); DebugSerial.println("");
	DebugSerial.print(F("Touch Xmin\t")); DebugSerial.print(addrts_xmin); DebugSerial.print("\t"); DebugSerial.print(sizeof(ts_xmin)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrts_xmin)); DebugSerial.println("");
	DebugSerial.print(F("Touch Ymax\t")); DebugSerial.print(addrts_ymax); DebugSerial.print("\t"); DebugSerial.print(sizeof(ts_ymax)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrts_ymax)); DebugSerial.println("");
	DebugSerial.print(F("Touch Ymin\t")); DebugSerial.print(addrts_ymin); DebugSerial.print("\t"); DebugSerial.print(sizeof(ts_ymin)); DebugSerial.print("\t"); DebugSerial.print(EEPROM.readInt(addrts_ymin)); DebugSerial.println("");
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
	DebugSerial.print(F("Tscreen xmax\t\t")); DebugSerial.print(ts_xmax); DebugSerial.println("");
	DebugSerial.print(F("Tscreen xmin\t\t")); DebugSerial.print(ts_xmin); DebugSerial.println("");
	DebugSerial.print(F("Tscreen ymax\t\t")); DebugSerial.print(ts_ymax); DebugSerial.println("");
	DebugSerial.print(F("Tscreen ymin\t\t")); DebugSerial.print(ts_ymin); DebugSerial.println("");
#endif
}

