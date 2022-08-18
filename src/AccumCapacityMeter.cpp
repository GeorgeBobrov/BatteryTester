#include "wiring_private.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>


constexpr byte pinEncButton = A1;
constexpr byte pinEnc1 = 3;
constexpr byte pinEnc2 = 2;
constexpr byte pinAnVoltage = A7;
constexpr byte pinDischarge = 4;

constexpr float loadResistor = 30.2;
// Resistor divider and internal reference voltage 
constexpr float voltageKoef = ((20100.0 + 5080.0) / 5080.0) * 1.08;


Encoder encoder(pinEnc1, pinEnc2, pinEncButton); 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void isrCLK() {
	encoder.tick();
}
							

void isrDT() {
	encoder.tick();
}

void setup() {
	// Buzzer connected to LED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
//	pinMode(pinEncButton, INPUT_PULLUP);
	analogReference(INTERNAL);

	Serial.begin(500000);
	encoder.setType(TYPE2);

	// mode = EEPROM.read(0);

	display.begin();


	attachInterrupt(0, isrCLK, CHANGE);
	attachInterrupt(1, isrDT, CHANGE);

}

unsigned long lastTimeRead_us = 0;
bool enableWriteCom;
bool enableDischarge;

unsigned long dischargeTime_us; 
float accumCapacity_AH;
constexpr float us_in_hour = 60.0 * 60.0 * 1000.0 * 1000.0;
void printTime(unsigned long time_s, char startXpos, char startYpos, char charWidth);

void loop() {

	encoder.tick();

	unsigned long curTime_us = micros();
	unsigned long periodFromLastMeasure_us = curTime_us - lastTimeRead_us;
	if (periodFromLastMeasure_us >= 50000) {
		lastTimeRead_us = curTime_us;

 
		if (encoder.isClick()){

			enableDischarge = !enableDischarge;
			digitalWrite(pinDischarge, enableDischarge);

			if (enableDischarge) {
				dischargeTime_us = 0;
				accumCapacity_AH = 0;
				// tone(LED_BUILTIN, 2000, 500);
				tone(LED_BUILTIN, 2, 2000);

 			}


			int readed = Serial.read();
			if (readed > -1) {
				char command = readed - '0';
				if (command < 2)
					enableWriteCom = command;
			}

			// delay(500);	 
		}



		int accumVoltageD = analogRead(pinAnVoltage);
		float accumVoltage = (accumVoltageD / 1024.0) * voltageKoef;
		float loadCurrent = accumVoltage / loadResistor;

		if (enableDischarge) {
			float accumCapacityPerPeriod_AH = loadCurrent * periodFromLastMeasure_us / us_in_hour;
			accumCapacity_AH += accumCapacityPerPeriod_AH;
			dischargeTime_us += periodFromLastMeasure_us;
		}

		if ((accumVoltage < 3.0) && enableDischarge) { // stopDischarge
			enableDischarge = false;
			digitalWrite(pinDischarge, enableDischarge);
			tone(LED_BUILTIN, 1, 2000);
		}

		if (enableWriteCom) {
			Serial.print(F("accumVoltage="));
			Serial.print(accumVoltage); 

			Serial.print(F("\tdischargeTime="));
			Serial.print(dischargeTime_us); 

			Serial.print(F("\taccumCapacity="));
			Serial.print(accumCapacity_AH * 1000); 

		}


		display.clearBuffer(); 
		display.setFont(u8g2_font_6x10_tf); 
		constexpr char charWidth = 6;

		display.setCursor(0, 10);
		if (enableDischarge) {
			display.print(F("Discharge")); 

			display.setCursor(10*charWidth, 10);
			display.print(F("Cur       A")); 
			display.setCursor(14*charWidth, 10);
			display.print(loadCurrent, 3); 		
		}

		display.setCursor(0, 25);
		display.print(F("Voltage        V")); 
		display.setCursor(9*charWidth, 25);
		display.print(accumVoltage); 

		display.setCursor(0, 40);
		display.print(F("Capacity          mAh"));
		display.setCursor(9*charWidth, 40);
		display.print(accumCapacity_AH * 1000);

		unsigned long time_s = dischargeTime_us / 1000000;

		display.setCursor(0, 55);
		display.print(F("Time"));

		printTime(time_s, 50, 55, charWidth);

		display.sendBuffer();
	}

}

void printTime(unsigned long time_s, char startXpos, char startYpos, char charWidth) {
	unsigned char sec = time_s % 60;
	time_s = time_s / 60;
	unsigned char min = time_s % 60;
	unsigned char hour = time_s / 60;

	char t[15];
	sprintf_P(t, PSTR("%3dh %02dm %02ds"), (int) hour, (int) min, (int) sec);
	display.setCursor(startXpos, startYpos);
	display.print(t);
}



