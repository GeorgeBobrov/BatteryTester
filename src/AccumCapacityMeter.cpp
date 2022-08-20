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
	pinMode(pinDischarge, OUTPUT);
//	pinMode(pinEncButton, INPUT_PULLUP);

	analogReference(INTERNAL);

	Serial.begin(500000);
	encoder.setType(TYPE2);

	// mode = EEPROM.read(0);

	display.begin();


	attachInterrupt(0, isrCLK, CHANGE);
	attachInterrupt(1, isrDT, CHANGE);
}

unsigned long lastTimeMeasure_us = 0;
unsigned long lastTimeSerial_us = 0;
bool enableSendMeasurementsToSerial;
bool enableDischarge;

unsigned long dischargeTime_s; 
unsigned long sumOfSeveralPeriods_us; 
float accumCapacity_AH;
float threshholdVoltage; //Log capacity when reaching threshhold voltage
constexpr float threshholdDischargeVoltage = 3.0;

constexpr float us_in_hour = 60.0 * 60.0 * 1000.0 * 1000.0;
void printTime(unsigned long time_s, byte startXpos, byte startYpos, byte charWidth);
void sendMeasurementsToSerial(float Voltage, float Capacity_AH, unsigned long dischargeTime_s, float Current);

struct AccumCapacityRecord
{
	float Voltage;
	float Current;
	float Capacity_AH;
	unsigned long dischargeTime_s;
};

constexpr byte recordsInEEPROM = (EEPROM.length() / sizeof(AccumCapacityRecord)) - 1;
constexpr byte offssetOfRecords = 4;


void loop() {

	encoder.tick();

	unsigned long curTime_us = micros();
	unsigned long periodFromLastMeasure_us = curTime_us - lastTimeMeasure_us;
	if (periodFromLastMeasure_us >= 50000) {
		lastTimeMeasure_us = curTime_us;

 
		if (encoder.isClick()){

			enableDischarge = !enableDischarge;
			digitalWrite(pinDischarge, enableDischarge);

			if (enableDischarge) {
				dischargeTime_s = 0;
				accumCapacity_AH = 0;
				// tone(LED_BUILTIN, 2000, 500);
				tone(LED_BUILTIN, 2, 2000);

				threshholdVoltage = 4.2;
 			}

			// delay(500);	 
		}


		// int accumVoltageD = analogRead(pinAnVoltage);
		constexpr byte averCount = 3;
		int voltSum = 0;
		for (byte i = 0; i < averCount; i++)
			voltSum += analogRead(pinAnVoltage);
		int accumVoltageD = round(float(voltSum) / averCount);
		  
		float accumVoltage = (accumVoltageD / 1024.0) * voltageKoef;
		float loadCurrent = enableDischarge ? (accumVoltage / loadResistor) : 0;

		if (enableDischarge) {
			float accumCapacityPerPeriod_AH = loadCurrent * periodFromLastMeasure_us / us_in_hour;
			accumCapacity_AH += accumCapacityPerPeriod_AH;
			sumOfSeveralPeriods_us += periodFromLastMeasure_us;

			if (sumOfSeveralPeriods_us > 1000000) {
				dischargeTime_s += sumOfSeveralPeriods_us / 1000000;
				sumOfSeveralPeriods_us %= 1000000;
			}
		}

		// Log to EEPROM capacity when reaching threshhold voltage
		if ((accumVoltage <= threshholdVoltage) && enableDischarge) { 
			// AccumCapacityRecord capacityRecord;
			// capacityRecord.Voltage = accumVoltage;
			// capacityRecord.Current = loadCurrent;
			// capacityRecord.Capacity_AH = accumCapacity_AH;
			// capacityRecord.dischargeTime_s = dischargeTime_s;

			// byte recordsHead = EEPROM[0];
			// recordsHead = (recordsHead + 1) % recordsInEEPROM;

			// EEPROM.put(offssetOfRecords + recordsHead * sizeof(AccumCapacityRecord), capacityRecord);

			sendMeasurementsToSerial(accumVoltage, accumCapacity_AH, dischargeTime_s, loadCurrent);

			// Next log when accum discharges more by 0.1V
			threshholdVoltage = (round((accumVoltage - 0.1) * 10)) / 10.0;  
		}


		if ((accumVoltage <= threshholdDischargeVoltage) && enableDischarge) { // stopDischarge
			enableDischarge = false;
			digitalWrite(pinDischarge, enableDischarge);
			tone(LED_BUILTIN, 1, 2000);
		}



		display.clearBuffer(); 
		display.setFont(u8g2_font_6x10_tf); 
		constexpr byte charWidth = 6;

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

		unsigned long time_s = dischargeTime_s;

		display.setCursor(0, 55);
		display.print(F("Time"));

		printTime(time_s, 50, 55, charWidth);

		display.sendBuffer();


		if (Serial.available() > 0) {
			auto str = Serial.readString();
			if (str == F("meas"))
				enableSendMeasurementsToSerial = !enableSendMeasurementsToSerial;
		}

		unsigned long periodFromLastSerial_us = curTime_us - lastTimeSerial_us;
		if ((periodFromLastSerial_us >= 1000000) && enableSendMeasurementsToSerial) {
			lastTimeSerial_us = curTime_us;
			sendMeasurementsToSerial(accumVoltage, accumCapacity_AH, dischargeTime_s, loadCurrent);
		}
	}

}

void printTime(unsigned long time_s, byte startXpos, byte startYpos, byte charWidth) {
	byte sec = time_s % 60;
	time_s = time_s / 60;
	byte min = time_s % 60;
	byte hour = time_s / 60;

	char t[15];
	sprintf_P(t, PSTR("%3dh %02dm %02ds"), hour, min, sec);
	display.setCursor(startXpos, startYpos);
	display.print(t);
}

// void logToEEPROMcapacity()

void sendMeasurementsToSerial(float Voltage, float Capacity_AH, unsigned long dischargeTime_s, float Current) {
	Serial.print(F("Voltage\t"));
	Serial.print(Voltage); 

	Serial.print(F("\tCapacity_mAH\t"));
	Serial.print(Capacity_AH * 1000); 

	Serial.print(F("\tTime_s\t"));
	Serial.print(dischargeTime_s); 

	Serial.print(F("\tCurrent\t"));
	Serial.println(Current, 3); 

}



