#include "wiring_private.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <GyverPower.h>


constexpr byte pinEncButton = 6;
constexpr byte pinEnc1 = 3;
constexpr byte pinEnc2 = 2;
constexpr byte pinEnableDischarge = 4;
constexpr byte pinPwmSetCurrent = 9;
constexpr byte pinMeasureVoltage = A7;
constexpr byte pinMeasureCurrent = A1;

constexpr float internalReferenceVoltage = 1.1;
// Resistor divider for measure accum voltage
constexpr float resistorDividerKoef = (80500.0 + 20100.0) / 20100.0;

constexpr float currentShuntResistance = 1.02;

Encoder encoder(pinEnc1, pinEnc2, pinEncButton); 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void isrCLK() {
	encoder.tick();
}
							
void isrDT() {
	encoder.tick();
}

void setup() {
	//set board_build.f_cpu = 8000000L in platformio.ini and 
	//divide real freq 16000000 by 2 by SystemPrescaler (for working on low voltages)
	power.setSystemPrescaler(PRESCALER_2);

	//set Timer1 frequency to 31372 (set timer prescaler 1)
	TCCR1A = (TCCR1A & 0xF8) | 1;

	// Buzzer connected to LED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(pinEnableDischarge, OUTPUT);
	pinMode(pinPwmSetCurrent, OUTPUT);
//	pinMode(pinEncButton, INPUT_PULLUP);

	pinMode(5, INPUT_PULLUP);
	pinMode(7, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(10, INPUT_PULLUP);
	pinMode(11, INPUT_PULLUP);
	pinMode(12, INPUT_PULLUP);

	pinMode(A0, INPUT_PULLUP);
	pinMode(A2, INPUT_PULLUP);
	pinMode(A3, INPUT_PULLUP);
	pinMode(A6, INPUT_PULLUP);
		

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
bool enableSerial;
bool enableDischarge;
bool enableShowOnDisplay = true;
byte prevPwmValueForCurrent;

unsigned long dischargeTime_s; 
unsigned long sumOfSeveralPeriods_us; 
float accumCapacity_AH;
float threshholdVoltage; //Log capacity when reaching threshhold voltage
constexpr float threshholdDischargeVoltage = 3.0;
float settedDischargeCurrent = 0.1;

constexpr unsigned long us_in_hour = 60UL * 60 * 1000 * 1000;
void printTime(unsigned long time_s, byte startXpos, byte startYpos, byte charWidth);
void sendMeasurementsToSerial(float Voltage, float Capacity_AH, unsigned long dischargeTime_s, float Current);
// void printTableCaption(const char * msg);
void printTableCaption(const __FlashStringHelper *ifsh);

struct AccumCapacityRecord
{
	float Voltage;
	float Current;
	float Capacity_AH;
	unsigned long dischargeTime_s;
};

// need to to add "constexpr" in EEPROM.h in this line:
//    constexpr uint16_t length()                    { return E2END + 1; }
constexpr byte recordsInEEPROM = (EEPROM.length() / sizeof(AccumCapacityRecord)) - 1;
constexpr byte offssetOfRecords = 4;


void loop() {

	encoder.tick();

	unsigned long curTime_us = micros();
	unsigned long periodFromLastMeasure_us = curTime_us - lastTimeMeasure_us;
	if (periodFromLastMeasure_us >= 50000) {
		lastTimeMeasure_us = curTime_us;

		enableSerial = digitalRead(12);
		if (enableSerial) 
			Serial.begin(500000);
		else
			Serial.end();


		if (encoder.isLeft()) {
			enableShowOnDisplay = false;
			display.noDisplay();
			Serial.end();
			Wire.end();
			// pinMode(0, OUTPUT); // RX pin
			// pinMode(1, OUTPUT); // TX pin
			digitalWrite(0, 0); // Write 0 to reduce power consumption (Arduino nano schematic was modified)
			digitalWrite(1, 0);

			power.setSleepMode(POWERDOWN_SLEEP);
			power.sleep(SLEEP_FOREVER);
		}	

		if (encoder.isRight()) {
			enableShowOnDisplay = true;
			display.begin();
			Serial.begin(500000);
		}	
 
		if (encoder.isClick()) {

			enableDischarge = !enableDischarge;
			digitalWrite(pinPwmSetCurrent, 0);
			digitalWrite(pinEnableDischarge, enableDischarge);

			if (enableDischarge) {
				dischargeTime_s = 0;
				accumCapacity_AH = 0;
				// tone(LED_BUILTIN, 2000, 500);
				tone(LED_BUILTIN, 2, 2000);

				threshholdVoltage = 4.5;
				printTableCaption(F("***Start Discharge***"));
 			} else
				printTableCaption(F("***Stop Discharge***"));				

		}


		// int accumVoltageD = analogRead(pinAnVoltage);
		constexpr byte averCount = 5;
		int voltSum = 0;
		for (byte i = 0; i < averCount; i++)
			voltSum += analogRead(pinMeasureVoltage);
		float accumVoltageD = float(voltSum) / averCount;

		float accumVoltage = (accumVoltageD / 1024.0) * resistorDividerKoef * internalReferenceVoltage;
		//Internal reference voltage in fact depends on supply voltage (this was measured and dependency identified)
		float referenceVoltageCorrected = internalReferenceVoltage - 0.006 * accumVoltage;
		accumVoltage = (accumVoltageD / 1024.0) * resistorDividerKoef * referenceVoltageCorrected;

		float shuntVoltage = (analogRead(pinMeasureCurrent) / 1024.0) * referenceVoltageCorrected;
		float accumCurrent = shuntVoltage / currentShuntResistance;
		// Add 10 mA due to microcontroller and display consumption
		float loadCurrent = enableDischarge ? (accumCurrent + 0.01) : 0;

		if (enableDischarge) {
			float accumCapacityPerPeriod_AH = loadCurrent * periodFromLastMeasure_us / us_in_hour;
			accumCapacity_AH += accumCapacityPerPeriod_AH;
			sumOfSeveralPeriods_us += periodFromLastMeasure_us;

			if (sumOfSeveralPeriods_us > 1000000) {
				dischargeTime_s += sumOfSeveralPeriods_us / 1000000;
				sumOfSeveralPeriods_us %= 1000000;
			}
			// Setting discharg current via PWM
			// Subtract the voltage drop on the Schottky diode
			float microcontrollerVoltage = accumVoltage - 0.25;
			float pursuitShuntVoltage = settedDischargeCurrent * currentShuntResistance;
			byte pwmValueForCurrent = (pursuitShuntVoltage / microcontrollerVoltage) * 255;
			if (pwmValueForCurrent > prevPwmValueForCurrent) {
				analogWrite(pinPwmSetCurrent, pwmValueForCurrent);
				prevPwmValueForCurrent = pwmValueForCurrent;
			}
		}

		// Log to EEPROM capacity when reaching threshhold voltage
		if ((accumVoltage <= threshholdVoltage) && enableDischarge) { 
			AccumCapacityRecord capacityRecord;
			capacityRecord.Voltage = accumVoltage;
			capacityRecord.Current = loadCurrent;
			capacityRecord.Capacity_AH = accumCapacity_AH;
			capacityRecord.dischargeTime_s = dischargeTime_s;

			byte recordsHead = EEPROM[0];
			recordsHead = (recordsHead + 1) % recordsInEEPROM;

			EEPROM.put(offssetOfRecords + recordsHead * sizeof(AccumCapacityRecord), capacityRecord);
			EEPROM[0] = recordsHead;

			sendMeasurementsToSerial(accumVoltage, accumCapacity_AH, dischargeTime_s, loadCurrent);

			// Next log when accum discharges more by 0.1V
			threshholdVoltage = (round((accumVoltage - 0.1) * 10)) / 10.0;  
		}


		if ((accumVoltage <= threshholdDischargeVoltage) && enableDischarge) { // stopDischarge
			enableDischarge = false;
			digitalWrite(pinEnableDischarge, enableDischarge);
			printTableCaption(F("***End Discharge***"));				

			tone(LED_BUILTIN, 1, 2000);
		}


		if (enableShowOnDisplay) {
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
			Wire.flush();
		}

		if (Serial.available() > 0) {
			auto str = Serial.readString();
			if (str == F("meas"))
				enableSendMeasurementsToSerial = !enableSendMeasurementsToSerial;

			if (str == F("log")) {
				AccumCapacityRecord capacityRecord;

				byte recordsHead = EEPROM[0];

				printTableCaption(F("***Log***"));	

				for (byte i = recordsHead + 1; i < recordsInEEPROM; i++) {
					EEPROM.get(offssetOfRecords + i * sizeof(AccumCapacityRecord), capacityRecord);

					sendMeasurementsToSerial(capacityRecord.Voltage, capacityRecord.Capacity_AH, 
						capacityRecord.dischargeTime_s, capacityRecord.Current);
				}
				for (byte i = 0; i <= recordsHead; i++) {
					EEPROM.get(offssetOfRecords + i * sizeof(AccumCapacityRecord), capacityRecord);

					sendMeasurementsToSerial(capacityRecord.Voltage, capacityRecord.Capacity_AH, 
						capacityRecord.dischargeTime_s, capacityRecord.Current);
				}

				printTableCaption(F("***Log end***"));

			}

			if (str.startsWith(F("set current"))) {
				str.remove(0, 11);
				str.trim();
				settedDischargeCurrent = str.toFloat();
				if (settedDischargeCurrent == 0.0)
					settedDischargeCurrent = 0.1;
			}

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


void printTableCaption(const __FlashStringHelper *msg) {
	if (enableSerial) {
		Serial.print(F("Voltage\tCapacity_mAH\tTime_s\tCurrent\t"));
		Serial.println(msg);
	}
}

// void printTableCaption(const char * msg) {
// 	Serial.print(F("Voltage\tCapacity_mAH\tTime_s\tCurrent\t"));
// 	Serial.println(msg);
// }

void sendMeasurementsToSerial(float Voltage, float Capacity_AH, unsigned long dischargeTime_s, float Current) {
	if (enableSerial) {
		Serial.print(Voltage); 
		Serial.print(F("\t"));

		Serial.print(Capacity_AH * 1000); 
		Serial.print(F("\t"));

		Serial.print(dischargeTime_s); 
		Serial.print(F("\t"));

		Serial.println(Current, 3); 
	}
}



