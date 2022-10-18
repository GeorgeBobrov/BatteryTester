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
constexpr byte pinUsbConnected = 12;

constexpr float internalReferenceVoltage = 1.1;
// Resistor divider for measure accum voltage
constexpr float resistorDividerKoef = (80500.0 + 20100.0) / 20100.0;

constexpr float currentShuntResistance = 1.02;

Encoder encoder(pinEnc1, pinEnc2, pinEncButton); 
U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

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
	TCCR1B = (TCCR1B & 0xF8) | 1;

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
	pinMode(pinUsbConnected, INPUT);

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
bool usbConnected;
bool prevUsbConnected;

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

enum class DisplayMode {
	main,
	setCurrent,
	off
};

enum SelectedActionOnMain {
	startDischarge,
	setCurrent,
	off
};

DisplayMode displayMode;
SelectedActionOnMain selectedActionOnMain;

void powerdownSleep() {
	displayMode = DisplayMode::off;
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

void restoreAfterSleep() {
	displayMode = DisplayMode::main;
	display.begin();
	if (usbConnected)
		Serial.begin(500000);
}

void startStopDischarge(bool l_enableDischarge) {
	enableDischarge = l_enableDischarge;
	digitalWrite(pinPwmSetCurrent, 0);
	digitalWrite(pinEnableDischarge, enableDischarge);

	if (enableDischarge) {
		dischargeTime_s = 0;
		accumCapacity_AH = 0;
		// tone(LED_BUILTIN, 2000, 500);
		// tone(LED_BUILTIN, 2, 2000);

		threshholdVoltage = 4.5;
		prevPwmValueForCurrent = 0;

		printTableCaption(F("***Start Discharge***"));
	} else
		printTableCaption(F("***Stop Discharge***"));				
}

void loop() {
	// Serial.print(micros()); 
	// Serial.println(F(" Start Loop"));

	encoder.tick();

	unsigned long curTime_us = micros();
	unsigned long periodFromLastMeasure_us = curTime_us - lastTimeMeasure_us;
	if (periodFromLastMeasure_us >= 50000) {
		lastTimeMeasure_us = curTime_us;

		usbConnected = digitalRead(pinUsbConnected);
		if (usbConnected != prevUsbConnected) {
			if (usbConnected) 
				Serial.begin(500000);
			else
				Serial.end();

			digitalWrite(LED_BUILTIN, usbConnected);
			prevUsbConnected = usbConnected;
		}

		if (encoder.isLeft()) {
			switch (displayMode) {
				case DisplayMode::main:
					selectedActionOnMain = SelectedActionOnMain( byte(selectedActionOnMain) + 1);
					if (selectedActionOnMain > SelectedActionOnMain::off) selectedActionOnMain = 0;
					break;
					
				case DisplayMode::setCurrent:
					settedDischargeCurrent += 0.05;
					prevPwmValueForCurrent = 0;
					break;

				case DisplayMode::off:	
					restoreAfterSleep();
					break;
			} 
		}	

		if (encoder.isRight()) {
			switch (displayMode) {
				case DisplayMode::main:
					selectedActionOnMain = SelectedActionOnMain( byte(selectedActionOnMain) - 1);
					if (selectedActionOnMain < 0) selectedActionOnMain = SelectedActionOnMain::off;
					break;

				case DisplayMode::setCurrent:
					settedDischargeCurrent -= 0.05; 
					prevPwmValueForCurrent = 0;
					break;

				case DisplayMode::off:	
					restoreAfterSleep();
					break;
			} 
		}	
 
		if (encoder.isClick()) {
			switch (displayMode) {
				case DisplayMode::main:
					switch (selectedActionOnMain) {
						case startDischarge:
							startStopDischarge(!enableDischarge);
							break;
						case setCurrent:
							displayMode = DisplayMode::setCurrent;
							break;
						case off:
							powerdownSleep();
							break;
					}
					break;

				case DisplayMode::setCurrent:
					displayMode = DisplayMode::main;
					break;
			} 
		}


		// int accumVoltageADC = analogRead(pinAnVoltage);
		// Averaging a number of measurements for more stable readings
		constexpr byte averCount = 5;
		int voltAdcSum = 0;
		int curAdcSum = 0;
		for (byte i = 0; i < averCount; i++) {
			voltAdcSum += analogRead(pinMeasureVoltage);
			curAdcSum += analogRead(pinMeasureCurrent);
		}
		float accumVoltageADC = float(voltAdcSum) / averCount;

		float accumVoltage = (accumVoltageADC / 1024.0) * resistorDividerKoef * internalReferenceVoltage;
		//Internal reference voltage in fact depends on supply voltage (this was measured and dependency identified)
		float referenceVoltageCorrected = internalReferenceVoltage - 0.006 * accumVoltage;
		accumVoltage = (accumVoltageADC / 1024.0) * resistorDividerKoef * referenceVoltageCorrected;

		float shuntVoltageADC = float(curAdcSum) / averCount;
		float shuntVoltage = (shuntVoltageADC / 1024.0) * referenceVoltageCorrected;
		float accumCurrent = shuntVoltage / currentShuntResistance;
		float loadCurrent = 0;

		if (enableDischarge) {
			// Subtract the voltage drop on the Schottky diode
			float microcontrollerVoltage = accumVoltage - 0.25;
			if (usbConnected) microcontrollerVoltage = 5.0 - 0.25;
			// Microcontroller and display consumption (was measured)
			float microcontrollerCurrent = 0.003 * microcontrollerVoltage;
			if (usbConnected)
				loadCurrent = accumCurrent;
			else
				loadCurrent = accumCurrent + microcontrollerCurrent;

			float accumCapacityPerPeriod_AH = loadCurrent * periodFromLastMeasure_us / us_in_hour;
			accumCapacity_AH += accumCapacityPerPeriod_AH;
			sumOfSeveralPeriods_us += periodFromLastMeasure_us;

			if (sumOfSeveralPeriods_us > 1000000) {
				dischargeTime_s += sumOfSeveralPeriods_us / 1000000;
				sumOfSeveralPeriods_us %= 1000000;
			}

			// Setting discharg current via PWM
			float pursuitShuntVoltage = settedDischargeCurrent * currentShuntResistance;
			word pwmValueForCurrent = (pursuitShuntVoltage / microcontrollerVoltage) * 255;
			pwmValueForCurrent = min(pwmValueForCurrent, 255);
			if ((pwmValueForCurrent > prevPwmValueForCurrent) ||
				(pwmValueForCurrent == 0) ||
				(pwmValueForCurrent < prevPwmValueForCurrent * 0.85)) {
				// if (usbConnected) {
				// 	Serial.print(F("pursuitShuntVoltage "));
				// 	Serial.println(pursuitShuntVoltage, 3); 

				// 	Serial.print(F("referenceVoltageCorrected "));
				// 	Serial.println(referenceVoltageCorrected, 3); 

				// 	Serial.print(F("settedDischargeCurrent "));
				// 	Serial.println(settedDischargeCurrent, 3); 

				// 	Serial.print(F("pwmValueForCurrent "));
				// 	Serial.println(pwmValueForCurrent); 

				// }

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
			startStopDischarge(false);

			Serial.flush();
			powerdownSleep();				

			// tone(LED_BUILTIN, 1, 2000);
		}


		if (displayMode != DisplayMode::off) {
			display.firstPage();
			// Serial.print(micros()); 
			// Serial.println(F(" Before Display Loop"));
			byte pageNum = 0;
			do {
				// Serial.print(micros()); 
				// Serial.println(F(" Before Display Paint"));

				display.setFont(u8g2_font_6x10_tf); 
				constexpr byte charWidth = 6;
				constexpr byte charHeight = 10;
				byte y = 10;

				switch (displayMode) {

					case DisplayMode::main:
						// Draw button
						display.setCursor(2, y);
						switch (selectedActionOnMain) {
							case startDischarge:
								if (enableDischarge)
									display.print(F("Stop Discharge"));
								else	 
									display.print(F("Start Discharge"));
								break;

							case setCurrent:
								display.print(F("Set Discharge Current")); break; 
							case off:
								display.print(F("Off")); break; 
						}	
						display.drawFrame(0, y-9, 127, 13);
						break; 

					case DisplayMode::setCurrent:
						display.setCursor(0, y);

						display.print(F("Current       A")); 

						display.setCursor(8*charWidth, y);
						display.print(settedDischargeCurrent, 3); 

						byte x = 127 - 3*charWidth - 2;
						display.setCursor(x, y);
						display.print(F("Set")); 

						display.drawFrame(x - 2, y-9, 3*charWidth + 4, 13);
						break;
				}

				// Offset of top yellow part of display (and minus baseline offset of font)
				y = 16 + charHeight - 3;
				display.setCursor(0, y);
				if (enableDischarge) {
					display.print(F("Discharge")); 

					display.setCursor(10*charWidth, y);
					display.print(F("Cur       A")); 
					display.setCursor(14*charWidth, y);
					display.print(loadCurrent, 3); 		
				}

				
				y += (charHeight + 2);
				display.setCursor(0, y);
				display.print(F("Voltage        V")); 
				display.setCursor(9*charWidth, y);
				display.print(accumVoltage); 

				if (selectedActionOnMain == setCurrent) {
					display.setCursor(127 - 3*charWidth, y);
					display.print(prevPwmValueForCurrent); 	
				}			

				y += (charHeight + 2);
				display.setCursor(0, y);
				display.print(F("Capacity          mAh"));
				display.setCursor(9*charWidth, y);
				display.print(accumCapacity_AH * 1000);

				unsigned long time_s = dischargeTime_s;

				y += (charHeight + 2);
				display.setCursor(0, y);
				display.print(F("Time"));

				printTime(time_s, 50, y, charWidth);

				encoder.tick();
				// Serial.print(micros()); 
				// Serial.println(F(" Before Display nextPage"));
				pageNum++;
			} while ( display.nextPage() );

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

		// Serial.print(micros()); 
		// Serial.println(F(" End Loop"));

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
	if (usbConnected) {
		Serial.print(F("Voltage\tCapacity_mAH\tTime_s\tCurrent\t"));
		Serial.println(msg);
	}
}

// void printTableCaption(const char * msg) {
// 	Serial.print(F("Voltage\tCapacity_mAH\tTime_s\tCurrent\t"));
// 	Serial.println(msg);
// }

void sendMeasurementsToSerial(float Voltage, float Capacity_AH, unsigned long dischargeTime_s, float Current) {
	if (usbConnected) {
		Serial.print(Voltage); 
		Serial.print(F("\t"));

		Serial.print(Capacity_AH * 1000); 
		Serial.print(F("\t"));

		Serial.print(dischargeTime_s); 
		Serial.print(F("\t"));

		Serial.println(Current, 3); 
	}
}



