#include "wiring_private.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <GyverPower.h>
#include <GyverPWM.h>

constexpr byte pinEncButton = 6;
constexpr byte pinEnc1 = 2;
constexpr byte pinEnc2 = 3;
constexpr byte pinEnableDischarge = 4;
constexpr byte pinPwmSetCurrent = 9;
constexpr byte pinMeasureVoltage = A7;
constexpr byte pinMeasureCurrent = A6;
constexpr byte pinUsbConnected = 12;

constexpr float internalReferenceVoltage = 1.1;
// Resistor divider for measure accum voltage
constexpr float Rup = 80500.0;
constexpr float Rdown = 20100.0;
constexpr float resistorDividerCoef = (Rup + Rdown) / Rdown;

constexpr float currentShuntResistance = 0.99;
float stopDischargeVoltage = 3.0;
float wantedDischargeCurrent = 0.1; // Initial value 
constexpr float maxDischargeCurrent = 1; 
constexpr float maxVoltage = 5.0; 

constexpr int16_t MAX_ADC_VALUE = bit(10) - 1;
// Use GyverPWM library, 10 bit PWM (PWM_16KHZ_D9 function)
constexpr int16_t MAX_PWM_VALUE = bit(10) - 1;


Encoder encoder(pinEnc1, pinEnc2, pinEncButton); 
U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
constexpr byte displayWidth = 128;
constexpr byte displayHeight = 64;

// Variables created by the build process
// when the sketch is compiled
extern int __bss_end;
extern void *__brkval;
 
// Function that returns the amount of free RAM
int getSizeFreeRam()
{
   int freeValue;
   if((int)__brkval == 0)
      freeValue = ((int)&freeValue) - ((int)&__bss_end);
   else
      freeValue = ((int)&freeValue) - ((int)__brkval);
   return freeValue;
}

void isrCLK() {
	encoder.tick();
}
							
void isrDT() {
	encoder.tick();
}

unsigned long timeShowDescription;

void setup() {
	//set board_build.f_cpu = 8000000L in platformio.ini and 
	//divide real freq 16000000 by 2 by SystemPrescaler (for working on low voltages)
	power.setSystemPrescaler(PRESCALER_2);

	//set Timer1 frequency to 31372 (set timer prescaler 1)
	// TCCR1B = (TCCR1B & 0xF8) | 1; //No need, using GyverPWM

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
	pinMode(A1, INPUT_PULLUP);
	pinMode(A2, INPUT_PULLUP);
	pinMode(A3, INPUT_PULLUP);
		

	analogReference(INTERNAL);

	Serial.begin(500000);
	encoder.setType(TYPE2);

	// mode = EEPROM.read(0);

	display.begin();
	display.setFontPosBottom(); // to eliminate correction y by fontBaseline when printing text on display every time

	timeShowDescription = micros() + 3000000;

	attachInterrupt(0, isrCLK, CHANGE);
	attachInterrupt(1, isrDT, CHANGE);
}

unsigned long lastTimeMeasure_us = 0;
unsigned long lastTimeSerial_us = 0;
bool enableSendMeasurementsToSerial;
bool enableSendOscillogramToSerial;
bool usbConnected;
bool prevUsbConnected;

bool enableDischarge;
bool enableShowOnDisplay = true;
uint16_t prevPwmValueForCurrent;

unsigned long dischargeTime_s; 
unsigned long sumOfSeveralPeriods_us; 
float accumCapacity_AH;
float lastSavedAccumCapacity_AH;
float threshholdVoltage; //Log capacity when reaching threshhold voltage

constexpr unsigned long us_in_hour = 60UL * 60 * 1000 * 1000;
void printTime(unsigned long time_s, byte startXpos, byte startYpos);
void sendMeasurementsToSerial(float &Voltage, float &Capacity_AH, unsigned long &dischargeTime_s, float &Current);
void sendOscillogramToSerial(float &Time_s, float &Voltage, float &Current, float &Capacity_AH);
// void printTableCaption(const char * msg);
void printTableCaption(const __FlashStringHelper *ifsh);
void drawLog(byte shiftFromLastRec, bool drawHeader = true);
void checkCommandsFromSerial();
// void sendTimeDebugInfoToSerial(const __FlashStringHelper *msg, byte marker = 0);

struct AccumCapacityRecord
{
	float Voltage;
	float Current;
	float Capacity_AH;
	unsigned long dischargeTime_s;
};
void saveRecordToEEPROM(AccumCapacityRecord &capacityRecord, bool rewriteLastRecord = false);

// need to to add "constexpr" in EEPROM.h in this line:
//    constexpr uint16_t length()                    { return E2END + 1; }
constexpr byte numOfRecordsInEEPROM = (EEPROM.length() / sizeof(AccumCapacityRecord)) - 1;
constexpr byte offssetOfLogRecordsInEEPROM = 4; // for storing indexOfLastRecordInEEPROM

constexpr byte charWidth = 6;
constexpr byte charHeight = 10;
constexpr byte yellowHeaderHeight = 16;
constexpr byte tableHeaderHeight = yellowHeaderHeight;
constexpr byte tableHeight = displayHeight - tableHeaderHeight;
// 64 (displayHeight) - 16 (tableHeaderHeight) = 48
// 48 / charHeight (10) = 4.8, round up to 5 
constexpr byte linesOfLogOnScreen = round(float(tableHeight) / charHeight); 
constexpr byte additionalSpacingForFirstLine = 2;

enum class DisplayMode: byte {
	main,
	setCurrent,
	setDischargeVoltage,
	// measureRin,
	viewLog,
	off
};

enum class ClickAction: byte {
	startDischarge,
	setCurrent,
	setDischargeVoltage,
	// measureRin,
	viewLog,
	off
};

DisplayMode displayMode;
ClickAction clickAction;
bool debugMode;
bool showFreeRAM;
// For scrolling log
byte shiftFromLastRec; 
unsigned long timeDischargeStarted;
unsigned long timeDischargeStopped;
bool rewriteLastRecord;
unsigned long timeOscillogramStarted_us;
uint16_t oscilPeriod_ms;
float accumVoltage;

enum class MeasureRinStatus: byte {
	notMeasured,
	measuredVoltageStart,
	measuredVoltageEnd
};

struct MeasureRin {
	float voltageStart, voltageEnd, current;  
	MeasureRinStatus status;
} measureRin;

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

		threshholdVoltage = maxVoltage;
		prevPwmValueForCurrent = 0;
		measureRin.status = MeasureRinStatus::notMeasured;

		printTableCaption(F("***Start Discharge***"));
	} else
		printTableCaption(F("***Stop Discharge***"));				
}

void loop() {
	// sendTimeDebugInfoToSerial(F(" Start_Loop "), 1);
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

		switch (displayMode) {
			case DisplayMode::main:
				if (encoder.isRight()) {
					if (clickAction == ClickAction::off)
						clickAction = ClickAction(0);
					else	
						clickAction = ClickAction( byte(clickAction) + 1);
					 
					timeShowDescription = curTime_us + 1000000;
					if (clickAction == ClickAction::startDischarge)
						timeShowDescription = curTime_us;
				}

				if (encoder.isLeft()) {
					if (clickAction == ClickAction(0)) 
						clickAction = ClickAction::off;
					else	
						clickAction = ClickAction( byte(clickAction) - 1);
					
					timeShowDescription = curTime_us + 1000000;
					if (clickAction == ClickAction::startDischarge)
						timeShowDescription = curTime_us;
				}

				if (encoder.isClick()) {
					switch (clickAction) {
						case ClickAction::startDischarge:
							if (!enableDischarge) // going to start discharge
								if (accumVoltage < stopDischargeVoltage) {
									displayMode = DisplayMode::setDischargeVoltage;
									break;
								}	
						
							startStopDischarge(!enableDischarge);
							if (enableDischarge)
								timeDischargeStarted = curTime_us;
						break;
						case ClickAction::setCurrent:
							displayMode = DisplayMode::setCurrent;
						break;
						case ClickAction::setDischargeVoltage:
							displayMode = DisplayMode::setDischargeVoltage;
						break;
						// case ClickAction::measureRin:
						// 	displayMode = DisplayMode::measureRin;
						// 	break;
						case ClickAction::viewLog:
							displayMode = DisplayMode::viewLog;
						break;
						case ClickAction::off:
							powerdownSleep();
						break;
					}
				}
			break;
				
			case DisplayMode::setCurrent:
				if (encoder.isRight()) {
					wantedDischargeCurrent += 0.05;
					if (wantedDischargeCurrent > maxDischargeCurrent) 
						wantedDischargeCurrent = maxDischargeCurrent;
					prevPwmValueForCurrent = 0;
				}
				if (encoder.isLeft()) {
					wantedDischargeCurrent -= 0.05; 
					if (wantedDischargeCurrent < 0) wantedDischargeCurrent = 0;
					prevPwmValueForCurrent = 0;
				}
				if (encoder.isClick())
					displayMode = DisplayMode::main;
			break;
				
			case DisplayMode::setDischargeVoltage:
				if (encoder.isRight()) {
					stopDischargeVoltage += 0.1;
					if (stopDischargeVoltage > maxVoltage) 
						stopDischargeVoltage = maxVoltage;
					prevPwmValueForCurrent = 0;
				}
				if (encoder.isLeft()) {
					stopDischargeVoltage -= 0.1; 
					// accumVoltage can't reach 0V since schottky diode reverse current leakage
					// (bad feature of the electrical circuit)
					if (stopDischargeVoltage < 0.1) stopDischargeVoltage = 0.1;
					prevPwmValueForCurrent = 0;
				}
				if (encoder.isClick())
					displayMode = DisplayMode::main;
			break;

			case DisplayMode::viewLog:
				if (encoder.isLeft()) {
					if (shiftFromLastRec < numOfRecordsInEEPROM - linesOfLogOnScreen) 
						shiftFromLastRec++;
				}
				if (encoder.isRight()) {
					if (shiftFromLastRec > 0)
						shiftFromLastRec--; 
				}
				if (encoder.isClick()) {
					displayMode = DisplayMode::main;
					shiftFromLastRec = 0;
				}
			break;

			case DisplayMode::off:	
				if (encoder.isLeft())
					restoreAfterSleep();
				if (encoder.isRight())
					restoreAfterSleep();
			break;
		} 
		if ((clickAction == ClickAction::setCurrent) && encoder.isHolded())
			debugMode = !debugMode;


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

		constexpr float UsbVoltage = 5.0;
		accumVoltage = (accumVoltageADC / MAX_ADC_VALUE) * resistorDividerCoef * internalReferenceVoltage;
		float microcontrollerVoltage = accumVoltage;

		// Internal reference voltage in fact depends on supply voltage (this was measured and dependency identified)
		float referenceVoltageCorrected = internalReferenceVoltage - 0.006 * microcontrollerVoltage;
		if (usbConnected) referenceVoltageCorrected = 1.08;

		// Repeat calculations with corrected reference voltage value
		float accumVoltageDivided = (accumVoltageADC / MAX_ADC_VALUE) * referenceVoltageCorrected;
		accumVoltage = accumVoltageDivided * resistorDividerCoef;
		microcontrollerVoltage = accumVoltage;
		if (usbConnected) microcontrollerVoltage = UsbVoltage;

		float shuntVoltageADC = float(curAdcSum) / averCount;
		float shuntVoltage = (shuntVoltageADC / MAX_ADC_VALUE) * referenceVoltageCorrected;
		float accumCurrent = shuntVoltage / currentShuntResistance;
		float loadCurrent = accumCurrent;
		// Microcontroller and display consumption (this was measured and dependency identified)
		float microcontrollerCurrent = 0.003 * microcontrollerVoltage;
		if (!usbConnected) 
			loadCurrent += microcontrollerCurrent;

		// Save log just a secont after discharge started for internal resistance measurement
		if ((timeDischargeStarted > 0) && (curTime_us - timeDischargeStarted >= 3000000)) {
			timeDischargeStarted = 0;

			AccumCapacityRecord capacityRecord{.Voltage = accumVoltage, .Current = loadCurrent, 
				.Capacity_AH = accumCapacity_AH, .dischargeTime_s = dischargeTime_s};

			saveRecordToEEPROM(capacityRecord);

			if (measureRin.status == MeasureRinStatus::measuredVoltageStart) {
				measureRin.voltageEnd = accumVoltage;
				measureRin.current = loadCurrent;
				measureRin.status = MeasureRinStatus::measuredVoltageEnd;
			}
		}		

		if (enableDischarge) {
			float wantedDischargeCurrentCorrected = wantedDischargeCurrent;
			// If USB connected, microcontroller and display consump current from USB
			// if powered from accumulator, we need to take microcontroller current into account	
			if (!usbConnected) {
				wantedDischargeCurrentCorrected -= microcontrollerCurrent;
				if (wantedDischargeCurrentCorrected < 0) wantedDischargeCurrentCorrected = 0;
			}

			float accumCapacityPerPeriod_AH = loadCurrent * periodFromLastMeasure_us / us_in_hour;
			accumCapacity_AH += accumCapacityPerPeriod_AH;
			sumOfSeveralPeriods_us += periodFromLastMeasure_us;

			if (sumOfSeveralPeriods_us > 1000000) {
				dischargeTime_s += sumOfSeveralPeriods_us / 1000000;
				sumOfSeveralPeriods_us %= 1000000;
			}

			// Setting discharg current via PWM
			float wantedShuntVoltage = wantedDischargeCurrentCorrected * currentShuntResistance;
			// The red LED used as a stabilitron to limit PWM amplitude voltage (see schematic)
			constexpr float redLedVoltage = 1.61;
			// Since the charge and discharge resistances are not equal, a correction is needed 
			constexpr float PwmCorrectionCoef = 1.04;
			constexpr float PwmStepInV = (redLedVoltage / MAX_PWM_VALUE) * PwmCorrectionCoef;
			uint16_t pwmValueForCurrent = wantedShuntVoltage / PwmStepInV;
			pwmValueForCurrent = min(pwmValueForCurrent, MAX_PWM_VALUE);
			if ((pwmValueForCurrent > prevPwmValueForCurrent * 1.15) ||
				(pwmValueForCurrent == 0) ||
				(pwmValueForCurrent < prevPwmValueForCurrent * 0.85)) {

				// analogWrite(pinPwmSetCurrent, pwmValueForCurrent);
				PWM_16KHZ_D9(pwmValueForCurrent);
				prevPwmValueForCurrent = pwmValueForCurrent;
			}
		}

		// Log to EEPROM capacity when reaching threshhold voltage
		if ((accumVoltage <= threshholdVoltage) && enableDischarge) { 
			// // Stop timer to prevent excessive log saving
			// if ((timeDischargeStarted > 0) && (curTime_us - timeDischargeStarted < 1000000)) 
			// 	timeDischargeStarted = 0;

			AccumCapacityRecord capacityRecord{.Voltage = accumVoltage, .Current = loadCurrent, 
				.Capacity_AH = accumCapacity_AH, .dischargeTime_s = dischargeTime_s};

			saveRecordToEEPROM(capacityRecord, rewriteLastRecord);
			rewriteLastRecord = false;

			sendMeasurementsToSerial(accumVoltage, accumCapacity_AH, dischargeTime_s, loadCurrent);

			if (measureRin.status == MeasureRinStatus::notMeasured) {
				measureRin.voltageStart = accumVoltage;
				measureRin.status = MeasureRinStatus::measuredVoltageStart;
			}

			// Next log when accum discharges more by 0.1V
			threshholdVoltage = (round((accumVoltage - 0.1) * 10)) / 10.0;  
			lastSavedAccumCapacity_AH = accumCapacity_AH;
		}

		// Accumulator may suddenly turn off on low voltages (due to protect sircuit), so
		// save AccumCapacityRecord periodically
		if (!usbConnected && (threshholdVoltage == stopDischargeVoltage) && enableDischarge) 
			if (accumCapacity_AH > lastSavedAccumCapacity_AH * 1.01) {
				AccumCapacityRecord capacityRecord{.Voltage = accumVoltage, .Current = loadCurrent, 
					.Capacity_AH = accumCapacity_AH, .dischargeTime_s = dischargeTime_s};

				saveRecordToEEPROM(capacityRecord, rewriteLastRecord);			
				rewriteLastRecord = true;

				lastSavedAccumCapacity_AH = accumCapacity_AH;
			}


		if ((accumVoltage <= stopDischargeVoltage) && enableDischarge) { // stopDischarge
			startStopDischarge(false);

			// Wait a second after discharge stopped, save log and then fall to sleep
			timeDischargeStopped = curTime_us;

			// tone(LED_BUILTIN, 1, 2000);
		}

		if ((timeDischargeStopped > 0) && (curTime_us - timeDischargeStopped >= 3000000)) {
			timeDischargeStopped = 0;

			AccumCapacityRecord capacityRecord{.Voltage = accumVoltage, .Current = loadCurrent, 
				.Capacity_AH = accumCapacity_AH, .dischargeTime_s = dischargeTime_s};

			saveRecordToEEPROM(capacityRecord);

			if (!(usbConnected && enableSendOscillogramToSerial)) {
				Serial.flush();
				powerdownSleep();
			}
		}

		if (displayMode != DisplayMode::off) {
			display.firstPage();
			// sendTimeDebugInfoToSerial(F(" Before_Display_Loop "));

			byte pageNum = 0;
			do {
				// sendTimeDebugInfoToSerial(F(" Before_Display_Paint "));
				encoder.tick();
				if (encoder.isTurn()) break;

				display.setFont(u8g2_font_6x10_tf); 
				byte y = charHeight + additionalSpacingForFirstLine;
				byte x = 0;


				if (displayMode == DisplayMode::main) {
					x = 2;
					display.setCursor(x, y);

					if (clickAction == ClickAction::startDischarge) {
						bool showDescription = (curTime_us < timeShowDescription);
						if (showDescription)
							display.print(F("Battery tester"));
						else {
							if (enableDischarge)
								display.print(F("Stop Discharge"));
							else	 
								display.print(F("Start Discharge"));

							// Draw button frame
							display.drawFrame(x - 2, y - charHeight - 1, displayWidth, charHeight + 3);
						}	
					}

					if (clickAction == ClickAction::viewLog) {
						bool showDescription = (curTime_us < timeShowDescription);
						if (showDescription)
							display.print(F("Log"));
						drawLog(0, !showDescription); 
						display.drawFrame(x - 2, y - charHeight - 1, displayWidth, charHeight + 3);

					}
						
					if (clickAction == ClickAction::off) {
						display.print(F("Sleep")); 
						display.drawFrame(x - 2, y - charHeight - 1, displayWidth, charHeight + 3);
					}
				} 
				
				if (displayMode == DisplayMode::viewLog)
					drawLog(shiftFromLastRec); 

				if ((displayMode == DisplayMode::setCurrent) || 
					((displayMode == DisplayMode::main) && (clickAction == ClickAction::setCurrent))) {
					display.setCursor(0, y);

					bool showDescription = (curTime_us < timeShowDescription);
					if ((displayMode == DisplayMode::main) && showDescription)
						display.print(F("Discharge Current"));
					else {	
						display.print(F("Dsc Cur:       A")); 

						display.setCursor(9*charWidth, y);
						display.print(wantedDischargeCurrent, 3); 
					}

					x = displayWidth - 3*charWidth - 3;
					display.setCursor(x, y);
					if (displayMode == DisplayMode::setCurrent)
						display.print(F("OK")); 
					else	
						display.print(F("Set")); 

					display.drawFrame(x - 2, y - charHeight - 1, 3*charWidth + 4, charHeight + 3);
				}

				if ((displayMode == DisplayMode::setDischargeVoltage) || 
					((displayMode == DisplayMode::main) && (clickAction == ClickAction::setDischargeVoltage))) {
					display.setCursor(0, y);

					bool showDescription = (curTime_us < timeShowDescription);
					if ((displayMode == DisplayMode::main) && showDescription)
						display.print(F("Stop Voltage"));
					else {	
						display.print(F("Stop Volt:      V")); 

						display.setCursor(11*charWidth, y);
						display.print(stopDischargeVoltage); 
					}

					x = displayWidth - 3*charWidth - 3;
					display.setCursor(x, y);
					if (displayMode == DisplayMode::setDischargeVoltage)
						display.print(F("OK")); 
					else	
						display.print(F("Set")); 

					display.drawFrame(x - 2, y - charHeight - 1, 3*charWidth + 4, charHeight + 3);
				}

				if (clickAction != ClickAction::viewLog) {
					// Offset of top yellow part of display
					y = yellowHeaderHeight + charHeight;
					x = 0;

					byte y_line = y - charHeight;
					byte y2 = y + (charHeight + /*spacingBetweenLines*/2);

					display.setCursor(x, y);
					// display.print(F("FreeRAM")); 
					display.print(F("Voltage")); 

					display.setCursor(x, y2);
					display.print(accumVoltage); 
					// display.print(getSizeFreeRam()); 
					
					display.setCursor(x + 6*charWidth, y2);
					display.print(F("V")); 

					display.drawVLine(x + 7.7*charWidth, y_line, 2*charHeight);

					x = 8.5*charWidth;
					if (enableDischarge) {
						// display.print(F("Discharge")); 

						display.setCursor(x, y);
						display.print(F("Current")); 

						display.setCursor(x, y2);
						display.print(loadCurrent, 3); 

						display.setCursor(x + 6*charWidth, y2);
						display.print(F("A")); 
					}
					display.drawVLine(x + 7.7*charWidth, y_line, 2*charHeight);


					if (measureRin.status == MeasureRinStatus::measuredVoltageEnd) {
						float Rin = (measureRin.voltageStart - measureRin.voltageEnd) / measureRin.current;

						x = 17*charWidth;
						display.setCursor(x, y);
						display.print(F("Rin")); 

						display.setCursor(x, y2);
						display.print(Rin); 
					}

					y = y2;
					display.drawHLine(0, y + 1, displayWidth);

					y += (charHeight + /*spacingBetweenLines*/3 + /*HLine*/1);

					if ((clickAction == ClickAction::setCurrent) && debugMode) {
						char t[21];
						sprintf_P(t, PSTR("C %dmV %d+%dmA %dref"), int(shuntVoltage * 1000), int(accumCurrent * 1000), 
							int(microcontrollerCurrent * 1000), int(referenceVoltageCorrected * 1000));
						display.setCursor(0, y);
						display.print(t);
						
						sprintf_P(t, PSTR("V %dmV PWM %d"), int(accumVoltageDivided * 1000), prevPwmValueForCurrent);
						y += (charHeight + /*spacingBetweenLines*/3);
						display.setCursor(0, y);
						display.print(t);
					} else {
						display.setCursor(0, y);
						display.print(F("Capacity          mAh"));

						display.setCursor(9*charWidth, y);
						display.print(accumCapacity_AH * 1000);

						display.drawHLine(0, y + 1, displayWidth);

						unsigned long time_s = dischargeTime_s;

						y += (charHeight + /*spacingBetweenLines*/3 + /*HLine*/1);
						display.setCursor(0, y);
						display.print(F("Time"));
						printTime(time_s, 50, y);
					}
				}

				if (showFreeRAM) {
					x = displayWidth - 4*charWidth - 3;
					y = displayHeight + 2;

					display.setDrawColor(0);
					display.drawBox(x, displayHeight - charHeight, 4*charWidth, charHeight);
					display.setDrawColor(1);

					display.setCursor(x, y);
					display.print(getSizeFreeRam()); 
				}
				
				encoder.tick();
				if (encoder.isTurn()) break;
				// sendTimeDebugInfoToSerial(F(" Before_Display_nextPage "));

				pageNum++;
			} while ( display.nextPage() );

		}

		checkCommandsFromSerial();

		unsigned long periodFromLastSerial_us = curTime_us - lastTimeSerial_us;
		if ((periodFromLastSerial_us >= 1000000) && enableSendMeasurementsToSerial && !enableSendOscillogramToSerial) {
			lastTimeSerial_us = curTime_us;
			sendMeasurementsToSerial(accumVoltage, accumCapacity_AH, dischargeTime_s, loadCurrent);
		}

		if (enableSendOscillogramToSerial) {
			uint16_t periodFromLastOscil_ms = (curTime_us - lastTimeSerial_us) / 1000;
			if (periodFromLastOscil_ms >= oscilPeriod_ms) {
				lastTimeSerial_us = curTime_us;
				float timeFromOscillogramStarted_s = (curTime_us - timeOscillogramStarted_us)  / 1000000.0;
				if (timeFromOscillogramStarted_s < 0) timeFromOscillogramStarted_s = 0; 
				sendOscillogramToSerial(timeFromOscillogramStarted_s, accumVoltage, loadCurrent, accumCapacity_AH);
			}
		}
		// sendTimeDebugInfoToSerial(F(" End_Loop ")); 

	}

}

void printTime(unsigned long time_s, byte startXpos, byte startYpos) {
	byte sec = time_s % 60;
	time_s = time_s / 60;
	byte min = time_s % 60;
	byte hour = time_s / 60;

	char t[16];
	sprintf_P(t, PSTR("%3dh %02dm %02ds"), hour, min, sec);
	display.setCursor(startXpos, startYpos);
	display.print(t);
}

void drawOneLogLine(byte lineNum, AccumCapacityRecord &capacityRecord) {
	byte y = yellowHeaderHeight + (charHeight * lineNum);
	display.setCursor(0, y);
	// 0.00 (4 chars) 
	display.print(capacityRecord.Voltage); 

	display.setCursor(5*charWidth, y);
	// 0000 (4 chars max) 
	display.print(capacityRecord.Capacity_AH * 1000, 0); 

	unsigned long mins = capacityRecord.dischargeTime_s / 60;
	byte min = mins % 60;
	byte hour = mins / 60;
	char t[16];
	sprintf_P(t, PSTR("%3dh%02dm"), hour, min);
	display.setCursor(10*charWidth, y);
	display.print(t);

	display.setCursor(18*charWidth, y);
	display.print(capacityRecord.Current * 1000, 0);

	display.drawHLine(0, y - 1, displayWidth);
}

void drawLog(byte shiftFromLastRec, bool drawHeader) {
	//add scroll bar
	byte y = charHeight + additionalSpacingForFirstLine;

	if (drawHeader) {
		display.setCursor(2, y);
		display.print(F("Volt")); 

		display.setCursor(5*charWidth, y);
		display.print(F("mAH")); 

		display.setCursor(10*charWidth, y);
		display.print(F("Time")); 

		display.setCursor(18*charWidth, y);
		display.print(F("mA")); 	
	}

	display.drawHLine(0, y + 2, displayWidth);

	display.drawVLine(4*charWidth + 3, 0, displayHeight);
	display.drawVLine(9*charWidth + 3, 0, displayHeight);
	display.drawVLine(17*charWidth + 3, 0, displayHeight);

	byte indexOfLastRecordInEEPROM = EEPROM[0];

	for (byte lineIndex = 1; lineIndex <= linesOfLogOnScreen; lineIndex++) {
		byte ri = (lineIndex + indexOfLastRecordInEEPROM + numOfRecordsInEEPROM - shiftFromLastRec - linesOfLogOnScreen) % numOfRecordsInEEPROM;

		AccumCapacityRecord capacityRecord;
		EEPROM.get(offssetOfLogRecordsInEEPROM + ri * sizeof(AccumCapacityRecord), capacityRecord);

		drawOneLogLine(lineIndex, capacityRecord);
	}

	float yEndScrollMarker = (float(numOfRecordsInEEPROM - shiftFromLastRec) / numOfRecordsInEEPROM) * tableHeight;
	float yStartScrollMarker = (float(numOfRecordsInEEPROM - shiftFromLastRec - linesOfLogOnScreen) / numOfRecordsInEEPROM) * tableHeight;

	byte yScrollMarker = tableHeaderHeight + round(yStartScrollMarker); 
	byte scrollMarkerHeight = round(yEndScrollMarker) - round(yStartScrollMarker); 

	display.drawVLine(displayWidth - 1, yScrollMarker, scrollMarkerHeight);
}


void printTableCaption(const __FlashStringHelper *msg) {
	if (usbConnected && !enableSendOscillogramToSerial) {
		Serial.print(F("Voltage\tCapacity_mAH\tTime_s\tCurrent\t"));
		Serial.println(msg);
	}
}

// void printTableCaption(const char * msg) {
// 	Serial.print(F("Voltage\tCapacity_mAH\tTime_s\tCurrent\t"));
// 	Serial.println(msg);
// }

void sendMeasurementsToSerial(float &Voltage, float &Capacity_AH, unsigned long &dischargeTime_s, float &Current) {
	if (usbConnected && !enableSendOscillogramToSerial) {
		Serial.print(Voltage, 3); 
		Serial.print(F("\t"));

		Serial.print(Capacity_AH * 1000); 
		Serial.print(F("\t"));

		Serial.print(dischargeTime_s); 
		Serial.print(F("\t"));

		Serial.println(Current, 3); 
	}
}

void sendOscillogramToSerial(float &Time_s, float &Voltage, float &Current, float &Capacity_AH) {
	if (usbConnected) {
		Serial.print(Time_s, 3); 
		Serial.print(F("\t"));

		Serial.print(Voltage, 3); 
		Serial.print(F("\t"));

		Serial.print(Current, 3); 
		Serial.print(F("\t"));

		Serial.println(Capacity_AH * 1000); 
	}
}
// unsigned long prevDebugInfoTime;  
// unsigned long markerDebugInfoTime;  
// void sendTimeDebugInfoToSerial(const __FlashStringHelper *msg, byte marker = 0) {
// 	if (usbConnected && debugMode) {
// 		unsigned long curDebugInfoTime = millis();
// 		Serial.print(curDebugInfoTime); 
// 		Serial.print(msg);
// 		Serial.println(curDebugInfoTime - prevDebugInfoTime); 
		
// 		if (marker) {
// 			Serial.print(F("marker "));
// 			Serial.println(curDebugInfoTime - markerDebugInfoTime); 
// 			markerDebugInfoTime = curDebugInfoTime;
// 		}

// 		prevDebugInfoTime = curDebugInfoTime;
// 	}
// }

void saveRecordToEEPROM(AccumCapacityRecord &capacityRecord, bool rewriteLastRecord/* = false*/) {
	byte indexOfLastRecordInEEPROM = EEPROM[0];
	if (!rewriteLastRecord) {
		indexOfLastRecordInEEPROM = (indexOfLastRecordInEEPROM + 1) % numOfRecordsInEEPROM;
		EEPROM[0] = indexOfLastRecordInEEPROM;
	}	

	EEPROM.put(offssetOfLogRecordsInEEPROM + indexOfLastRecordInEEPROM * sizeof(AccumCapacityRecord), capacityRecord);
}

void checkCommandsFromSerial() {
	if (Serial.available() > 0) {
		auto str = Serial.readString();
		if (str == F("meas"))
			enableSendMeasurementsToSerial = !enableSendMeasurementsToSerial;

		if (str == F("oscil")) {
			enableSendOscillogramToSerial = !enableSendOscillogramToSerial;
			timeOscillogramStarted_us = micros();
		}

		if (str == F("log")) {

			byte indexOfLastRecordInEEPROM = EEPROM[0];

			printTableCaption(F("***Log***"));	

			for (byte i = 0; i < numOfRecordsInEEPROM; i++) {
				byte ri = (i + indexOfLastRecordInEEPROM + 1) % numOfRecordsInEEPROM;

				AccumCapacityRecord capacityRecord;
				EEPROM.get(offssetOfLogRecordsInEEPROM + ri * sizeof(AccumCapacityRecord), capacityRecord);

				sendMeasurementsToSerial(capacityRecord.Voltage, capacityRecord.Capacity_AH, 
					capacityRecord.dischargeTime_s, capacityRecord.Current);
			}

			printTableCaption(F("***Log end***"));

		}

		if (str.startsWith(F("set current"))) {
			str.remove(0, 11);
			str.trim();
			wantedDischargeCurrent = str.toFloat();
			if (wantedDischargeCurrent < 0) wantedDischargeCurrent = 0;
			if (wantedDischargeCurrent > maxDischargeCurrent) wantedDischargeCurrent = maxDischargeCurrent;
		}

		if (str.startsWith(F("set oscil period"))) {
			str.remove(0, 16);
			str.trim();
			oscilPeriod_ms = str.toInt();
			if (oscilPeriod_ms < 0) oscilPeriod_ms = 0;
		}

		if (str.startsWith(F("debug")))
			debugMode = !debugMode;

		if (str.startsWith(F("showFreeRAM")))
			showFreeRAM = !showFreeRAM;

	}
}



