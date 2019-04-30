/*
 * hardware.c
 *
 *  Created on: 2Sep.,2017
 *      Author: Ben V. Brown
 */

// These are all the functions for interacting with the hardware
#include "hardware.h"
#include "FreeRTOS.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
volatile uint16_t PWMSafetyTimer = 0;
volatile int16_t CalibrationTempOffset = 0;
uint16_t tipGainCalValue = 0;
void setTipType(enum TipType tipType, uint8_t manualCalGain) {
	if (manualCalGain)
		tipGainCalValue = manualCalGain;
	else
		tipGainCalValue = lookupTipDefaultCalValue(tipType);
}
void setCalibrationOffset(int16_t offSet) {
	CalibrationTempOffset = offSet;
}
uint16_t getHandleTemperature() {
	// We return the current handle temperature in X10 C
	// TMP36 in handle, 0.5V offset and then 10mV per deg C (0.75V @ 25C for
	// example) STM32 = 4096 count @ 3.3V input -> But We oversample by 32/(2^2) =
	// 8 times oversampling Therefore 32768 is the 3.3V input, so 0.1007080078125
	// mV per count So we need to subtract an offset of 0.5V to center on 0C
	// (4964.8 counts)
	//
	int32_t result = getADC(0);
	result -= 4965;  // remove 0.5V offset
	// 10mV per C
	// 99.29 counts per Deg C above 0C
	result *= 100;
	result /= 993;
	return result;
}

// fx16.16 fixed point type, used for calculations
typedef uint32_t fx_t;

#define FX_FRAC 16

#define fx_float(a) (a / (float)(1<<FX_FRAC))
#define fx_make(a)  ((fx_t)(a * (1<<FX_FRAC)))
#define fx_add(a,b) (a + b)
#define fx_sub(a,b) (a - b)
#define fx_mul(a,b) ((fx_t)(((uint64_t)a * b) >> FX_FRAC))
// no division because uint64_t division is problematic (routine takes about 400 bytes)
//#define fx_div(a,b) ((fx_t)(((uint64_t)a << FX_FRAC) / b))


static fx_t fx_linter(fx_t ratio, fx_t y0, fx_t y1) {
	return fx_add(y0, fx_mul(fx_sub(y1, y0), ratio));
}

// Lookup table for N type termocouple
// Created using polygon interpolation
// Source:
// Result is obtained by interpolating between two lookup entries
struct nLookup_t {
	uint16_t s;	// raw sample data
	uint16_t c;	// temperature in fx9.6 fixed point format
};

#define AMP_GAIN (1.0+(750e3/2.37e3))
#define ADC2mV (3.3*1000.0/4.0/4095.0)
// converts mV to sample data
#define sp_make(x) ((uint16_t)(x*AMP_GAIN/ADC2mV))

#define FX97_FRAC 7
// converts floats to fx9.7
#define fx97_make(a) ((uint16_t)(a * (1<<FX97_FRAC)))
// converts fx9.7 to fx_t
#define fx97_to_fx(x) (((fx_t)x)<<(FX_FRAC-FX97_FRAC))

static const struct nLookup_t nLookup[8] = {
	// nType
	/*
	{sp_make(0.000000), fx97_make(0.000000)},
	{sp_make(1.632445), fx97_make(60.973165)},
	{sp_make(3.225425), fx97_make(115.337541)},
	{sp_make(5.013433), fx97_make(172.687182)},
	{sp_make(7.066727), fx97_make(234.731031)},
	{sp_make(9.476449), fx97_make(304.057276)},
	{sp_make(12.297174), fx97_make(381.951755)},
	{sp_make(15.600000), fx97_make(470.108006)}
	*/
	// c type
	/*
	{sp_make(0.000000), fx97_make(0.000000)},
	{sp_make(0.765148), fx97_make(55.013861)},
	{sp_make(1.539694), fx97_make(105.913302)},
	{sp_make(2.442240), fx97_make(161.918284)},
	{sp_make(3.493425), fx97_make(223.613730)},
	{sp_make(4.727829), fx97_make(292.784099)},
	{sp_make(6.206138), fx97_make(372.478125)},
	{sp_make(8.074175), fx97_make(470.192003)},
	*/
	// kType

       {sp_make(0), fx97_make(0)},
       {sp_make(2.96), fx97_make(72.61)},
       {sp_make(5.90), fx97_make(144.12)},
       {sp_make(8.86), fx97_make(217.99)},
       {sp_make(11.80), fx97_make(290.15)},
       {sp_make(14.74), fx97_make(350.64)},
       {sp_make(17.70), fx97_make(430.78)},
	   {sp_make(20.65), fx97_make(500.00)},

	// gType
	/*{sp_make(0.000000), fx97_make(0.000000)},
	{sp_make(0.056435), fx97_make(31.810780)},
	{sp_make(0.214438), fx97_make(77.192567)},
	{sp_make(0.519326), fx97_make(133.478068)},
	{sp_make(1.026422), fx97_make(200.407976)},
	{sp_make(1.793533), fx97_make(278.309952)},
	{sp_make(2.884027), fx97_make(368.035289)},
	{sp_make(4.372252), fx97_make(471.270671)},
	*/
	//type d
	/*
	{sp_make(0.000000), fx97_make(0.000000)},
	{sp_make(0.465523), fx97_make(45.305463)},
	{sp_make(1.076232), fx97_make(95.264459)},
	{sp_make(1.853910), fx97_make(151.481287)},
	{sp_make(2.828534), fx97_make(214.669703)},
	{sp_make(4.041981), fx97_make(286.612233)},
	{sp_make(5.555256), fx97_make(370.002711)},
	{sp_make(7.484334), fx97_make(470.389025)},
	*/
};

#define nLookupLen (sizeof(nLookup)/sizeof(nLookup[0]))


uint16_t inverseLookup(uint16_t fx97) {
	uint8_t i = 0;
	for (; i < nLookupLen-1; i++) {
		if (nLookup[i].c > fx97) {
			break;
		}
	}

	struct nLookup_t l0 = nLookup[i-1];
	struct nLookup_t l1 = nLookup[i];

	uint16_t a = fx97 - l0.c;
	uint16_t b = l1.c - l0.c;

	fx_t ratio = (fx_t) (((uint32_t)a) << FX_FRAC) / b;
	return fx_linter(ratio, fx_make(l0.s), fx_make(l1.s)) >> FX_FRAC;
}

uint16_t getHandleOffset()  {
	uint16_t handleTemp = getHandleTemperature();
	fx_t fxHandleTemp = fx_mul(fx_make(handleTemp), fx_make(0.1));

	uint16_t fx97HandleTemp = fxHandleTemp >> (FX_FRAC - FX97_FRAC);

	return inverseLookup(fx97HandleTemp);
}


uint16_t tipMeasurementToC(uint16_t raw) {
	return tipMeasurementFX97(raw) >> FX97_FRAC;
}

uint16_t tipMeasurementFX97(uint16_t raw) {
	// ColdJunctionOffset = LUT-1(handleTemp)
	// LUT(Raw Tip - RawOffset + ColdJunctionOffset) = TipTemp in fx16.16
	// return TipTemp >> FX_FRAC

	/*uint16_t s = raw - CalibrationTempOffset;
	if (s > raw) {
		s= 0;
	}

	fx_t mV = fx_mul(fx_make(s), fx_make(1/(AMP_GAIN/ADC2mV)*100));
	return mV >> (FX_FRAC - FX97_FRAC);
	*/




	uint16_t handleOffset = getHandleOffset();

	uint32_t raw32 = raw;
	raw32 += handleOffset;
	raw32 -= CalibrationTempOffset;
	uint16_t sample = raw32;

	//fx_t mV = fx_mul(fx_make(handleOffset), fx_make(1/(AMP_GAIN/ADC2mV)*100));
	//return mV >> (FX_FRAC - FX97_FRAC);

	uint8_t i = 0;
	// nLookupLen-1 because we want to use last entry if we are over it
	for (; i < nLookupLen-1; i++) {
		if (nLookup[i].s > sample) {
			break;
		}
	}
	// i can't be 0

	struct nLookup_t l0 = nLookup[i-1];
	struct nLookup_t l1 = nLookup[i];

	uint16_t a = sample - l0.s; // fx16.0
	uint16_t b = l1.s - l0.s; // fx16.0

	// we can divide a and b in a uint32_t becuse they are fx16.0
	// result is fx16.16 which is fx_t
	fx_t ratio = (fx_t) (((uint32_t)a) << FX_FRAC) / b;
	fx_t c = fx_linter(ratio, fx97_to_fx(l0.c), fx97_to_fx(l1.c));

	return c >> (FX_FRAC - FX97_FRAC);
}
/*uint16_t ctoTipMeasurement(uint16_t temp) {
	//[ (temp-handle/10) * 10000 ]/calibrationgain = tip raw delta
	// tip raw delta + tip offset = tip ADC reading
	uint16_t handleOffset = getHandleOffset();
	uint16_t TipRaw = inverseLookup(temp << 7);
	TipRaw += CalibrationTempOffset;
	TipRaw -= handleOffset;
	return TipRaw;
}
*/

uint16_t cToTempTarget(uint16_t c) {
	return c << FX97_FRAC;
}
uint16_t fToTempTarget(uint16_t f) {
	return cToTempTarget(((f - 32) * 5) / 9);
}

uint16_t tipMeasurementToF(uint16_t raw) {
	// Convert result from C to F
	return (tipMeasurementToC(raw) * 9) / 5 + 32;
}
/*uint16_t ftoTipMeasurement(uint16_t temp) {
	// Convert the temp back to C from F
	return ctoTipMeasurement(((temp - 32) * 5) / 9);
}
*/

uint16_t getTipInstantTemperature() {
	uint16_t sum;
	sum = hadc1.Instance->JDR1;
	sum += hadc1.Instance->JDR2;
	sum += hadc1.Instance->JDR3;
	sum += hadc1.Instance->JDR4;
	sum += hadc2.Instance->JDR1;
	sum += hadc2.Instance->JDR2;
	sum += hadc2.Instance->JDR3;
	sum += hadc2.Instance->JDR4;
	return sum;  // 8x over sample
}
/*
 * Loopup table for the tip calibration values for
 * the gain of the tip's
 * This can be found by line of best fit of TipRaw on X, and TipTemp-handle on
 * Y. Then take the m term * 10000
 * */
uint16_t lookupTipDefaultCalValue(enum TipType tipID) {
#ifdef MODEL_TS100
	switch (tipID) {
		case TS_D24:
		return 141;
		break;
		case TS_BC2:
		return (133 + 129) / 2;
		break;
		case TS_C1:
		return 133;
		break;
		case TS_B2:
		return 133;
		default:
		return 132;  // make this the average of all
		break;
	}
#else
	switch (tipID) {
	case TS_D25:
		return 154;
		break;
	case TS_B02:
		return 154;
		break;
	default:
		return 154;  // make this the average of all
		break;
	}
#endif
}

uint16_t getTipRawTemp(uint8_t refresh) {
	static uint16_t lastSample = 0;

	if (refresh) {
		lastSample = getTipInstantTemperature();
	}

	return lastSample;
}

uint16_t getInputVoltageX10(uint16_t divisor, uint8_t sample) {
	// ADC maximum is 32767 == 3.3V at input == 28.05V at VIN
	// Therefore we can divide down from there
	// Multiplying ADC max by 4 for additional calibration options,
	// ideal term is 467
#define BATTFILTERDEPTH 32
	static uint8_t preFillneeded = 10;
	static uint32_t samples[BATTFILTERDEPTH];
	static uint8_t index = 0;
	if (preFillneeded) {
		for (uint8_t i = 0; i < BATTFILTERDEPTH; i++)
			samples[i] = getADC(1);
		preFillneeded--;
	}
	if (sample) {
		samples[index] = getADC(1);
		index = (index + 1) % BATTFILTERDEPTH;
	}
	uint32_t sum = 0;

	for (uint8_t i = 0; i < BATTFILTERDEPTH; i++)
		sum += samples[i];

	sum /= BATTFILTERDEPTH;
	return sum * 4 / divisor;
}
#ifdef MODEL_TS80
uint8_t QCMode = 0;
uint8_t QCTries = 0;
void seekQC(int16_t Vx10, uint16_t divisor) {
	if (QCMode == 5)
		startQC(divisor);
	if (QCMode == 0)
		return;  // NOT connected to a QC Charger

	if (Vx10 < 45)
		return;
	if (Vx10 > 130)
		Vx10 = 130;  //Cap max value at 13V
	// Seek the QC to the Voltage given if this adapter supports continuous mode
	// try and step towards the wanted value

	// 1. Measure current voltage
	int16_t vStart = getInputVoltageX10(divisor, 0);
	int difference = Vx10 - vStart;

	// 2. calculate ideal steps (0.2V changes)

	int steps = difference / 2;
	if (QCMode == 3) {
		while (steps < 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	//D+0.6
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	//D-3.3V
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// D-3.3Vs
			vTaskDelay(3);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	//-0.6V
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

			HAL_Delay(1);
			steps++;
		}
		while (steps > 0) {
			// step once up
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			vTaskDelay(3);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

			HAL_Delay(1);
			steps--;
		}
	}
	// Re-measure
	/* Disabled due to nothing to test and code space of around 1k*/
#ifdef QC2_ROUND_DOWN
	steps = vStart - getInputVoltageX10(195);
	if (steps < 0) steps = -steps;
	if (steps > (difference / 2)) {
		// No continuous mode, so QC2
		QCMode = 2;
		// Goto nearest
		if (Vx10 > 10.5) {
			// request 12V
			// D- = 0.6V, D+ = 0.6V
			// Clamp PB3
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);// pull down D+
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		} else {
			// request 9V
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	}
#endif
}

// Must be called after FreeRToS Starts
void startQC(uint16_t divisor) {
	// Pre check that the input could be >5V already, and if so, dont both
	// negotiating as someone is feeding in hv
	uint16_t vin = getInputVoltageX10(divisor, 1);
	if (vin > 150)
		return;	// Over voltage
	if (vin > 100) {
		QCMode = 1;  // ALready at ~12V
		return;
	}
	GPIO_InitTypeDef GPIO_InitStruct;

	// Tries to negotiate QC for 9V
	// This is a multiple step process.
	// 1. Set around 0.6V on D+ for 1.25 Seconds or so
	// 2. After this It should un-short D+->D- and instead add a 20k pulldown on
	// D-
	// 3. Now set D+ to 3.3V and D- to 0.6V to request 9V
	// OR both at 0.6V for 12V request (if the adapter can do it).
	// If 12V is implimented then should fallback to 9V after validation
	// Step 1. We want to pull D+ to 0.6V
	// Pull PB3 donwn to ground
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);// pull low to put 0.6V on D+
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);// pull low to put 0.6V on D+
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_13;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Delay 1.25 seconds
	uint8_t enteredQC = 0;
	for (uint16_t i = 0; i < 130 && enteredQC == 0; i++) {
		//		HAL_Delay(10);
		vTaskDelay(1);

	}
	// Check if D- is low to spot a QC charger
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET)
		enteredQC = 1;
	if (enteredQC) {
		// We have a QC capable charger
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_8;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		// Wait for frontend ADC to stabilise
		QCMode = 4;
		for (uint8_t i = 0; i < 10; i++) {
			if (getInputVoltageX10(divisor, 1) > 80) {
				// yay we have at least QC2.0 or QC3.0
				QCMode = 3;	// We have at least QC2, pray for 3
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
				return;
			}
			vTaskDelay(10);  // 100mS
		}
		QCMode = 5;
		QCTries++;
		if (QCTries > 10) // 10 goes to get it going
			QCMode = 0;
	} else {
		// no QC
		QCMode = 0;

	}
	if (QCTries > 10)
		QCMode = 0;
}
// Get tip resistance in milliohms
uint32_t calculateTipR() {
	static uint32_t lastRes = 0;
	if (lastRes)
		return lastRes;
	// We inject a small current into the front end of the iron,
	// By measuring the Vdrop over the tip we can calculate the resistance
	// Turn PA0 into an output and drive high to inject (3.3V-0.6)/(6K8+Rtip)
	// current PA0->Diode -> 6K8 -> Tip -> GND So the op-amp will amplify the
	// small signal across the tip and convert this into an easily read voltage
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	// Set low first
	setTipPWM(0);
	vTaskDelay(1);
	uint32_t offReading = getTipRawTemp(1);
	for (uint8_t i = 0; i < 49; i++) {
		vTaskDelay(1);  // delay to allow it to stabilize
		HAL_IWDG_Refresh(&hiwdg);
		offReading += getTipRawTemp(1);
	}

	// Turn on
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  // Set hgih
	vTaskDelay(1); // delay to allow it too stabilize
	uint32_t onReading = getTipInstantTemperature();
	for (uint8_t i = 0; i < 49; i++) {
		vTaskDelay(1);  // delay to allow it to stabilize
		HAL_IWDG_Refresh(&hiwdg);
		onReading += getTipRawTemp(1);
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Turn the output off finally
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	uint32_t difference = onReading - offReading;
	// V = IR, therefore I = V/R
	// We can divide this reading by a known "gain" to get the resulting
	// resistance This was determined emperically This tip is 4.688444162 ohms,
	// 4688 milliohms (Measured using 4 terminal measurement) 25x oversampling
	// reads this as around 47490 Almost perfectly 10x the milliohms value This
	// will drift massively with tip temp However we really only need 10x ohms
	lastRes = (difference / 21) + 1;	// ceil
	return lastRes;
}
static unsigned int sqrt32(unsigned long n) {
	unsigned int c = 0x8000;
	unsigned int g = 0x8000;

	for (;;) {
		if (g * g > n)
			g ^= c;
		c >>= 1;
		if (c == 0)
			return g;
		g |= c;
	}
}
int16_t calculateMaxVoltage(uint8_t useHP) {
	// This measures the tip resistance, then it calculates the appropriate
	// voltage To stay under ~18W. Mosfet is "9A", so no issues there
	// QC3.0 supports up to 18W, which is 2A @9V and 1.5A @12V
	uint32_t milliOhms = calculateTipR();
	// Check no tip
	if (milliOhms > 10000)
		return -1;
	//Because of tolerance, if a user has asked for the higher power mode, then just goto 12V and call it a day
	if (useHP)
		return 120;
	//
	// V = sqrt(18W*R)
	// Convert this to sqrt(18W)*sqrt(milli ohms)*sqrt(1/1000)

	uint32_t Vx = sqrt32(milliOhms);
	if (useHP)
		Vx *= 1549;	//sqrt(24)*sqrt(1/1000)*10000
	else
		Vx *= 1342;	// sqrt(18) * sqrt(1/1000)*10000

	// Round to nearest 200mV,
	// So divide by 100 to start, to get in Vxx
	Vx /= 100;
	if (Vx % 10 >= 5)
		Vx += 10;
	Vx /= 10;
	// Round to nearest increment of 2
	if (Vx % 2 == 1)
		Vx++;
	//Because of how bad the tolerance is on detecting the tip resistance is
	//Its more functional to bin this
	if (Vx < 90)
		Vx = 90;
	else if (Vx >= 105)
		Vx = 120;
	return Vx;
}

#endif
volatile uint8_t pendingPWM = 0;

void setTipPWM(uint8_t pulse) {
	PWMSafetyTimer = 10; // This is decremented in the handler for PWM so that the tip pwm is
						 // disabled if the PID task is not scheduled often enough.

	pendingPWM = pulse;
}

// These are called by the HAL after the corresponding events from the system
// timers.

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Period has elapsed
	if (htim->Instance == TIM2) {
		// we want to turn on the output again
		PWMSafetyTimer--;
		// We decrement this safety value so that lockups in the
		// scheduler will not cause the PWM to become locked in an
		// active driving state.
		// While we could assume this could never happen, its a small price for
		// increased safety
		htim2.Instance->CCR4 = pendingPWM;
		if (htim2.Instance->CCR4 && PWMSafetyTimer) {
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		} else {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}
	} else if (htim->Instance == TIM1) {
		// STM uses this for internal functions as a counter for timeouts
		HAL_IncTick();
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	// This was a when the PWM for the output has timed out
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	}
}
