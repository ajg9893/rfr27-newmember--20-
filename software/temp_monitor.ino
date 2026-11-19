/*
Project: RFR New Member Project
Group: 20
Authors: Arjun
Date: 10/18/26 v0.1 Code Skeleton
      11/19/26 v0.2 5 V reference, 3-diode calibration, lookup table, I2C send, sensor checks

Hardware Used: Arduino Nano, CD4051BE (8 channels), sensors = 3x 1N4007 in series per channel,
               each biased from 5 V through 4.7 kOhm, 100 nF filter cap per channel
Outputs: Maximum temperature over an I2C bus, and a blinking red LED above 60C

Description:
1. selectChannel() writes the channel number in binary to the mux select pins A, B, C,
   -> connects sensor to the Nano's A0 pin.
2. ADC converts the sensor voltage (about 1.3-2.4 V) to a number using the 5 V
   reference. Each channel is read 8 times and averaged to smooth out noise.
3. The voltage is converted to temperature with a lookup table and linear interpolation.
   Each channel gets its own offset from a room-temperature calibration, because no two
   diode strings have exactly the same voltage. (3 diodes x -2 mV/C = about -6 mV/C.)
4. Channels whose voltage is outside the valid range (open or shorted sensor) are flagged
   and skipped.
5. The hottest valid channel is the max temperature.
6. Fault turns on above 60C and turns off at or below 57C (hysteresis).
7. The red LED blinks at 2 Hz while the fault is active (non-blocking, uses millis()).
8. When the I2C master requests data, the Nano sends 3 bytes:
     byte 0-1: max temperature x 10 as a signed 16-bit integer, high byte first
               (example: 61.3 C -> 613 -> 0x02, 0x65)
     byte 2:   status bits: bit 0 = fault, bit 1 = at least one sensor invalid
   CONFIRM THIS FORMAT WITH THE LV LEAD before the demo.

Calibration steps (do this once on the real board):
1. Measure the Nano's 5V pin with a multimeter and put it in V_REF.
2. Set DEBUG to 1, upload, open Serial Monitor at 115200 baud.
3. With every sensor at room temperature, copy each channel's printed voltage into V_CAL,
   and the actual room temperature into TEMP_CAL.
4. If the lead has a voltage vs. temperature table for the sensors, replace LUT_V / LUT_T.
*/

#include <Wire.h>

// Pins 
const int PIN_A = 2;      // mux A 
const int PIN_B = 3;      // mux B 
const int PIN_C = 4;      // mux C 
const int PIN_ADC = A0;   // mux COM
const int PIN_LED = 7;    // 2N7000 gate

// ADC 
const float V_REF = 5.0; 
const int NUM_SAMPLES = 8, NUM_CHANNELS = 8;

// Valid sensor voltage window. Open sensor reads about 5 V, shorted sensor about 0 V.
const float V_MIN_VALID = 1.0;
const float V_MAX_VALID = 2.6;

// Fault / LED 
const float FAULT_TEMP = 60.0;
const float CLEAR_TEMP = 57.0;
const unsigned long BLINK_MS = 250;   // toggle every 250 ms = 2 Hz blink

// Calibration 
// Room-temperature voltage of each channel's sensor string 
const float V_CAL[NUM_CHANNELS]    = {1.65, 1.65, 1.65, 1.65, 1.65, 1.65, 1.65, 1.65};
// Room temperature when V_CAL was measured 
const float TEMP_CAL[NUM_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0};

// Lookup table for a nominal sensor string: voltage (V) vs temperature (C).
// Voltage falls as temperature rises, so LUT_V is in DESCENDING order.
// PLACEHOLDER values from -6 mV/C around 1.65 V at 25 C. Replace with the lead's data
// or with points measured at known temperatures if available.
const int LUT_SIZE = 8;
const float LUT_V[LUT_SIZE] = {1.920, 1.800, 1.680, 1.560, 1.440, 1.320, 1.200, 1.080};
const float LUT_T[LUT_SIZE] = {-20.0,   0.0,  20.0,  40.0,  60.0,  80.0, 100.0, 120.0};

// I2C 
const int I2C_ADDRESS = 0x08;
const int I2C_LEN = 3;

//  Debug 
#define DEBUG 1                       
const unsigned long PRINT_MS = 500;

//  Globals 
float temps[NUM_CHANNELS];
float volts[NUM_CHANNELS];
bool valid[NUM_CHANNELS];
float offsets[NUM_CHANNELS];         

float maxTemp = 0;
bool fault = false;
bool sensorError = false;

bool ledOn = false;
unsigned long last_toggled = 0;
unsigned long last_printed = 0;

// Bytes the I2C handler sends
volatile uint8_t i2cBuf[I2C_LEN] = {0, 0, 0};

// ---------------- Setup ----------------
void setup() {
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  analogReference(DEFAULT);         
  analogRead(PIN_ADC);                

  computeOffsets();

  Wire.begin(I2C_ADDRESS);             // join the I2C bus as a slave
  Wire.onRequest(onI2CRequest);

#if DEBUG
  Serial.begin(115200);
#endif
}

// ---------------- Main loop ----------------
void loop() {
  scanAllChannels();
  maxTemp = findMaxTemp();
  updateFault();
  updateLed();
  updateI2CBuffer();

#if DEBUG
  printReadings();
#endif
}

// ---------------- Helper functions ----------------

// Connects the selected sensor to the Nano's A0 pin.
void selectChannel(int channel) {
  digitalWrite(PIN_A, (channel >> 0) & 1);
  digitalWrite(PIN_B, (channel >> 1) & 1);
  digitalWrite(PIN_C, (channel >> 2) & 1);
}

// Returns the averaged voltage on one channel.
float readVoltage(int channel) {
  selectChannel(channel);
  delayMicroseconds(10);               // let the mux output settle
  analogRead(PIN_ADC);                 // discard first reading after switching

  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(PIN_ADC);
  }
  float avgCount = (float)sum / NUM_SAMPLES;
  return avgCount * V_REF / 1024.0;
}

// Linear interpolation on the lookup table: voltage -> temperature.
// Outside the table, it extends the first/last segment.
float lutVoltageToTemp(float v) {
  int i = 0;
  // LUT_V is descending: find the segment where LUT_V[i] >= v >= LUT_V[i+1]
  while (i < LUT_SIZE - 2 && v < LUT_V[i + 1]) {
    i++;
  }
  float v1 = LUT_V[i], v2 = LUT_V[i + 1];
  float t1 = LUT_T[i], t2 = LUT_T[i + 1];
  return t1 + (v - v1) * (t2 - t1) / (v2 - v1);
}

// Inverse lookup: temperature -> nominal voltage. Used only for calibration offsets.
float lutTempToVoltage(float t) {
  int i = 0;
  // LUT_T is ascending
  while (i < LUT_SIZE - 2 && t > LUT_T[i + 1]) {
    i++;
  }
  float v1 = LUT_V[i], v2 = LUT_V[i + 1];
  float t1 = LUT_T[i], t2 = LUT_T[i + 1];
  return v1 + (t - t1) * (v2 - v1) / (t2 - t1);
}

// Offset = how far each channel's string sits from the nominal table at the calibration temperature.
void computeOffsets() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    offsets[ch] = V_CAL[ch] - lutTempToVoltage(TEMP_CAL[ch]);
  }
}

float voltageToTemp(int channel, float voltage) {
  return lutVoltageToTemp(voltage - offsets[channel]);
}

void scanAllChannels() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    float v = readVoltage(i);
    volts[i] = v;
    valid[i] = (v >= V_MIN_VALID && v <= V_MAX_VALID);
    temps[i] = voltageToTemp(i, v);
  }
}

// Highest temperature among valid channels. Sets sensorError if any channel is invalid.
float findMaxTemp() {
  bool found = false;
  float highest = 0;
  sensorError = false;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (!valid[i]) {
      sensorError = true;
      continue;
    }
    if (!found || temps[i] > highest) {
      highest = temps[i];
      found = true;
    }
  }
  return highest;                      // 0 if no channel is valid (sensorError will be set)
}

void updateFault() {
  if (maxTemp > FAULT_TEMP) fault = true;
  else if (maxTemp <= CLEAR_TEMP) fault = false;
  // between 57 and 60: keep the previous state (hysteresis)
}

void updateLed() {
  if (!fault) {
    ledOn = false;
    digitalWrite(PIN_LED, LOW);
  }
  else if (millis() - last_toggled >= BLINK_MS) {
    ledOn = !ledOn;
    last_toggled = millis();
    digitalWrite(PIN_LED, ledOn);
  }
}

// Packs max temp and status into the I2C buffer.
void updateI2CBuffer() {
  float t10 = maxTemp * 10.0;
  if (t10 > 32767.0) t10 = 32767.0;
  if (t10 < -32768.0) t10 = -32768.0;
  int16_t tempX10 = (int16_t)lround(t10);

  uint8_t status = 0;
  if (fault)       status |= 0x01;
  if (sensorError) status |= 0x02;

  noInterrupts();                      // keep the I2C handler from reading mid-update
  i2cBuf[0] = (uint8_t)((tempX10 >> 8) & 0xFF);   // high byte
  i2cBuf[1] = (uint8_t)(tempX10 & 0xFF);          // low byte
  i2cBuf[2] = status;
  interrupts();
}

// Runs inside an interrupt when the master asks for data. Keep it short.
void onI2CRequest() {
  Wire.write((const uint8_t *)i2cBuf, I2C_LEN);   // send all bytes in one call
}

#if DEBUG
// Prints each channel's voltage and temperature twice a second (for calibration and testing).
void printReadings() {
  if (millis() - last_printed < PRINT_MS) return;
  last_printed = millis();

  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(volts[i], 3);
    Serial.print(" V  ");
    Serial.print(temps[i], 1);
    Serial.print(" C");
    if (!valid[i]) Serial.print(" (INVALID)");
    Serial.println();
  }
  Serial.print("MAX: ");
  Serial.print(maxTemp, 1);
  Serial.print(" C  FAULT: ");
  Serial.print(fault ? "ON" : "off");
  Serial.print("  SENSOR ERROR: ");
  Serial.println(sensorError ? "YES" : "no");
  Serial.println();
}
#endif