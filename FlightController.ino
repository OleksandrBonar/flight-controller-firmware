#include <Joystick.h>

//#define AUTOSEND_MODE
//#define DEBUG

Joystick_ joystick1(0x03, JOYSTICK_TYPE_JOYSTICK,
  11, 0, // Button Count, Hat Switch Count
  true, true, true, // X ,Y, no Z Axis
  true, false, true, // No Rx, Ry, or Rz
  false, true, // Rudder and Throttle
  false, false, false); // No accelerator, brake, or steering

Joystick_ joystick2(0x04, JOYSTICK_TYPE_JOYSTICK,
  0, 0, // Button Count, Hat Switch Count
  false, false, true, // X ,Y, no Z Axis
  true, true, true, // No Rx, Ry, or Rz
  false, false, // Rudder and Throttle
  false, false, false); // No accelerator, brake, or steering

unsigned long timer = 0;
int xaxis = 0;
int yaxis = 0;
int zaxis = 0;
int rxaxis = 0;
int ryaxis = 0;
int rzaxis = 0;
unsigned int throttle = 0;
unsigned int rudder = 0;
unsigned int flaps = 0;
unsigned int trimmer = 0;

int pitchPin = A0;
int rollPin = A1;
int yawPin = A2;
int brakeLeftPin = A3;
int brakeRightPin = A4;
int throttlePin = A5;
int propellerPin = 10;
int mixturePin = 9;
int flapsPin = 8;

int matrixColPin1 = 13;
int matrixColPin2 = 12;
int matrixColPin3 = 11;
int matrixRowPin1 = 7;
int matrixRowPin2 = 6;
int matrixRowPin3 = 5;
int matrixRowPin4 = 4;
int matrixRows[] = {matrixRowPin1, matrixRowPin2, matrixRowPin3, matrixRowPin4};
const int matrixRowCnt = sizeof(matrixRows) / sizeof(matrixRows[0]);
int matrixCols[] = {matrixColPin1, matrixColPin2, matrixColPin3};
const int matrixColCnt = sizeof(matrixCols) / sizeof(matrixCols[0]);
int matrixBtns[matrixColCnt][matrixRowCnt];

int encoderIntA = 0;
int encoderIntB = 1;
int encoderPinA = 2;
int encoderPinB = 3;

// Joystick Buttons
int ignitionOffBtn = 0;
int ignitionBothBtn = 1;
int ignitionStartBtn = 2;

int autopilotBtn = 3;

int pitoHeatBtn = 4;
int fuelPumpBtn = 5;

int lightsBcnBtn = 6;
int lightsLandBtn = 7;
int lightsTaxiBtn = 8;
int lightsNavBtn = 9;
int lightsStrobeBtn = 10;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastEncoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

long readEncoderValue(void)
{
    return encoderValue / 4;
}

void updateEncoderValue()
{
  int MSB = digitalRead(encoderPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) |LSB; // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    if (encoderValue <= 100) {
      encoderValue++;
    }
  }

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    if (encoderValue >= -100) {
      encoderValue--;
    }
  }

  lastEncoded = encoded; // store this value for next time
}

void initEncoder()
{
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // call updateEncoderValue() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(encoderIntA, updateEncoderValue, CHANGE);
  attachInterrupt(encoderIntB, updateEncoderValue, CHANGE);
}

void initMatrix()
{
  for (int x = 0; x < matrixRowCnt; x++) {
    pinMode(matrixRows[x], INPUT);
  }

  for (int x = 0; x < matrixColCnt; x++) {
    pinMode(matrixCols[x], INPUT_PULLUP);
  }
}

void readMatrix()
{
  // iterate the columns
  for (int colIndex = 0; colIndex < matrixColCnt; colIndex++) {
    // col: set to output to low
    int curCol = matrixCols[colIndex];
    pinMode(curCol, OUTPUT);
    digitalWrite(curCol, LOW);

    // row: interate through the rows
    for (int rowIndex = 0; rowIndex < matrixRowCnt; rowIndex++) {
      byte rowCol = matrixRows[rowIndex];
      pinMode(rowCol, INPUT_PULLUP);
      matrixBtns[colIndex][rowIndex] = digitalRead(rowCol);
      pinMode(rowCol, INPUT);
    }

    // disable the column
    pinMode(curCol, INPUT);
  }
}

void printMatrix()
{
  for (int rowIndex = 0; rowIndex < matrixRowCnt; rowIndex++) {
    if (rowIndex < 10) {
      Serial.print(F("0"));
    }
    Serial.print(rowIndex);
    Serial.print(F(": "));

    for (int colIndex = 0; colIndex < matrixColCnt; colIndex++) { 
      Serial.print(matrixBtns[colIndex][rowIndex]);

      if (colIndex < matrixColCnt) {
        Serial.print(F(", "));
      }
    }

    Serial.println("");
  }

  Serial.println("");
}

void setup()
{
  Serial.begin(9600);

  initMatrix();
  initEncoder();

  // Yoke: pitch and roll
  joystick1.setXAxisRange(109, 643);
  joystick1.setYAxisRange(496, 896);

  // Brakes: left and right
  joystick1.setZAxisRange(471, 546);
  joystick1.setRzAxisRange(647, 728);

  joystick1.setThrottleRange(0, 1023);
  // Rudder
  joystick1.setRxAxisRange(750, 882);

  // Flaps
  joystick2.setRxAxisRange(379, 621);
  // Trimmer
  joystick2.setRyAxisRange(-100, 100);

  // Propeller and Mixture
  joystick2.setZAxisRange(0, 1023);
  joystick2.setRzAxisRange(0, 1023);

#ifdef AUTOSEND_MODE
  joystick1.begin();
  joystick2.begin();
#else
  joystick1.begin(false);
  joystick2.begin(false);
#endif

  timer = millis();
}

void loop()
{
  // Joystick1
  xaxis = analogRead(pitchPin);
  joystick1.setXAxis(xaxis);

  yaxis = analogRead(rollPin);
  joystick1.setYAxis(yaxis);

  rxaxis = analogRead(brakeLeftPin);
  joystick1.setZAxis(rxaxis);

  ryaxis = analogRead(brakeRightPin);
  joystick1.setRzAxis(ryaxis);

  throttle = analogRead(throttlePin);
  joystick1.setThrottle(throttle);

  rudder = analogRead(yawPin);
  joystick1.setRxAxis(rudder);

  readMatrix();

  joystick1.setButton(ignitionOffBtn, !matrixBtns[1][2]);
  joystick1.setButton(ignitionBothBtn, matrixBtns[1][2] && matrixBtns[2][2]);
  joystick1.setButton(ignitionStartBtn, !matrixBtns[2][2]);

  joystick1.setButton(autopilotBtn, !matrixBtns[0][3]);

  joystick1.setButton(pitoHeatBtn, !matrixBtns[0][0]);
  joystick1.setButton(fuelPumpBtn, !matrixBtns[0][2]);

  joystick1.setButton(lightsBcnBtn, !matrixBtns[2][1]);
  joystick1.setButton(lightsLandBtn, !matrixBtns[1][1]);
  joystick1.setButton(lightsTaxiBtn, !matrixBtns[0][1]);
  joystick1.setButton(lightsNavBtn, !matrixBtns[2][0]);
  joystick1.setButton(lightsStrobeBtn, !matrixBtns[1][0]);

  // Joystick2 
  flaps = analogRead(flapsPin); // 379-621
  joystick2.setRxAxis(flaps);
  joystick2.setRyAxis(encoderValue);

  zaxis = analogRead(propellerPin);
  joystick2.setZAxis(zaxis);

  rzaxis = analogRead(mixturePin);
  joystick2.setRzAxis(rzaxis);

#ifndef AUTOSEND_MODE
  joystick1.sendState();
  joystick2.sendState();
#endif

#ifdef DEBUG
  if (millis() - timer >= 1000) {
    timer = millis();

    Serial.print("xaxis: ");
    Serial.println(xaxis);

    Serial.print("yaxis: ");
    Serial.println(yaxis);

    Serial.print("zaxis: ");
    Serial.println(zaxis);

    Serial.print("rxaxis: ");
    Serial.println(rxaxis);

    Serial.print("ryaxis: ");
    Serial.println(ryaxis);

    Serial.print("rzaxis: ");
    Serial.println(rzaxis);

    Serial.print("throttle: ");
    Serial.println(throttle);

    Serial.print("rudder: ");
    Serial.println(rudder);

    Serial.print("flaps: ");
    Serial.println(flaps);

    Serial.print("encoder: ");
    Serial.println(encoderValue);
  }
#endif
}
