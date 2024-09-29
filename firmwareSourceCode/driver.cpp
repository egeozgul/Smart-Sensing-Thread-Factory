#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD setup for 16 chars, 2 lines

#define thermistorPin A6
#define fanPin 6
#define conductivitySensorPin A2 
#define tensionSensorPin A3
#define heaterPin 9

// Speed calculation variable
float Speed = 10000.0f / 6000.0f;

// Function to convert sensor value to temperature
float convertToTemp(float Vo) {
  int ThermistorPin = 0;
  float R1 = 10000;
  float logR2, R2, T;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  R2 = R1 * (1023.0 / Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;

  return T;
}

// Buffer for moving average temperature calculation
float tempBuffer[30];

// Function to calculate moving average
float movingAverage(float *tempBuffer) {
  float sum = 0;
  for (int i = 0; i < 30; i++) {
    sum += tempBuffer[i];
  }
  return sum / 30.0f;
}

int index = 0;
void insertData(float newData) {
  tempBuffer[index] = newData;
  index = (index + 1) % 30;
}

int heaterValue = 0;

#define dirPin 2
#define stepPin 3
#define dirPinB 7
#define stepPinB 8
#define pumpDirPin 10
#define pumpStepPin 11

#define stepsPerRevolution 200

int stepperA_state = LOW;
int stepperB_state = LOW;
int stepperC_state = LOW;

// Stepper motor functions
void stepperStepA() {
  stepperA_state = !stepperA_state;
  digitalWrite(stepPin, stepperA_state);
}

void stepperStepB() {
  stepperB_state = !stepperB_state;
  digitalWrite(stepPinB, stepperB_state);
}

void stepperStepC() {
  stepperC_state = !stepperC_state;
  digitalWrite(pumpStepPin, stepperC_state);
}

// Setup for timer interrupts
void interruptSetup() {
  cli();  // Disable interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= B00000100;  // Set prescaler to 256
  TIMSK1 |= B00000010;  // Enable compare match A
  OCR1A = 5;  // Set compare register A value

  sei();  // Enable interrupts
}

long int timerPeriod = 80;

int counterA = 0;
int counterB = 0;
int counterC = 0;

long int posA = 0;
long int posB = 0;
long int posC = 0;
int motorDirB = 1;

int enableMotorA = 0;
int enableMotorB = 0;
int enableMotorC = 0;

volatile int counterALimit = 30;
volatile int counterBLimit = 30;
volatile int counterCLimit = 2;

ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;  // Reset timer

  counterA++;
  counterB++;
  counterC++;

  if (counterA >= counterALimit && enableMotorA == 1) {
    counterA = 0;
    stepperStepA();  
    posA++;
  }

  if (counterB >= counterBLimit && enableMotorB == 1) {
    counterB = 0;
    stepperStepB();

    if (motorDirB == 1) {
      digitalWrite(dirPinB, LOW);
    } else {
      digitalWrite(dirPinB, HIGH);
    }
    posB += motorDirB;
  }

  if (counterC >= counterCLimit && enableMotorC == 1) {
    counterC = 0;
    stepperStepC();
    posC++;
  }

  static int cc = 0;
  int dutyCycle = heaterValue;
  if (cc == 0) digitalWrite(heaterPin, HIGH);
  if (cc == dutyCycle) digitalWrite(heaterPin, LOW); 
  cc++;
  if (cc >= 10) cc = 0;
}

void setup() {
  // Set pin modes for stepper motors and peripherals
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pumpStepPin, OUTPUT);
  pinMode(pumpDirPin, OUTPUT);
  
  digitalWrite(dirPinB, LOW);
  digitalWrite(dirPin, HIGH);
  digitalWrite(pumpDirPin, LOW);
  
  interruptSetup();  // Set up interrupts

  lcd.init();  // Initialize LCD
  lcd.backlight();  // Turn on LCD backlight
  
  pinMode(13, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  analogWrite(fanPin, 255);  // Set fan speed

  Serial.begin(115200);

  pinMode(1, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);

  counterBLimit = 10000.0f / Speed;

  int stepperEnablePin = 4;
  pinMode(stepperEnablePin, OUTPUT);
  digitalWrite(stepperEnablePin, HIGH);
  delay(1000);
  digitalWrite(stepperEnablePin, LOW);
}

// PID control variables
static float P = 0;
static float I = 0;
static float D = 0;
static float kP = 250.0;
static float kI = 0.0001f;
static float kD = 0;
static float totalError = 0;

int ProcessStarted = false;
int ThreadTestStarted_A = false;
int ThreadTestStarted_B = false;

float inkFeedrate = 10;
float currentTemp = 0;
int desiredTensionRate = 850;
float desiredTemp  = 55;

static int selection = 0;
int nSelections = 8;

// Button functions
void buttonRight() {
  if (selection == 0) desiredTemp++;
  if (selection == 1) Speed += 10;
  if (selection == 2) desiredTensionRate += 5;
  if (selection == 4) inkFeedrate += 10;
  if (selection == 5) ProcessStarted = !ProcessStarted;
  if (selection == 6) ThreadTestStarted_A = !ThreadTestStarted_A;
  if (selection == 7) ThreadTestStarted_B = !ThreadTestStarted_B;

  counterBLimit = 10000.0f / Speed;
}

void buttonLeft() {
  if (selection == 0) desiredTemp--;
  if (selection == 1) Speed -= 10;
  if (selection == 2) desiredTensionRate -= 5;
  if (selection == 4) inkFeedrate -= 10;

  if (Speed < 5) Speed = 10;

  counterBLimit = 10000.0f / Speed;
}

void buttonUp() {
  selection = (selection + 1) % nSelections;
}

void buttonDown() {
  selection--;
  if (selection < 0) selection = nSelections - 1;
}

// Update the LCD with the current values
void updateLCD() {   
  lcd.setCursor(0, 0);
  lcd.print("tmp:");
  lcd.print((int)currentTemp);
  lcd.print("/");
  lcd.print((int)desiredTemp);
  
  lcd.setCursor(10, 0);
  lcd.print("Ink:");
  lcd.print(inkFeedrate);

  lcd.setCursor(0, 1);
  lcd.print("Con:");
  lcd.print(conductivity);

  lcd.setCursor(0, 2);
  lcd.print("Tension:");
  lcd.print((int)tensionRate);
  lcd.print("/");
  lcd.print((int)desiredTensionRate);

  lcd.setCursor(10, 1);
  lcd.print("Spd:");
  lcd.print(Speed);

  lcd.setCursor(0, 3);
  switch (selection) {
    case 0:
      lcd.print("Adjust temperature");  
      break;
    case 1:
      lcd.print("Adjust spool speed");
      break;
    case 2:
      lcd.print("Adjust tension");
      break;
    case 3:
      lcd.print("Manual pump control");
      break;
    case 4:
      lcd.print("Adjust pump rate");
      break;
    case 5:
      lcd.print(ProcessStarted ? "STOP COATING" : "START COATING");
      break;
    case 6:
      lcd.print(ThreadTestStarted_A ? "STOP THREAD TEST A" : "START THREAD TEST A");
      break;
    case 7:
      lcd.print(ThreadTestStarted_B ? "STOP THREAD TEST B" : "START THREAD TEST B");
      break;
  }
}

// PID control function for tension rate
void pid(int tensionRate) {
  float error = tensionRate - desiredTensionRate;
  float speedB = Speed;
  float speedA = speedB - kP * error;

  float result = 10000.0f / speedA;
  
  digitalWrite(dirPin, speedA >= 0 ? HIGH : LOW);

  counterALimit = abs(result);

  if (totalError > 0.30) totalError = 0.3;
  if (totalError < -0.30) totalError = -0.3;
}

void loop() {
  // Process control logic
  if (ProcessStarted) {
    enableMotorA = 1;
    enableMotorB = 1;
    digitalWrite(pumpDirPin, HIGH);
    
    if (inkFeedrate <= 5) {
      inkFeedrate = 5;
      counterCLimit = 1000000.0f / inkFeedrate;
      enableMotorC = 0;
    } else {
      counterCLimit = 1000000.0f / inkFeedrate;
      enableMotorC = 1;
    }
  } else if (ThreadTestStarted_A) {
    enableMotorA = 1;
    enableMotorB = 1;
    Speed = 5000;
    counterBLimit = 10000.0f / Speed; 
  } else if (ThreadTestStarted_B) {
    enableMotorA = 1;
    enableMotorB = 0;
  } else {
    enableMotorA = 0;
    enableMotorB = 0;
  }

  // Manual pump control
  if (digitalRead(1) == false && selection == 3) {
    digitalWrite(pumpDirPin, LOW);
    counterCLimit = 5;
    enableMotorC = 1;
  } else if (digitalRead(A0) == false && selection == 3) {
    digitalWrite(pumpDirPin, HIGH);
    counterCLimit = 5;
    enableMotorC = 1;
  } else {
    enableMotorC = ProcessStarted ? 1 : 0;
  }

  // Button handling
  int pins[4] = {0, A1, A0, 1};
  static long debouncingTimer[4] = {0, 0, 0, 0};
  static bool enable[4] = {1, 1, 1, 1};
  void (*buttonActions[4])() = {buttonUp, buttonDown, buttonRight, buttonLeft};
  
  for (int i = 0; i < 4; i++) {
    if (digitalRead(pins[i]) == false && enable[i] == 1) {
      debouncingTimer[i] = millis();
      enable[i] = 0;
      buttonActions[i]();
    }
    if (enable[i] == 0 && millis() > debouncingTimer[i] + 250) {
      enable[i] = 1;
    }
  }

  // Tension sensor reading
  tensionRate = analogRead(tensionSensorPin);
  
  // Conductivity sensor reading
  conductivity = analogRead(conductivitySensorPin);

  // Apply PID control for tension rate
  pid(tensionRate);

  // Heater control
  int rawTemp = analogRead(thermistorPin);
  insertData(rawTemp);
  rawTemp = movingAverage(tempBuffer);
  currentTemp = convertToTemp(rawTemp);

  heaterValue = (currentTemp < desiredTemp) ? 10 : 0;

  // Update the LCD periodically
  static unsigned long int LCD_update_counterStartTime = millis();
  if (millis() - LCD_update_counterStartTime > 50) {
    updateLCD();
    LCD_update_counterStartTime = millis();
  }

  // USB Serial logging
  static unsigned long int Log_counterStartTime = millis();
  if (millis() - Log_counterStartTime > 200) {
    if (ThreadTestStarted_A) {
      Serial.print((long int)posB);
      Serial.print(" ");
      Serial.println((int)conductivity);
    }

    if (ThreadTestStarted_B) {
      Serial.print((int)tensionRate);
      Serial.print(" ");
      Serial.print((int)conductivity);
      Serial.print(" ");
      Serial.println(counter_);
    }
    Log_counterStartTime = millis();
  }

  // Thread Test logic
  static int n_cycles = 1000;
  static bool countingUp = false;
  static int counter_period_millis = 25;
  static long int counter_counterStartTime = millis();

  if (millis() - counter_counterStartTime > counter_period_millis) {
    if (ThreadTestStarted_B) {
      if (countingUp) {
        if (tensionRate < 800) desiredTensionRate += 3;
        if (tensionRate > 800) {
          countingUp = false;
          counter_++;
        }
      } else {
        if (tensionRate > 450) desiredTensionRate -= 3;
        if (tensionRate < 450) countingUp = true;
      }
    }
    counter_counterStartTime = millis();
  }
  
  if (counter_ >= n_cycles) {
    ThreadTestStarted_B = false;
    Serial.println("CycleTestEnd");
  }
}
