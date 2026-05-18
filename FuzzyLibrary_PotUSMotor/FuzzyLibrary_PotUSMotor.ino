#include <FuzzyLibrary.h>
#include <math.h>
#include "AVR_PWM.h"

const uint8_t POT_PIN = A5;
const uint8_t US_TRIG_PIN = 5;
const uint8_t US_ECHO_PIN = 6;
const uint8_t MOTOR_PWM_PIN = 9;
const uint8_t MOTOR_IN1_PIN = 10;
const uint8_t MOTOR_IN2_PIN = 11;
const uint8_t MOTOR_CCW_IN1_STATE = HIGH;
const uint8_t MOTOR_CCW_IN2_STATE = LOW;
const uint8_t MOTOR_CW_IN1_STATE = LOW;
const uint8_t MOTOR_CW_IN2_STATE = HIGH;

const float MOTOR_PWM_FREQUENCY = 245.5f;

const float POT_MIN = 100.0f;
const float POT_MAX = 170.0f;
const float US_MIN = 4.0f;
const float US_MAX = 34.0f;
const float MOTOR_MIN = -35.0f;
const float MOTOR_MAX = 35.0f;
const float MOTOR_DEADBAND = 1.0f;

Fuzzy fuzzy;
uint8_t pot;
uint8_t us;
uint8_t motor;

AVR_PWM* PWM_Instance;

uint8_t potHighAngle;
uint8_t potBalancedAngle;
uint8_t potLowAngle;

uint8_t usLowDistance;
uint8_t usMediumDistance;
uint8_t usHighDistance;

uint8_t motorCCW;
uint8_t motorNoRotation;
uint8_t motorCW;

void setupFuzzy(){
  pot = fuzzy.createVariable(INPUT);
  potHighAngle = fuzzy.addTerm(pot);
  potBalancedAngle = fuzzy.addTerm(pot);
  potLowAngle = fuzzy.addTerm(pot);

  us = fuzzy.createVariable(INPUT);
  usLowDistance = fuzzy.addTerm(us);
  usMediumDistance = fuzzy.addTerm(us);
  usHighDistance = fuzzy.addTerm(us);

  motor = fuzzy.createVariable(OUTPUT);
  motorCCW = fuzzy.addTerm(motor);
  motorNoRotation = fuzzy.addTerm(motor);
  motorCW = fuzzy.addTerm(motor);

  fuzzy.addPointsTo(potHighAngle, 100.0f, 100.0f, 100.0f, 130.0f);
  fuzzy.addPointsTo(potBalancedAngle, 120.0f, 130.0f, 140.0f, 150.0f);
  fuzzy.addPointsTo(potLowAngle, 140.0f, 170.0f, 170.0f, 170.0f);

  fuzzy.addPointsTo(usLowDistance, 4.0f, 4.0f, 4.0f, 18.5f);
  fuzzy.addPointsTo(usMediumDistance, 15.0f, 19.5f, 22.5f, 27.0f);
  fuzzy.addPointsTo(usHighDistance, 22.5f, 34.0f, 34.0f, 34.0f);

  fuzzy.addPointsTo(motorCCW, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, -30.0f);
  fuzzy.addPointsTo(motorNoRotation, -MOTOR_DEADBAND, 0.0f, 0.0f, MOTOR_DEADBAND);
  fuzzy.addPointsTo(motorCW, 30.0f, MOTOR_MAX, MOTOR_MAX, MOTOR_MAX);

  fuzzy.createRule(pot, potHighAngle, us, usLowDistance, motor, motorCW);
  fuzzy.createRule(pot, potBalancedAngle, us, usLowDistance, motor, motorCW);
  fuzzy.createRule(pot, potLowAngle, us, usLowDistance, motor, motorNoRotation);

  fuzzy.createRule(pot, potHighAngle, us, usMediumDistance, motor, motorCW);
  fuzzy.createRule(pot, potBalancedAngle, us, usMediumDistance, motor, motorNoRotation);
  fuzzy.createRule(pot, potLowAngle, us, usMediumDistance, motor, motorCCW);

  fuzzy.createRule(pot, potHighAngle, us, usHighDistance, motor, motorNoRotation);
  fuzzy.createRule(pot, potBalancedAngle, us, usHighDistance, motor, motorCCW);
  fuzzy.createRule(pot, potLowAngle, us, usHighDistance, motor, motorCCW);
}

float readPotAngle(){
  int rawValue = analogRead(POT_PIN);
  float angle = POT_MIN + (static_cast<float>(rawValue) / 1023.0f) * (POT_MAX - POT_MIN);
  return constrain(angle, POT_MIN, POT_MAX);
}

float readUltrasonicDistance(){
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 30000UL);
  if(duration == 0){
    return US_MAX;
  }

  float distance = (duration * 0.0343f) / 2.0f;
  return constrain(distance, US_MIN, US_MAX);
}

void setMotorOutput(float motorSpeed){
  float clampedSpeed = constrain(motorSpeed, MOTOR_MIN, MOTOR_MAX);
  float speedAbs = fabsf(clampedSpeed);
  int pwmValue = static_cast<int>((speedAbs / MOTOR_MAX) * 255.0f);
  pwmValue = constrain(pwmValue, 0, 255);

  if(clampedSpeed > MOTOR_DEADBAND){
    digitalWrite(MOTOR_IN1_PIN, MOTOR_CW_IN1_STATE);
    digitalWrite(MOTOR_IN2_PIN, MOTOR_CW_IN2_STATE);
  } else if(clampedSpeed < -MOTOR_DEADBAND){
    digitalWrite(MOTOR_IN1_PIN, MOTOR_CCW_IN1_STATE);
    digitalWrite(MOTOR_IN2_PIN, MOTOR_CCW_IN2_STATE);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    pwmValue = 0;
  }

  float dutyCycle = (static_cast<float>(pwmValue) / 255.0f) * 100.0f;
  if(PWM_Instance){
    PWM_Instance->setPWM(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY, dutyCycle);
  } else {
    analogWrite(MOTOR_PWM_PIN, pwmValue);
  }
}

void setup(){
  Serial.begin(115200);

  pinMode(POT_PIN, INPUT);
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  digitalWrite(US_TRIG_PIN, LOW);

  PWM_Instance = new AVR_PWM(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY, 0.0f);
  if(PWM_Instance){
    PWM_Instance->setPWM();
  }

  setupFuzzy();
}

void loop(){
  float potAngle = readPotAngle();
  float usDistance = readUltrasonicDistance();

  fuzzy.setVariableValue(pot, potAngle);
  fuzzy.setVariableValue(us, usDistance);

  float motorSpeed = fuzzy.calculate();
  setMotorOutput(motorSpeed);

  Serial.print("Pot=");
  Serial.print(potAngle);
  Serial.print(" US=");
  Serial.print(usDistance);
  Serial.print(" Motor=");
  Serial.println(motorSpeed);

  delay(100);
}
