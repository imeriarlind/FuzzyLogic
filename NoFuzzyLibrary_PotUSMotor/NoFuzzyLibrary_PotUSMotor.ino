#include <Arduino.h>
#include <math.h>
#include <AVR_PWM.h>

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

const float MOTOR_PWM_FREQUENCY = 250.5f;

const float POT_MIN = 110.0f;
const float POT_MAX = 180.0f;
const float US_MIN = 4.0f;
const float US_MAX = 41.0f;
const float MOTOR_MIN = -50.0f;
const float MOTOR_MAX = 50.0f;
const float MOTOR_DEADBAND = 1.0f;
const uint8_t MOTOR_MIN_DUTY_CYCLE = 40;
const uint8_t MOTOR_MAX_DUTY_CYCLE = 60;

struct FuzzyTerm {
  float a;
  float b;
  float c;
  float d;
};

struct FuzzyRule {
  uint8_t potTerm;
  uint8_t usTerm;
  uint8_t motorTerm;
};

enum PotTerm {
  POT_HIGH_ANGLE,
  POT_BALANCED_ANGLE,
  POT_LOW_ANGLE
};

enum UsTerm {
  US_LOW_DISTANCE,
  US_MEDIUM_DISTANCE,
  US_HIGH_DISTANCE
};

enum MotorTerm {
  MOTOR_CCW,
  MOTOR_NO_ROTATION,
  MOTOR_CW
};

const FuzzyTerm POT_TERMS[] = {
  {110.0f, 110.0f, 110.0f, 135.0f},
  {130.0f, 155.0f, 160.0f, 170.0f},
  {165.0f, 165.0f, 165.0f, 185.0f}
};

const FuzzyTerm US_TERMS[] = {
  {6.0f, 6.0f, 6.0f, 18.5f},
  {15.0f, 20.0f, 22.0f, 27.0f},
  {22.5f, 41.0f, 41.0f, 41.0f}
};

const FuzzyTerm MOTOR_TERMS[] = {
  {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, -30.0f},
  {-MOTOR_DEADBAND, 0.0f, 0.0f, MOTOR_DEADBAND},
  {30.0f, MOTOR_MAX, MOTOR_MAX, MOTOR_MAX}
};

const FuzzyRule RULES[] = {
  {POT_HIGH_ANGLE, US_LOW_DISTANCE, MOTOR_CW},
  {POT_BALANCED_ANGLE, US_LOW_DISTANCE, MOTOR_CW},
  {POT_LOW_ANGLE, US_LOW_DISTANCE, MOTOR_NO_ROTATION},

  {POT_HIGH_ANGLE, US_MEDIUM_DISTANCE, MOTOR_CW},
  {POT_BALANCED_ANGLE, US_MEDIUM_DISTANCE, MOTOR_NO_ROTATION},
  {POT_LOW_ANGLE, US_MEDIUM_DISTANCE, MOTOR_CCW},

  {POT_HIGH_ANGLE, US_HIGH_DISTANCE, MOTOR_NO_ROTATION},
  {POT_BALANCED_ANGLE, US_HIGH_DISTANCE, MOTOR_CCW},
  {POT_LOW_ANGLE, US_HIGH_DISTANCE, MOTOR_CCW}
};

const uint8_t RULE_COUNT = sizeof(RULES) / sizeof(RULES[0]);

AVR_PWM* PWM_Instance;

float calculateMembership(const FuzzyTerm& term, float input){
  const float a = term.a;
  const float b = term.b;
  const float c = term.c;
  const float d = term.d;

  if(input < a || input > d){
    return 0.0f;
  }

  if(a == b && b == c && c == d){
    return 1.0f;
  }

  if((a == b || a == c) && (d == b || d == c) && c != b){
    return 1.0f;
  }

  if(a < b && a < c && b == c && c == d){
    if(input == a){
      return 0.0f;
    }
    return (input - a) / (b - a);
  }

  if(d > b && d > c && a == b && b == c){
    if(input == d){
      return 0.0f;
    }
    return (d - input) / (d - c);
  }

  if(a < b && a < c && b == c && d > b && d > c){
    if(input > a && input <= b){
      return (input - a) / (b - a);
    }
    if(input > c && input <= d){
      return (d - input) / (d - c);
    }
    return 0.0f;
  }

  if(input > a && input <= b){
    return (input - a) / (b - a);
  }
  if(input >= b && input <= c){
    return 1.0f;
  }
  if(input > c && input <= d){
    return (d - input) / (d - c);
  }

  return 0.0f;
}

float calculateOutputTermCentroid(const FuzzyTerm& term){
  float total = 0.0f;

  // Matches the output weighting used by the original fuzzy calculation.
  for(int value = static_cast<int>(term.a); value <= term.d; value++){
    total += value;
  }

  return total / (term.d - term.a);
}

float calculateMotorSpeed(float potAngle, float usDistance){
  float centroid = 0.0f;
  uint8_t activeRuleCount = 0;

  for(uint8_t i = 0; i < RULE_COUNT; i++){
    const FuzzyRule& rule = RULES[i];
    float potMembership = calculateMembership(POT_TERMS[rule.potTerm], potAngle);
    float usMembership = calculateMembership(US_TERMS[rule.usTerm], usDistance);
    float ruleActivation = min(potMembership, usMembership);

    if(ruleActivation > 0.0f){
      const FuzzyTerm& outputTerm = MOTOR_TERMS[rule.motorTerm];
      centroid += calculateOutputTermCentroid(outputTerm) * ruleActivation;
      activeRuleCount++;
    }
  }

  if(activeRuleCount == 0){
    return 0.0f;
  }

  return centroid / activeRuleCount;
}

float readPotAngle(){
  int rawValue = analogRead(POT_PIN);
  return constrain(static_cast<float>(rawValue), POT_MIN, POT_MAX);
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
  int speedValue = constrain(static_cast<int>(roundf(speedAbs)), 0, static_cast<int>(MOTOR_MAX));
  int dutyCycle = map(speedValue, 0, static_cast<int>(MOTOR_MAX), MOTOR_MIN_DUTY_CYCLE, MOTOR_MAX_DUTY_CYCLE);
  int pwmValue = map(dutyCycle, 0, 100, 0, 255);

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
    dutyCycle = 0;
  }

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
}

void loop(){
  float potAngle = readPotAngle();
  float usDistance = readUltrasonicDistance();

  float motorSpeed = calculateMotorSpeed(potAngle, usDistance);
  setMotorOutput(motorSpeed);

  Serial.print("Pot=");
  Serial.print(potAngle);
  Serial.print(" US=");
  Serial.print(usDistance);
  Serial.print(" Motor=");
  Serial.println(motorSpeed);
}
