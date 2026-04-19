#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ===== ENCODEURS =====
const byte encLA = 2;
const byte encLB = 18;
const byte encRA = 3;
const byte encRB = 19;

volatile long countL = 0;
volatile long countR = 0;

// ===== MOTEURS =====
  //MOTEUR DE GAUCHE
const int ENA = 5; //PWMA 
const int IN1 = 4; //AIN1
const int IN2 = 7; //AIN2

  //MOTEUR DE DROITE
const int ENB = 6;  //PWMB
const int IN3 = 8;  //BIN1
const int IN4 = 9;  //BIN2

const int STBY = 22;

// ===== PARAMETRES ROBOT =====
const float wheelDiameter = 65.0;
const float wheelCircumference = wheelDiameter * 3.1416;
const float pulsesPerTurn = 3840.0;
const float distPerPulse = wheelCircumference / pulsesPerTurn;

// ===== GEOMETRIE ROBOT =====
const float wheelBase = 112.0;

// ===== DISTANCES =====
const float forward1 = 755.0; //au lieu de 705
const float forward2 = 20.0;
const float backward1 = 20.0;
const float forward3 = 1290.0;
const float forward4 = 603.0; //553

// ===== ROTATIONS =====
const float turn45  = (3.1416 * wheelBase * 45.0) / 360.0;
const float turn45_bias = (3.1416 * wheelBase * 60.0) / 360.0;
const float turn90  = (3.1416 * wheelBase * 90.0) / 360.0;
const float turn135 = (3.1416 * wheelBase * 135.0) / 360.0;
const float turn270 = (3.1416 * wheelBase * 290.0) / 360.0;

// ===== SERVOS =====
Servo servo1;
Servo servo2;

const int servoPin1 = A1;
const int servoPin2 = 10;

const int anglepoulet1 = 65;
const int anglepoulet2 = 20;
const int angleCentre = 90;
const int angleGauche = 40;
const int angleDroite = 140;

// ===== LED =====
#define LED_PIN1 12
#define LED_PIN2 13
#define NUMPIXELS 8

Adafruit_NeoPixel strip1(NUMPIXELS, LED_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(NUMPIXELS, LED_PIN2, NEO_GRB + NEO_KHZ800);

// ===== VITESSES =====
int baseSpeed = 190;
int slowSpeed = 130;
float leftMotorBias = 1.1;

// ===== ETATS =====
enum State {
  WAIT_START,
  FORWARD1,
  TURN1,
  FORWARD2,
  WAIT_SERVO1,
  SERVO1,
  WAIT_SERVO2,
  BACKWARD1,
  TURN2,
  WAIT_LED1,
  LED,
  WAIT_LED2,
  TURN3,
  FORWARD3,
  TURN4,
  WAIT_SERVO3,
  SERVO2,
  WAIT_SERVO4,
  TURN5,
  FORWARD4,
  TURN6,
  STOPPED
};

State state = WAIT_START;

// ===== TIMER =====
unsigned long stateStart = 0;

// ===== ENCODEURS =====
void readEncoderL() {
  static uint8_t lastState = 0;
  uint8_t A = digitalRead(encLA);
  uint8_t B = digitalRead(encLB);
  uint8_t currentState = (A << 1) | B;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00))
    countL++;
  else
    countL--;

  lastState = currentState;
}

void readEncoderR() {
  static uint8_t lastState = 0;
  uint8_t A = digitalRead(encRA);
  uint8_t B = digitalRead(encRB);
  uint8_t currentState = (A << 1) | B;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00))
    countR++;
  else
    countR--;

  lastState = currentState;
}

// ===== RESET ENCODEURS =====
void resetEncoders() {
  noInterrupts();
  countL = 0;
  countR = 0;
  interrupts();
}

// ================================================================
// MOTEURS
// ================================================================
void forward(int sL, int sR) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, constrain((int)(sL * leftMotorBias), 0, 255));
//  analogWrite(ENA, sL);
  analogWrite(ENB, sR);
}

void backward(int speedL, int speedR) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

void rotateLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 130);
  analogWrite(ENB, 130);
}

void rotateRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 130);
  analogWrite(ENB, 130);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ===== AVANCER =====
void moveStraight(float target, float avgDistance) {
  int speed = baseSpeed;
  if (target - avgDistance < 80)
    speed = slowSpeed;

  long diff = abs(countL) - abs(countR);
  int correction = diff * 0.5;
  correction = constrain(correction, -30, 30);

  int speedL = constrain(speed - correction, 0, 255);
  int speedR = constrain(speed + correction, 0, 255);

  forward(speedL, speedR);
}

// ===== RECULER =====
void moveBackward(float target, float avgDistance) {
  int speed = baseSpeed;
  if (target - avgDistance < 80)
    speed = slowSpeed;

  backward(speed, speed);
}

// ===========SETUP=======================
void setup() {
  Serial.begin(115200);

  pinMode(encLA, INPUT_PULLUP);
  pinMode(encLB, INPUT_PULLUP);
  pinMode(encRA, INPUT_PULLUP);
  pinMode(encRB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encLA), readEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLB), readEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRA), readEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRB), readEncoderR, CHANGE);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  servo1.attach(servoPin1);
  //servo2.attach(servoPin2);

  servo1.write(anglepoulet1);
//  servo2.write(angleCentre);

  strip1.begin();
  strip2.begin();
  strip1.show();
  strip2.show();
   
  stateStart = millis();

  Serial.begin(115200);
  Serial.println("Start");
}

// ==============LOOP====================================
void loop() {
  float distL = abs(countL) * distPerPulse;
  float distR = abs(countR) * distPerPulse;
  float avgDistance = (distL + distR) / 2.0;

  switch(state) {

    case WAIT_START:
      stopMotors();
      if (millis() - stateStart > 1500) {
        resetEncoders();
        state = FORWARD1;
      }
      break;

    case FORWARD1:
      moveStraight(forward1, avgDistance);
      if (avgDistance >= forward1) {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = TURN1;
      }
      break;

    case TURN1:
      if (avgDistance < turn45)
        rotateLeft();
      else {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = FORWARD2;
      }
      break;

    case FORWARD2:
      moveStraight(forward2, avgDistance);
      if (avgDistance >= forward2) {
        stopMotors();
        stateStart = millis();
        state = WAIT_SERVO1;
      }
      break;

    case WAIT_SERVO1:
      if (millis() - stateStart >= 1000) {
        stateStart = millis();
        state = SERVO1;
      }
      break;

    case SERVO1:
      if (millis() - stateStart > 500) {
        servo1.write(anglepoulet2);
        delay(500);
        servo1.write(anglepoulet1);
        delay(500);
        resetEncoders();
        stateStart = millis();
        state = WAIT_SERVO2;
      }
      break;

    case WAIT_SERVO2:
      if (millis() - stateStart >= 1000) {
        resetEncoders();
        stateStart = millis();
        state = BACKWARD1;
      }
      break;

    case BACKWARD1:
      moveBackward(backward1, avgDistance);
      if (avgDistance >= backward1) {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = TURN2;
      }
      break;

    case TURN2:
      rotateLeft();
      if (avgDistance >= turn90) {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = WAIT_LED1;
      }
      break;

    case WAIT_LED1:
      if (millis() - stateStart >= 1000) {
        resetEncoders();
        stateStart = millis();
        state = LED;
      }
      break;

    case LED:
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < NUMPIXELS; i++) {
          strip1.setPixelColor(i, strip1.Color(0, 255, 0));
          strip2.setPixelColor(i, strip2.Color(0, 255, 0));
        }
        strip1.show();
        strip2.show();
        delay(500);
        strip1.clear();
        strip2.clear();
        strip1.show();
        strip2.show();
        delay(1000);
      }
      resetEncoders();
      stateStart = millis();
      state = WAIT_LED2;
      break;

    case WAIT_LED2:
      if (millis() - stateStart >= 1000) {
        resetEncoders();
        stateStart = millis();
        state = TURN3;
      }
      break;

    case TURN3:
      if (avgDistance < turn45_bias)
        rotateRight();
      else {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = FORWARD3;
      }
      break;

    case FORWARD3:
      moveStraight(forward3, avgDistance);
      if (avgDistance >= forward3) {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = TURN4;
      }
      break;

    case TURN4:
      if (avgDistance < turn135)
        rotateLeft();
      else {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = WAIT_SERVO3;
      }
      break;

    case WAIT_SERVO3:
      if (millis() - stateStart >= 1000) {
        resetEncoders();
        stateStart = millis();
        state = SERVO2;
      }
      break;

    case SERVO2:
      servo2.attach(servoPin2);
    
      for (int i = 0; i < 3; i++) {
        // Gauche
        servo2.write(angleGauche);
        delay(400);
    
        // Droite
        servo2.write(angleDroite);
        delay(400);
    
        // Retour gauche (complète le cycle)
        servo2.write(angleGauche);
        delay(400);
      }
    
      servo2.write(angleCentre);
      delay(300);
      servo2.detach();
    
      resetEncoders();
      stateStart = millis();
      state = WAIT_SERVO4;
      break;

    case WAIT_SERVO4:
      if (millis() - stateStart >= 25000) { 
        resetEncoders();
        stateStart = millis();
        state = TURN5;
      }
      break;

    case TURN5:
      if (avgDistance < turn45_bias)
        rotateRight();
      else {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = FORWARD4;
      }
      break;

    case FORWARD4:
      moveStraight(forward4, avgDistance);
      if (avgDistance >= forward4) {
        stopMotors();
        resetEncoders();
        stateStart = millis();
        state = TURN6;
      }
      break;

    case TURN6:
      if (avgDistance < turn270)
        rotateRight();
      else {
        stopMotors();
        state = STOPPED;
      }
      break;

    case STOPPED:
      stopMotors();
      break;
  }
}
