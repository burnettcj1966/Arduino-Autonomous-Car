#include <IRremote.h>
#include <Servo.h>
#include <stdio.h>


/*ultrasonic sensor*/
#define recvPin 12
#define echoPin A4
#define trigPin A5
long duration, inches, standardDistance, results;

/*motor controls*/
#define leftSide 5
#define rightSide 6
#define F1 7
#define R1 8
#define R2 9
#define F2 11
int maxSpeed = 250;

/*various movement modes of the car */
enum moveModes {
  forward,
  backward,
  left,
  right,
  halt,
  AIleft,
  AIright,
  AIspin,
} moveMode = halt;

/*Servo Motor */
Servo servo;
#define servoPin 3

long IRTimeMillis, AITimeMillis;
bool AIMode = false;

void setup() {
  Serial.begin(9600);
  /*Setup Motor Control */
  pinMode(F1, OUTPUT);
  pinMode(F2, OUTPUT);
  pinMode (R1, OUTPUT);
  pinMode (R2, OUTPUT);
  pinMode(leftSide, OUTPUT);
  pinMode(rightSide, OUTPUT);
  /*Setup IR Control*/
  IrReceiver.begin(recvPin, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
  /*Setup Ultrasonic Sensor*/
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  /*Setup Servo Motor*/
  servo.attach(servoPin);
  servo.write(90);

}

void loop() {
  checkIR();
  if (AIMode == true) checkAI();
  moveCar();
}

long getRange() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    inches = (duration/2) /74;
    results = results + inches;
  }
  results = results / 10;
  return results;
}


void checkAI() {
  long detection = getRange();
  if (detection < 12) {
    moveMode = halt;
    moveCar();

    /*checks left range*/
    servo.attach(3);
    servo.write(180);
    delay(500);
    long leftDetection = getRange();

    /*checks right range*/
    servo.attach(3);
    servo.write(0);
    delay(500);
    servo.detach();
    long rightDetection = getRange();

    /*reset servo to normal angle*/
    servo.attach(3);
    servo.write(90);
    delay(500);
    servo.detach();

    if (leftDetection > 12 && rightDetection > 12) {
      if (leftDetection > rightDetection) {
        turnLeft();
      }
      else {
        turnRight();
      }
    }
    else if (leftDetection > 12) {
      turnLeft();
    }
    else if (rightDetection > 12) {
      turnRight();
    }
    else {
      spin();
    }
  }
  moveMode = forward;
}


void turnLeft() {
  long start = millis();
  moveLeft();
  while(true) {
    if (millis() - start > 300) {
      analogWrite(leftSide, 0);
      analogWrite(rightSide, 0);
      break;
    }
  }
}

void turnRight() {
  long start = millis();
  moveRight();
  while (true) {
    if (millis() - start > 300) {
      analogWrite(leftSide, 0);
      analogWrite(rightSide, 0);
      break;
    }
  }
}

void spin() {
  long start = millis();
  moveRight();
  while (true) {
    if (millis() - start > 600) {
      analogWrite(leftSide, 0);
      analogWrite(rightSide, 0);
      break;
    }
  }
}

void moveCar() {
  switch (moveMode) {
    case forward:
      moveForward();
      break;
    case backward:
      moveBackward();
      break;
    case left:
      moveLeft();
      break;
    case right:
      moveRight();
      break;
    case halt:
      moveHalt();
      break;
  }
  if (millis() - IRTimeMillis > 250) {
    moveMode = halt;
    IRTimeMillis = millis();
  }
}

void checkIR() {
  if (IrReceiver.decode()) {
    int IRValue = IrReceiver.decodedIRData.command;
    IRTimeMillis = millis();
    Serial.println(IRValue);
    switch(IRValue) {
      case 64:
        moveMode = halt;
        AIMode = false;
        break;
      case 70:
        moveMode = forward;
        AIMode = false;
        break;
      case 21:
        moveMode = backward;
        AIMode = false;
        break;
      case 68:
        moveMode = left;
        AIMode = false;
        break;
      case 67:
        moveMode = right;
        AIMode = false;
        break;
      case 22:
        AIMode = true;
        break;
    } 
    IrReceiver.resume();
  }
}

/*Code below manages the movement of the car*/

/*moves car forward*/
void moveForward() {
  analogWrite(leftSide, maxSpeed);
  analogWrite(rightSide, maxSpeed);
  digitalWrite(F1, HIGH);
  digitalWrite(F2, HIGH);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
}

/*moves car backward*/
void moveBackward() {
  analogWrite(leftSide, maxSpeed);
  analogWrite(rightSide, maxSpeed);
  digitalWrite(F1, LOW);
  digitalWrite(F2, LOW);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, HIGH);
}

/*moves car left*/
void moveLeft() {
  analogWrite(leftSide, maxSpeed);
  analogWrite(rightSide, maxSpeed);
  digitalWrite(F1, LOW);
  digitalWrite(F2, HIGH);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
}

/*moves car right*/
void moveRight() {
  analogWrite(leftSide, maxSpeed);
  analogWrite(rightSide, maxSpeed);
  digitalWrite(F1, HIGH);
  digitalWrite(F2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void moveHalt() {
  analogWrite(leftSide, maxSpeed);
  analogWrite(rightSide, maxSpeed);
  digitalWrite(F1, LOW);
  digitalWrite(F2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
}
