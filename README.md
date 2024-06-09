# Autonomous Vehicular Navigator (AVN) A

#include <SoftwareSerial.h>
#include <Servo.h>
#include <AFMotor.h>

SoftwareSerial BTSerial(A1, A0);

#define Echo A2
#define Trig A3
#define motor 10
#define Speed 255
#define spoint 103
#define IRSensor A4
#define BuzzerPin 9  
char value;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;


Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}
void loop() {
  //forward();
 //Obstacle();
  Bluetoothcontrol();
 //voicecontrol();
}

void Bluetoothcontrol() {
  if (BTSerial.available() > 0) {
    value = BTSerial.read();
    Serial.println(value);
  }
  if (value == 'F') {
    forward();
  } else if (value == 'B') {
    backward();

  } else if (value == 'R') {
    left();

  } else if (value == 'L') {
    right();
  } else if (value == 'G') {
    forward();
    right();
  } else if (value == 'I') {
    forward();
    left();
  } else if (value == 'H') {
    backward();
    right();
  } else if (value == 'J') {
    backward();
    left();
  } else if (value == '1') {
#define Speed 150
  } else if (value == 'V') {
    digitalWrite(BuzzerPin, HIGH);
    delay(300);
    digitalWrite(BuzzerPin, LOW);
    delay(300);
    digitalWrite(BuzzerPin, HIGH);
    delay(300);
    digitalWrite(BuzzerPin, LOW);
    delay(300);
    digitalWrite(BuzzerPin, HIGH);
    delay(300);
    digitalWrite(BuzzerPin, LOW);
  } else if (value == 'X') {
    int leftObstacle = leftsee();  
    servo.write(spoint);          
    delay(800);                   
    int rightObstacle = rightsee();  
    servo.write(spoint);            
  } else if (value == 'V') {
    digitalWrite(BuzzerPin, HIGH); 
    delay(500);
    digitalWrite(BuzzerPin, LOW);
  } else if (value == 'S') {
    Stop();
  }
}
void Obstacle() {
  distance = ultrasonic();

  if (distance < 40) {
    Stop();
    digitalWrite(BuzzerPin, HIGH);
    delay(300);
    digitalWrite(BuzzerPin, LOW);
    delay(100);
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      left();
      delay(900);
      Stop();
      delay(200);
    } else if (L > R) {
      right();
      delay(900);
      Stop();
      delay(200);
    }
  } else {
    forward();
  }
}

void voicecontrol() {
  distance = ultrasonic();
  if (distance < 30) {
    Stop();
    digitalWrite(BuzzerPin, HIGH);
    delay(300);
    digitalWrite(BuzzerPin, LOW);
    delay(1500);
    backward();
    delay(1000);
    right();
    delay(1000);
    Stop();
   
  }
  if (BTSerial.available() > 0) {
    value = BTSerial.read();
    Serial.println(value);

    if (value == 'U') {
      forward();
    } else if (value == 'D') {
      backward();
    } else if (value == 'L') {
      left();
    } else if (value == 'R') {
      right();
    } else if (value == 'S') {
      Stop();
    }
  }
}

int ultrasonic() {
  delay(20);
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  Serial.println(t);
  long cm = t / 29 / 2;  
  return cm;
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void right() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void left() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}
int rightsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}

int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}

 
