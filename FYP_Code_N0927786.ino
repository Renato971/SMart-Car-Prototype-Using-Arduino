#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

// Define pin connections
const int trigPin = 9;
const int echoPin = 10;
const int motor1Pin1 = 2;
const int motor1Pin2 = 3;
const int motor2Pin1 = 4;
const int motor2Pin2 = 5;
const int motorSpeed = 255;
const int irPin = 11;

//line tracking
const int lineSensorLeft = A0;
const int lineSensorRight = A1;

// Define distances and speeds
const int safeDistance = 20;
const int backupTime = 500;
const int turnTime = 250;

// Define IR remote codes
const unsigned int forward_code = 0xFF629D;
const unsigned int back_code = 0xFF22DD;
const unsigned int left_code = 0xFF02FD;
const unsigned int right_code = 0xFFC23D;
const unsigned int stop_code = 0xFFA857;
const unsigned int lineTrack_code = 0xFFE21D;
const int lineSensorThreshold = 500; // set threshold value for line sensor

int Echo = 12;  
int Trig = 13; 
int mode=0;//different modes of working of vehicle

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define carSpeed 120
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

void forward(){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void back() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}

void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left");
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 

void setup() { 
  mode=0;
  // Start serial communication
  Serial.begin(9600);
  myservo.attach(10);  // attach servo on pin 3 to servo object    
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(irPin, INPUT);
  pinMode(lineSensorLeft, INPUT);
  pinMode(lineSensorRight, INPUT);
  stop();
} 



  void Remote () {
      // Check for IR remote commands
    if (Serial.available()) {
      unsigned int code = Serial.read();
      if (code == forward_code) {
        forward();  
      } else if (code == back_code) {
        back();
      } else if (code == left_code) {
        left();
      } else if (code == right_code) {
        right();
      } else if (code == stop_code) {
        stop();
      } else if (code == lineTrack_code) {
        line_tracking();
        }
      }
    }
    
  //Ultrasonic distance measurement Sub function
  int Distance_test() {
    digitalWrite(Trig, LOW);   
    delayMicroseconds(20);
    digitalWrite(Trig, HIGH);  
    delayMicroseconds(20);
    digitalWrite(Trig, LOW);   
    float Fdistance = pulseIn(Echo, HIGH);  
    Fdistance= Fdistance / 58;       
    Serial.println(Fdistance);
    return (int)Fdistance;
  }  

  void obstacle_avoidance () {
    // Send ultrasonic pulse
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    // Measure echo duration
    long duration = pulseIn(Echo, HIGH);
    // Calculate distance
    int distance = duration / 58;
    // Print distance to serial monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    // Check distance and take action
    if (distance <= safeDistance) {
      // Back up
      back();
      delay(backupTime);
      // Turn
      right();
      delay(turnTime);
    }
      myservo.write(90);  //setservo position according to scaled value
      delay(500); 
      middleDistance = Distance_test();

      if(middleDistance <= 50) {     
        stop();
        delay(500);                         
        myservo.write(10);          
        delay(1000);      
        rightDistance = Distance_test();
        
        delay(500);
        myservo.write(90);              
        delay(1000);                                                  
        myservo.write(180);              
        delay(1000); 
        leftDistance = Distance_test();
        
        delay(500);
        myservo.write(90);              
        delay(1000);
        if(rightDistance > leftDistance) {
          right();
          delay(360);
        }
        else if(rightDistance < leftDistance) {
          left();
          delay(360);
        }
        if((rightDistance <= 50) && (leftDistance <= 50)) {
          back();
          delay(360);
          stop();
          delay(500);
        }
        else {
          forward();
        }
      }  
      else {
          forward();
      } 
  }

void line_tracking(){
    // Check line tracking sensors
    int leftSensorValue = analogRead(lineSensorLeft);
    int rightSensorValue = analogRead(lineSensorRight);
    if (leftSensorValue > lineSensorThreshold && rightSensorValue > lineSensorThreshold) {
      // No line detected, dont move
      stop();
    } else if (leftSensorValue <= lineSensorThreshold && rightSensorValue > lineSensorThreshold) {
      // Follow line to the right
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);
      analogWrite(motorSpeed, motorSpeed);
    } else if (leftSensorValue > lineSensorThreshold && rightSensorValue <= lineSensorThreshold) {
      // Follow line to the left
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      analogWrite(motorSpeed, motorSpeed);
    } else if (leftSensorValue <= lineSensorThreshold && rightSensorValue <= lineSensorThreshold) {
      // Follow line straight
      forward();
    }
  }

void loop() { 

  switch (mode){
    case 0: 
      obstacle_avoidance ();
      break;

    case 1: 
      Remote ();
      break;

    case 2: 
      line_tracking ();
      break;    
  }
}