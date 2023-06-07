#include <Arduino.h>

#include <AFMotor.h> //add Adafruit Motor Shield library
#include <Servo.h>   //add Servo Motor library
#include <NewPing.h> //add Ultrasonic sensor library

#define TRIG_PIN A0                              // Pin A0 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN A2                              // Pin A1 on the Motor Drive Shield soldered to the ultrasonic sensor
#define SERVO_PIN 10                             // Pin 10 on the Motor Drive Shield soldered to the servo
#define MAX_DISTANCE 300                         // sets maximum useable sensor measuring distance to 300cm
#define MAX_SPEED 160                            // sets speed of DC traction motors to 150/250 or about 70% of full speed - to get power drain down.
#define MAX_SPEED_OFFSET 40                      // this sets offset to allow for differences between the two DC traction motors
#define COLL_DIST 20                             // sets distance at which robot stops and reverses to 30cm
#define TURN_DIST COLL_DIST + 10                 // sets distance at which robot veers away from object
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sets up sensor library to use the correct pins to measure distance.

AF_DCMotor leftMotor1(4, MOTOR12_1KHZ);  // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor leftMotor2(1, MOTOR12_1KHZ);  // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor1(3, MOTOR34_1KHZ); // create motor #3, using M3 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor2(2, MOTOR34_1KHZ); // create motor #4, using M4 output, set to 1kHz PWM frequency

Servo myservo; // create servo object to control a servo

long static currMode = 0;

int leftDistance, rightDistance; // distances on either side
int curDist = 0;
String motorSet = "";
int speedSet = 0;

void modeChange();

//-------------------------------------------------------------------------------------------------------------------------------------

int readPing()
{ // read the ultrasonic sensor distance
  delay(70);
  unsigned int uS = sonar.ping();
  int cm = uS / US_ROUNDTRIP_CM;
  return cm;
}

//-------------------------------------------------------------------------------------------------------------------------------------
void moveStop()
{
  leftMotor1.run(RELEASE);
  leftMotor2.run(RELEASE);
  rightMotor1.run(RELEASE);
  rightMotor2.run(RELEASE);
} // stop the motors.

//-------------------------------------------------------------------------------------------------------------------------------------
void moveForward()
{
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);                                // turn it on going forward
  leftMotor2.run(FORWARD);                                // turn it on going forward
  rightMotor1.run(FORWARD);                               // turn it on going forward
  rightMotor2.run(FORWARD);                               // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet);
    rightMotor2.setSpeed(speedSet);
    delay(5);
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------
void moveBackward()
{
  motorSet = "BACKWARD";
  leftMotor1.run(BACKWARD);                               // turn it on going backward
  leftMotor2.run(BACKWARD);                               // turn it on going backward
  rightMotor1.run(BACKWARD);                              // turn it on going backward
  rightMotor2.run(BACKWARD);                              // turn it on going backward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet);
    rightMotor2.setSpeed(speedSet);
    delay(5);
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight()
{
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);   // turn motor 1 forward
  leftMotor2.run(FORWARD);   // turn motor 2 forward
  rightMotor1.run(BACKWARD); // turn motor 3 backward
  rightMotor2.run(BACKWARD); // turn motor 4 backward
  rightMotor1.setSpeed(speedSet + MAX_SPEED_OFFSET);
  rightMotor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
  delay(1500); // run motors this way for 1500
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD); // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);
}

//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft()
{
  motorSet = "LEFT";
  leftMotor1.run(BACKWARD); // turn motor 1 backward
  leftMotor2.run(BACKWARD); // turn motor 2 backward
  leftMotor1.setSpeed(speedSet + MAX_SPEED_OFFSET);
  leftMotor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
  rightMotor1.run(FORWARD); // turn motor 3 forward
  rightMotor2.run(FORWARD); // turn motor 4 forward

  delay(500); // run motors this way for 1500
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);  // turn it on going forward
  leftMotor2.run(FORWARD);  // turn it on going forward
  rightMotor1.run(FORWARD); // turn it on going forward
  rightMotor2.run(FORWARD); // turn it on going forward
}

//-------------------------------------------------------------------------------------------------------------------------------------
void turnAround()
{
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);   // turn motor 1 forward
  leftMotor2.run(FORWARD);   // turn motor 2 forward
  rightMotor1.run(BACKWARD); // turn motor 3 backward
  rightMotor2.run(BACKWARD); // turn motor 4 backward
  rightMotor1.setSpeed(speedSet + MAX_SPEED_OFFSET);
  rightMotor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
  delay(1700); // run motors this way for 1700
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD); // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);
}

void compareDistance() // find the longest distance
{
  if (leftDistance > rightDistance) // if left is less obstructed
  {
    turnLeft();
  }
  else if (rightDistance > leftDistance) // if right is less obstructed
  {
    turnRight();
  }
  else // if they are equally obstructed
  {
    turnAround();
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

void changePath()
{
  moveStop();        // stop forward movement
  myservo.write(20); // check distance to the right
  delay(500);
  rightDistance = readPing(); // set right distance
  delay(500);
  myservo.write(160); // check distace to the left
  delay(700);
  leftDistance = readPing(); // set left distance
  delay(500);
  myservo.write(90); // return to center
  delay(100);
  compareDistance();
}

//-------------------------------------------------------------------------------------------------------------------------------------
void remoteControlViaBlueTooth()
{
  Serial.println("Remote control started ...");
  while (true)
  {
    if (Serial.available() > 0)
    {
      long value = Serial.read();
      Serial.println(value);

      if(value == 'A' || value == 'V') {

        if(value == 'A'){
          currMode = 0;
        }else{
          currMode = 2;
        }

        break;
      }

      if (value == 49)
      {
        moveForward();
      }
      else if (value == 50)
      {
        moveBackward();
      }
      else if (value == 51)
      {
        turnRight();
      }
      else if (value == 52)
      {
        turnLeft();
      }
      else if (value == 53)
      {
        turnAround();
      }
      else if (value == 48)
      {
        moveStop();
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

void voiceControlViaBlueTooth()
{
  Serial.println("Voice control started ...");
  while (true)
  {
    if (Serial.available() > 0)
    {
      long value = Serial.read();
      if (value == 'R' || value == 'A')
      {
        if(value == 'R'){
          currMode = 1;
        }else{
          currMode = 0;
        }

        break;
      }

      if (value == 'f')
      {
        moveForward();
      }
      else if (value == 'b')
      {
        moveBackward();
      }
      else if (value == 'r')
      {
        turnRight();
      }
      else if (value == 'l')
      {
        turnLeft();
      }
      else if (value == 'a')
      {
        turnAround();
      }
      else if (value == 's')
      {
        moveStop();
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

void autoMaticMode()
{
  Serial.println("Automatic mode started ...");
  while (true)
  {
    if (Serial.available() > 0)
    {
      int mode = Serial.read();
      Serial.println(mode);
      if (mode == 'R' || mode == 'V')
      {
        if(mode == 'R'){
          currMode = 1;
        }else{
          currMode = 2;
        }
        break;
      }
    }
    
    
    myservo.write(90); // move eyes forward
    delay(90);
    curDist = readPing(); // read distance the current distance in cm
    if (curDist < COLL_DIST)
    {
      changePath();
    }              // if forward is blocked change direction
    moveForward(); // move forward
    delay(500);
  }
}

//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600); // set up Serial library at 9600 bps
  myservo.attach(10); // attaches the servo on pin 10 (SERVO_1 on the Motor Drive Shield to the servo object
  myservo.write(90);  // tells the servo to position at 90-degrees ie. facing forward.

  Serial.println("Program started ...");

  currMode = 0;

  delay(1000); // delay for one seconds
}
//------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
void loop()
{
  // if (Serial.available() > 0)
  // {
  //   int mode = Serial.read();
  //   if (mode == 'A')
  //   {
  //     currMode = 0;
  //   }
  //   else if (mode == 'R')
  //   {
  //     currMode = 1;
  //   }
  //   else if (mode == 'V')
  //   {
  //     currMode = 2;
  //   }
  // }

  if (currMode == 0)
  {
    autoMaticMode();
  }
  else if (currMode == 1)
  {
    remoteControlViaBlueTooth();
  }
  else if (currMode == 2)
  {
    voiceControlViaBlueTooth();
  }
}
