/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//include all necessary libraries
#include <Arduino.h>       //include for PlatformIO Ide
#include <AccelStepper.h>  //include the stepper motor library
#include <MultiStepper.h>  //include multiple stepper motor library
#include <RPC.h>

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = { 5, 6, 7 };  //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48  //stepper enable pin on stepStick
#define rtStepPin 50      //right stepper motor step pin
#define rtDirPin 51       // right stepper motor direction pin
#define ltStepPin 52      //left stepper motor step pin
#define ltDirPin 53       //left stepper motor direction pin

struct lidar {
  // this can easily be extended to contain sonar data as well
  int front;
  int back;
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right)
} dist;

// read_lidars is the function used to get lidar data to the M7
struct lidar read_lidars() {
  return dist;
}
// reads a lidar given a pin
int read_lidar(int pin) {
  int16_t t = pulseIn(pin, HIGH);
  if (t == 0 || t > 1850)
    return 100000;
  int d = (t - 1000) * 3 / 40;
  if (d < 0) { d = 0; } 
  return d;
}
void setupM4() {
  // bind a method to return the lidar data all at once
  RPC.bind("read_sensor", read_lidars);
}
void loopM4() {
  // update the struct with current lidar data
  dist.front = read_lidar(8);
  dist.back = read_lidar(9);
  dist.left = read_lidar(10);
  dist.right = read_lidar(11);
}

//define the Lidar constants
#define LIDAR_FRONT 0
#define LIDAR_BACK 1
#define LIDAR_LEFT 2
#define LIDAR_RIGHT 3
#define numOfSens 4

//define the behavior constants
#define NO_BEHAVIOR 0
#define COLLIDE 1

//define the Lidar variables
int16_t ft_lidar = 8;
int16_t bk_lidar = 9;
int16_t lt_lidar = 10;
int16_t rt_lidar = 11;
int16_t lidar_pins[numOfSens] = {8,9,10,11};
int16_t lidarDist[numOfSens] = {0,0,0,0};

//define the Sonar constants
#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // The ultrasonic velocity (cm/us) compensated by temperature
#define SONAR_RIGHT 0
#define SONAR_LEFT 1

//define the Sonar variables
int16_t rt_trigechoPin = 3;
int16_t lt_trigechoPin = 4;
int16_t trig_EchoPin[2] = { 3,4 };
int16_t sonarDist[2] = {0,0};

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 //create instance to control multiple steppers at the same time

#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor

int pauseTime = 2500;  //time before robot moves
int stepTime = 500;    //delay time between high and low on step pin
int wait_time = 1000;  //delay for printing data

#define LIDAR_POLL_TIME 500 //time between lidar polls in miseconds
int lidarTimer = 0; //timer to determine when to poll lidar sensors

//define encoder pins
#define LEFT 0                        //left encoder
#define RIGHT 1                       //right encoder
const int ltEncoder = 18;             //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;             //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = { 0, 0 };  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = { 0, 0 };          //variable to hold encoder speed (left, right)
int accumTicks[2] = { 0, 0 };         //variable to hold accumulated ticks since last reset

bool run = false;

// Helper Functions

//interrupt function to count left encoder tickes
void LwheelSpeed() {
  encoder[LEFT]++;  //count the right wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed() {
  encoder[RIGHT]++;  //count the right wheel encoder interrupts
}

void allOFF() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(leds[i], LOW);
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);                   //sets pin as output
  pinMode(rtDirPin, OUTPUT);                    //sets pin as output
  pinMode(ltStepPin, OUTPUT);                   //sets pin as output
  pinMode(ltDirPin, OUTPUT);                    //sets pin as output
  pinMode(stepperEnable, OUTPUT);               //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   //set enable LED as output
  digitalWrite(enableLED, LOW);                 //turn off enable LED
  pinMode(redLED, OUTPUT);                      //set red LED as output
  pinMode(grnLED, OUTPUT);                      //set green LED as output
  pinMode(ylwLED, OUTPUT);                      //set yellow LED as output
  digitalWrite(redLED, HIGH);                   //turn on red LED
  digitalWrite(ylwLED, HIGH);                   //turn on yellow LED
  digitalWrite(grnLED, HIGH);                   //turn on green LED
  delay(pauseTime / 5);                         //wait 0.5 seconds
  digitalWrite(redLED, LOW);                    //turn off red LED
  digitalWrite(ylwLED, LOW);                    //turn off yellow LED
  digitalWrite(grnLED, LOW);                    //turn off green LED

  stepperRight.setMaxSpeed(500);              //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(500);          //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(500);               //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(500);           //set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);           //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);            //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               //turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                            //print manager timer
  if (millis() - timer > 100) {                              //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                         //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                       //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];     //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT];  //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;   //clear the left encoder data buffer
    encoder[RIGHT] = 0;  //clear the right encoder data buffer
    timer = millis();    //record current time since program started
  }
}



/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed() {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {}
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop(int behavior) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (behavior == 1){
      for (int i = 0;i<4;i++){
        if (readLidar(i) <= 5) {
          leftStopped = 1;
          stepperLeft.stop();  //stop left motor
          rightStopped = 1;
          stepperRight.stop();  //stop right motor
        }
      }
      for (int i = 0;i<2;i++){
        if (readSonar(i) <= 5) {
          leftStopped = 1;
          stepperLeft.stop();  //stop left motor
          rightStopped = 1;
          stepperRight.stop();  //stop right motor
        }
      }
    }
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();  //stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();  //stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}


/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void spin(int angle, int dir, int behavior) {
  int steps = angle * 5.585;
  if (dir) {
    stepperLeft.move(steps);    //move one full rotation forward relative to current position
    stepperRight.move(-steps);  //move one full rotation forward relative to current position
  } else {
    stepperRight.move(steps);  //move one full rotation forward relative to current position
    stepperLeft.move(-steps);  //move one full rotation forward relative to current position
  }
  runToStop(behavior);  //run until the robot reaches the target
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void pivot(int angle, int dir, int behavior) {
  int steps = angle * 5.585 * 2;
  if (dir) {
    stepperLeft.move(steps);  //move steps
  } else {
    stepperRight.move(steps);
  }

  runToStop(behavior);  //run until the robot reaches the target
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void turn(int time, int dir, int behavior) {
  int steps = time * 500;

  if (dir) {
    stepperLeft.setMaxSpeed(500);
    stepperRight.setMaxSpeed(250);
    stepperLeft.move(steps);       //move one full rotation forward relative to current position
    stepperRight.move(steps / 2);  //move one full rotation forward relative to current position
  } else {
    stepperRight.setMaxSpeed(500);
    stepperLeft.setMaxSpeed(250);
    stepperRight.move(steps);     //move one full rotation forward relative to current position
    stepperLeft.move(steps / 2);  //move one full rotation forward relative to current position
  }

  runToStop(behavior);  //run until the robot reaches the target
  init_stepper();
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void forward(int steps, int behavior) {
  // int steps = distance / 0.034375; // for distance in cm
  stepperRight.move(steps);  //move steps forward relative to current position
  stepperLeft.move(steps);   //move steps forward relative to current position

  runToStop(behavior);               //run until the robot reaches the target
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void reverse(int distance, int behavior) {
  int steps = distance / 0.034375;
  stepperRight.move(-steps);  //move one full rotation reverse relative to current position
  stepperLeft.move(-steps);   //move one full rotation reverse relative to current position
  runToStop(behavior);                //run until the robot reaches the target
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void stop() {
  stepperRight.setSpeed(0);  //set right motor speed
  stepperLeft.setSpeed(0);   //set left motor speed
}

//this function will read the left or right sensor based upon input value
int readLidar(uint16_t side) {
  int16_t t = pulseIn(lidar_pins[side], HIGH);
  int d; //distance to  object
  if (t == 0){
    // pulseIn() did not detect the start of a pulse within 1 second.
    //Serial.println("timeout");
    d = 100000; //no object detected
  }
  else if (t > 1850)  {
    //Serial.println("timeout");
    d = 100000; //no object detected
  }
  else  {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    d = (t - 1000) * 3 / 40;
 
    // Limit minimum distance to 0.
    if (d < 0) { d = 0; } 
  }
  // Serial.println(d);
  // Serial.print(" cm, ");
  return d;
}

//this function will read the left or right sensor based upon input value
uint16_t readSonar(uint16_t side) {
  uint16_t distance;
  uint32_t pulseWidthUs;
  int16_t dist, temp, dist_in;

  pinMode(trig_EchoPin[side], OUTPUT);
  digitalWrite(trig_EchoPin[side], LOW);
  digitalWrite(trig_EchoPin[side], HIGH);  //Set the trig pin High
  delayMicroseconds(10);               //Delay of 10 microseconds
  digitalWrite(trig_EchoPin[side], LOW);   //Set the trig pin Low
  pinMode(trig_EchoPin[side], INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(trig_EchoPin[side], HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * VELOCITY_TEMP(20) / 2.0;  //The distance can be calculated according to the flight time of ultrasonic wave,/
                                                      //and the ultrasonic sound speed can be compensated according to the actual ambient temperature
  dist_in = 0.394*distance;    //convert cm to inches
  // Serial.print(dist_in, DEC);   //print inches
  // Serial.print(" inches ");                                          
  // Serial.print(distance, DEC);  //print cm
  // Serial.println(" cm");
  return distance;
}

void randomWander(int behavior) {
  digitalWrite(grnLED, HIGH);      //turn on green LED
  digitalWrite(ylwLED, LOW);       //turn off yellow LED
  digitalWrite(redLED, LOW);       //turn off red LED

  int angle = random(20, 180);
  int dir = random(0,2);

  spin(angle, dir, behavior);

  int distance = random(4000);

  forward(distance, behavior);

}

void collide(void) {
  digitalWrite(grnLED, LOW);      //turn on green LED
  digitalWrite(ylwLED, LOW);       //turn off yellow LED
  digitalWrite(redLED, HIGH);       //turn off red LED

  stepperRight.setSpeed(200);  //set right motor speed
  stepperLeft.setSpeed(200);   //set left motor speed  

  if (lidarTimer < millis() - LIDAR_POLL_TIME) {
    run = true;
    lidarTimer = millis();
    for (int i = 0;i<4;i++){
      int dist = readLidar(i);
      if (dist <= 10) {
        run = false;
      }
    }
  }

  // for (int i = 0;i<1;i++){
  //   if (readSonar(i) <= 10) {
  //     run = false;
  //   }
  // }

  if (run) {
    runAtSpeed();
    // Serial.println("run");
  }
}

void runaway(void) {
  digitalWrite(grnLED, LOW);      //turn on green LED
  digitalWrite(ylwLED, HIGH);       //turn off yellow LED
  digitalWrite(redLED, LOW);       //turn off red LED

  int maxSpeed = 200;
  int rightSpeed;
  int leftSpeed;
  int x;
  int y;

  dist = RPC.call("read_lidars").as<struct lidar>();

  if (abs(dist.back) < 30 && abs(dist.front) < 30) {
    x = dist.front - dist.back; // x direction of repulsive vector
  } else if (abs(dist.back) < 30) {
    x = 30 - dist.back; // x direction of repulsive vector
  } else if (abs(dist.front) < 30) {
    x = -30 + dist.front; // x direction of repulsive vector
  } else {
    x = 0;
  }

  if (abs(dist.left) < 30 && abs(dist.right) < 30) {
    y = dist.left - dist.right; // x direction of repulsive vector
  } else if (abs(dist.right) < 30) {
    y = 30 - dist.right; // x direction of repulsive vector
  } else if (abs(dist.left) < 30) {
    y = -30 + dist.left; // x direction of repulsive vector
  } else {
    y = 0;
  }

  int angle = atan2(y,x) * 180 / 3.1415;

  // Serial.print("x = ");
  // Serial.print(x);
  // Serial.print(" y = ");
  // Serial.print(y);
  // Serial.print(" angle = ");
  // Serial.println(angle);

  if (abs(x) > 5  || abs (y) > 5) {
    if (angle > -45 && angle <= 45) {
      rightSpeed = maxSpeed;
      leftSpeed = maxSpeed;
    } else if ((angle > 45 && angle <= 90) || (angle > -135 && angle <= -90)) {
      rightSpeed = maxSpeed;
      leftSpeed = -maxSpeed;
    } else if ((angle > -90 && angle <= -45) || (angle > 90 && angle <= 135)) {
      rightSpeed = -maxSpeed;
      leftSpeed = maxSpeed;
    } else {
      rightSpeed = -maxSpeed;
      leftSpeed = -maxSpeed;
    }
  } else {
    rightSpeed = 0;
    leftSpeed = 0;
  }

  // if (angle <= 90 && angle >= -90) {
  //   rightSpeed = maxSpeed * abs((angle + 90)) / 180; 
  //   leftSpeed = maxSpeed * abs((angle - 90)) / 180;
  // } else {
  //   rightSpeed = -maxSpeed * abs((angle + 90)) / 180; 
  //   leftSpeed = -maxSpeed * abs((angle - 90)) / 180;
  // }

  stepperRight.setSpeed(rightSpeed);  //set right motor speed
  stepperLeft.setSpeed(leftSpeed);   //set left motor speed  

  runAtSpeed(); 
}

void setup() {
  RPC.begin();
  if(HAL_GetCurrentCPUID() == CM7_CPUID) {
    // if on M7 CPU, run M7 setup & loop
    setupM7();
    while(1) loopM7();
  } else {
    // if on M4 CPU, run M7 setup & loop
    setupM4();
    while(1) loopM4();
  }
}

// loop() is never called as setup() never returns
void loop() {}

//// MAIN
void setupM7() {
  int baudrate = 9600;  //serial monitor baud rate'
  init_stepper();       //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);  //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);  //init the interrupt mode for the right encoder

  for (int i = 0; i<numOfSens;i++){
    pinMode(lidar_pins[i],OUTPUT);
  }

//  Serial.begin(baudrate);  //start serial monitor communication

//   while (!Serial) {
//     delay(10);  // will pause until serial console opens
//   }
//   Serial.println("Robot starting...Put ON TEST STAND");
//   delay(pauseTime);  //always  wait 2.5 seconds before the robot moves
}


void loopM7() {
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  // print_encoder_data();   //prints encoder data

  //collide();

  runaway();

  //delay(wait_time);               //wait to move robot or read data
}