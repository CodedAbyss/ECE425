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

//includew all necessary libraries
#include <Arduino.h>       //include for PlatformIO Ide
#include <AccelStepper.h>  //include the stepper motor library
#include <MultiStepper.h>  //include multiple stepper motor library

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

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 //create instance to control multiple steppers at the same time

#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor

int pauseTime = 2500;  //time before robot moves
int stepTime = 500;    //delay time between high and low on step pin
int wait_time = 1000;  //delay for printing data

//define encoder pins
#define LEFT 0                        //left encoder
#define RIGHT 1                       //right encoder
const int ltEncoder = 18;             //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;             //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = { 0, 0 };  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = { 0, 0 };          //variable to hold encoder speed (left, right)
int accumTicks[2] = { 0, 0 };         //variable to hold accumulated ticks since last reset


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

  stepperRight.setMaxSpeed(1500);              //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(1000);          //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);               //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(1000);           //set desired acceleration in steps/s^2
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
void runAtSpeed(void) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop(void) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
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
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  Serial.println("move1 function");
  digitalWrite(redLED, HIGH);    //turn on red LED
  digitalWrite(grnLED, LOW);     //turn off green LED
  digitalWrite(ylwLED, LOW);     //turn off yellow LED
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000);                  // One second delay
  digitalWrite(ltDirPin, LOW);  // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW);  // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000);  // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  Serial.println("move2 function");
  digitalWrite(redLED, LOW);          //turn off red LED
  digitalWrite(grnLED, HIGH);         //turn on green LED
  digitalWrite(ylwLED, LOW);          //turn off yellow LED
  stepperRight.moveTo(800);           //move one full rotation forward relative to current position
  stepperLeft.moveTo(800);            //move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);        //set right motor speed
  stepperLeft.setSpeed(1000);         //set left motor speed
  stepperRight.runSpeedToPosition();  //move right motor
  stepperLeft.runSpeedToPosition();   //move left motor
  runToStop();                        //run until the robot reaches the target
  delay(1000);                        // One second delay
  stepperRight.moveTo(0);             //move one full rotation backward relative to current position
  stepperLeft.moveTo(0);              //move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);        //set right motor speed
  stepperLeft.setSpeed(1000);         //set left motor speed
  stepperRight.runSpeedToPosition();  //move right motor
  stepperLeft.runSpeedToPosition();   //move left motor
  runToStop();                        //run until the robot reaches the target
  delay(1000);                        // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  Serial.println("move3 function");
  digitalWrite(redLED, LOW);   //turn off red LED
  digitalWrite(grnLED, LOW);   //turn off green LED
  digitalWrite(ylwLED, HIGH);  //turn on yellow LED
  long positions[2];           // Array of desired stepper positions
  positions[0] = 800;          //right motor absolute position
  positions[1] = 800;          //left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();  // Blocks until all are in position
  delay(1000);                    //wait one second
  // Move to a different coordinate
  positions[0] = 0;  //right motor absolute position
  positions[1] = 0;  //left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();  // Blocks until all are in position
  delay(1000);                    //wait one second
}

/*this function will move to target at 2 different speeds*/
void move4() {
  Serial.println("move4 function");
  int leftPos = 5000;   //right motor absolute position
  int rightPos = 1000;  //left motor absolute position
  int leftSpd = 5000;   //right motor speed
  int rightSpd = 1000;  //left motor speed

  digitalWrite(redLED, HIGH);  //turn on red LED
  digitalWrite(grnLED, HIGH);  //turn on green LED
  digitalWrite(ylwLED, LOW);   //turn off yellow LED

  //Uncomment the next 4 lines for absolute movement
  stepperLeft.setCurrentPosition(0);   //set left wheel position to zero
  stepperRight.setCurrentPosition(0);  //set right wheel position to zero
  stepperLeft.moveTo(leftPos);         //move left wheel to absolute position
  stepperRight.moveTo(rightPos);       //move right wheel to absolute position

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);    //move left wheel to relative position
  stepperRight.move(rightPos);  //move right wheel to relative position

  //Uncomment the next two lines to set the speed
  stepperLeft.setSpeed(leftSpd);    //set left motor speed
  stepperRight.setSpeed(rightSpd);  //set right motor speed
  runAtSpeedToPosition();           //run at speed to target position
}

/*This function will move continuously at 2 different speeds*/
void move5() {
  Serial.println("move5 function");
  digitalWrite(redLED, LOW);        //turn off red LED
  digitalWrite(grnLED, HIGH);       //turn on green LED
  digitalWrite(ylwLED, HIGH);       //turn on yellow LED
  int leftSpd = 250;                //right motor speed
  int rightSpd = 500;               //left motor speed
  stepperLeft.setSpeed(leftSpd);    //set left motor speed
  stepperRight.setSpeed(rightSpd);  //set right motor speed
  runAtSpeed();
}

/*
  goToAngle takes in an angle in degress, and the robot rotates itself that many degrees about the center of the robot.
  This uses encoder control.
*/
void goToAngle(int angle) {
  //A wheel travels 27.5cm per revolution
  //A wheel travels 69.1cm per 360 spin
  //There are 800 steps per wheel revolution (quarter stepping)
  //69.1/27.5*800 = 2010.6 steps per 360 spin
  digitalWrite(grnLED, HIGH);   //turn on green LED

  int eCounts = abs(angle / 3.45);
  int speed = 300;

  if (angle < 0) {
    stepperLeft.setSpeed(speed);  //set left motor speed
    stepperRight.setSpeed(-speed);  //set right motor speed
    Serial.println("neg");
  } else {
    stepperLeft.setSpeed(-speed);  //set left motor speed
    stepperRight.setSpeed(speed);  //set right motor speed
  }

  // run the motors until they travel to the desired count
  while (encoder[RIGHT] - eCounts < 0 || encoder[LEFT] - eCounts < 0) {
    stepperRight.runSpeed();
    stepperLeft.runSpeed();
  }

  encoder[RIGHT] = 0;
  encoder[LEFT] = 0;

  digitalWrite(grnLED, LOW);       //turn off green LED
}

/*
  goToGoal travels to an x/y coordinate relative to the bot's position and orientation. This uses encoder control.
  The forward axis of the robot is x, and the perpendicular axis (side to side) is y. units are inches.
*/
void goToGoal(int x, int y) {
  digitalWrite(grnLED, HIGH);       // turn on green LED
  digitalWrite(ylwLED, HIGH);       // turn on yellow LED

  int angle;

  angle = atan2(y, x)*180/3.1415;   // use atan2 to calculate the angle the robot must rotate

  goToAngle(angle);                 // rotate the robot
  digitalWrite(grnLED, HIGH);       // turn on green LED

  delay(1000);

  double distance = sqrt(pow(x,2) + pow(y,2)); // calculate the distance the robot must move forward
  int eCounts = distance / 10.8 * 40;
  int speed = 300;

  stepperLeft.setSpeed(speed);      // set left motor speed
  stepperRight.setSpeed(speed);     // set right motor speed

  while (encoder[RIGHT] - eCounts < 0 || encoder[LEFT] - eCounts < 0) { // drive until distance has been reached
    stepperRight.runSpeed();
    stepperLeft.runSpeed();
  }

  encoder[RIGHT] = 0;
  encoder[LEFT] = 0;

  digitalWrite(grnLED, LOW);       //turn off green LED
  digitalWrite(ylwLED, LOW);       //turn off yellow LED
}


/*
  squarePath makes the robot travel in a square with a given side length. length is in inches
*/
void squarePath(int length) {
  digitalWrite(grnLED, HIGH);       // turn on green LED
  digitalWrite(ylwLED, HIGH);       // turn on yellow LED
  digitalWrite(redLED, HIGH);       // turn on red LED
  goToGoal(length, 0);
  digitalWrite(grnLED, HIGH);       // turn on green LED
  digitalWrite(ylwLED, HIGH);       // turn on yellow LED
  delay(1000);
  goToGoal(0,length);
  digitalWrite(grnLED, HIGH);       // turn on green LED
  digitalWrite(ylwLED, HIGH);       // turn on yellow LED
  delay(1000);
  goToGoal(0, length);
  digitalWrite(grnLED, HIGH);       // turn on green LED
  digitalWrite(ylwLED, HIGH);       // turn on yellow LED
  delay(1000);
  goToGoal(0, length);
  digitalWrite(redLED, LOW);
}


/*
  spin takes an angle and dir, and spins the robot about it's center to that angle. There is no encoder control.
  Dir is a flag (LEFT or RIGHT) to define which direction the spin is in.
  The angle is the amount of degrees it will spin
*/
void spin(int angle, int dir) {
  int steps = angle * 5.585;
  if (dir) {
    stepperLeft.move(steps);    // move one full rotation forward relative to current position
    stepperRight.move(-steps);  // move one full rotation forward relative to current position
  } else {
    stepperRight.move(steps);  // move one full rotation forward relative to current position
    stepperLeft.move(-steps);  // move one full rotation forward relative to current position
  }
  runToStop();  //run until the robot reaches the target
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void pivot(int angle, int dir) {
  int steps = angle * 5.585 * 2;
  if (dir) {
    stepperLeft.move(steps);  //move steps
  } else {
    stepperRight.move(steps);
  }

  runToStop();  //run until the robot reaches the target
}

/*
  turn takes a time and dir, and turns the robot in the given direction for a desired amount of time. There is no encoder control.
  time is in seconds.
  Dir is a flag (LEFT or RIGHT) to define which direction the spin is in.
*/
void turn(int time, int dir) {
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

  runToStop();  //run until the robot reaches the target
  init_stepper();
}

/*
  move the robot fowards to a given distance in inches
*/
void forward(int distance) {
  int steps = distance / 0.034375;
  stepperRight.move(steps);  //move steps forward relative to current position
  stepperLeft.move(steps);   //move steps forward relative to current position
  runToStop();               //run until the robot reaches the target
}
/*
  move the robot backwards to a given distance in inches
*/
void reverse(int distance) {
  int steps = distance / 0.034375;
  stepperRight.move(-steps);  //move one full rotation reverse relative to current position
  stepperLeft.move(-steps);   //move one full rotation reverse relative to current position
  runToStop();                //run until the robot reaches the target
}
/*
  stop the robot (stop both motors)
*/
void stop() {
  stepperRight.setSpeed(0);  //set right motor speed
  stepperLeft.setSpeed(0);   //set left motor speed
  stepperRight.stop();
  stepperLeft.stop();
}




/*
  move in a circle, dir sets the direction of the turn (LEFT or RIGHT) and the diameter is in inches
*/
void moveCircle(int diam, int dir) {
  // diam:radius = 2:1, in:cm = 1:2.54
  float radius_cm = diam * 1.27f;

  // 800 steps per revolution : 8.75*Pi cm = 29.1 steps / cm
  // steps to travel = turn radius * 2Pi * 29.1
  // 2Pi * 29.1 = 182.84f
  float steps_inner = (radius_cm - 11.0f) * 182.84f;
  float steps_outer = (radius_cm + 11.0f) * 182.84f;

  float max_vel = 1000;
  
  Serial.print(steps_inner);
  Serial.print(", ");
  Serial.println(steps_outer);
  
  if(dir == LEFT) { // if turning left, right side goes max speed and left is slower
    stepperLeft.setMaxSpeed(steps_inner / steps_outer * max_vel);
    stepperRight.setMaxSpeed(max_vel);
    stepperLeft.move(steps_inner);
    stepperRight.move(steps_outer);
  } else { // if turning right, left side goes max speed and right is slower
    stepperLeft.setMaxSpeed(max_vel);
    stepperRight.setMaxSpeed(steps_inner / steps_outer * max_vel);
    stepperLeft.move(steps_outer);
    stepperRight.move(steps_inner);
  }

  int running = 3;
  while(running) { // run the motors until both stop
    if(!stepperLeft.run()) {
      running &= 1;
      stepperLeft.stop();
    }
    if(!stepperRight.run()) {
      running &= 2;
      stepperRight.stop();
    }
  }
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(int diam) {
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  moveCircle(diam, LEFT);
  moveCircle(diam, RIGHT);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
}



//// MAIN
void setup() {
  int baudrate = 9600;  //serial monitor baud rate'
  init_stepper();       //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);  //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);  //init the interrupt mode for the right encoder

  Serial.begin(baudrate); // start serial monitor communication if available
  delay(1000); // wait for connection, timeout after 1 second
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime);  //always wait 2.5 seconds before the robot moves
}


void loop() {
  //uncomment each function one at a time to see what the code does
  //move1();//call move back and forth function
  //move2();//call move back and forth function with AccelStepper library functions
  //move3();//call move back and forth function with MultiStepper library functions
  //move4(); //move to target position with 2 different speeds
  //move5(); //move continuously with 2 different speeds

  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  // print_encoder_data();   //prints encoder data

  //moveFigure8(36);

  //forward(30.5);

  //delay(10000);

  //reverse(30.5);

  //delay(10000);

  // goToAngle(45);
  // delay(5000);
  // goToAngle(-60);
  // delay(5000);
  // goToGoal(36,36);
  // delay(5000);

  // goToAngle(-90);
  // delay(5000);

  // goToGoal(36,-48);
  // delay(5000);

  // goToAngle(90);
  // delay(5000);
  // squarePath(36);
  // delay(5000);

  //delay(wait_time);               //wait to move robot or read data
}
