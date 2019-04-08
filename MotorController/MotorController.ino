//--------------------------------------------------------------------------------
// 4 MOTOR RUNNING CODE - VERSION 7 - TUNED
//--------------------------------------------------------------------------------

//INFORMATION: --> w = forward, s = backwards, a = left, d = right
//             --> q = shift right, e = shift left, f = shift forward, r = shift backward


//PWM Variables
unsigned int pwm_B = 150;
unsigned int pwm_L = 190;
unsigned int pwm_R = 190;
unsigned int pwm_F = 150;

unsigned int shiftpwm_B = 220;
unsigned int shiftpwm_L = 220;
unsigned int shiftpwm_R = 220;
unsigned int shiftpwm_F = 230;

#define rotatepwm_B 210
#define rotatepwm_L 240
#define rotatepwm_R 240
#define rotatepwm_F 240

unsigned long t_start;
unsigned long t_current;

//F is LEFT
//R is Front
//L is Right

//BOARD1w - FORWARD AND BACKWARD
#define PWMpinB 3 // back motor true is right a1
#define brakeMotorB 9
#define dirMotorB 12

#define PWMpinR 11 // left and false is forward b1
#define brakeMotorR 8 
#define dirMotorR 13 

//BOARD 2 - LEFT AND RIGHT

#define PWMpinF 6 // Right motor true is forward a2
#define dirMotorF 15 //An1
#define brakeMotorF 17 //An3

#define PWMpinL 10 // Forward motor and true is right b2
#define dirMotorL 14 //An0
#define brakeMotorL  16 // An2

//SHIFT TIMES
#define left_shift_time 60
#define right_shift_time 60
#define forward_shift_time 60
#define backward_shift_time 60

// REALIGN TIMES
#define realign_right_time 65
#define realign_left_time 65

//COMMANDS
char start_command;
char next_command;
char KylesClear;


void setup() {

  //MOTOR SHIELD 1 ON DIGITAL PINS
  pinMode(brakeMotorB, OUTPUT); //Initiates motor 1 channel A brake on board 1
  pinMode(brakeMotorL, OUTPUT); // Initiates motor 2 channel B brake on board 1
  pinMode(dirMotorB, OUTPUT); // Initiates motor 1 dir A on board 1
  pinMode(dirMotorL, OUTPUT); // Initiates motor 1 dir B on board 1

  //MOTORSHIELD 2 ON ANALOG PINS
  pinMode(dirMotorF, OUTPUT); //DIRECTION MOTOR B - BOARD 2
  pinMode(dirMotorR, OUTPUT); //DIRECTION MOTOR A - BOARD 2
  pinMode(brakeMotorF, OUTPUT); //BRAKE B - BOARD 2
  pinMode(brakeMotorR, OUTPUT); //BRAKE A - BOARD 2

  //Initiate Serial Monitor
  Serial.begin(9600);
  Serial.println("in Loop");
}


//--------------------------------------------------------------------------------
// TAKING IN COMMANDS LOOP
//--------------------------------------------------------------------------------

void loop() {

  if (Serial.available() > 0) start_command = Serial.read(); //GET STARTING COMMAND

  //MOVING FORWARDS
  if (start_command == 'w') {
    forward();
  }

  //MOVING BACKWARDS
  else if (start_command == 's') {
    backward();
  }

  //MOVING RIGHT
  else if (start_command == 'd') {
    right();
  }

  //MOVING LEFT
  else if (start_command == 'a') {
    left();
  }

  else if (start_command == 'q') {
    shiftLeft();
  }
  else if (start_command == 'e') {
    shiftRight();
  }
  else if (start_command == 'f') {
    shiftForward();
  }

  else if (start_command == 'r') {
    shiftBackward();
  }
  else if (start_command == 'x') {
    realignLeft();
  }

  else if (start_command == 'c') {
    realignRight();
  }


  //  Kyles_Clear = Serial.read();
}


//--------------------------------------------------------------------------------
// Cardinal Direction MOVEMENTS
//--------------------------------------------------------------------------------
void forward() {
  //Resets PWM and Start Driving
t_start = millis();

  //read next_command
  // if (Serial.available() > 0) next_command = Serial.read(); //GET NEXT COMMAND

  //Shift while driving
  while (next_command != 'z') { // wait for the stop command
    runR(pwm_R, false);
    runL(pwm_L, true);
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read();
    t_current = millis();
  }

  //Reset all commands and stop moving
  boardstop(); //STOP MOVING
  start_command = 'n';
  next_command = 'n';
}

void backward() {
t_start = millis();

  while (next_command != 'z') { // wait for the stop command
    runR(pwm_R, true);
    runL(pwm_L, false);
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read(); // read command if available
    t_current = millis();
  }

  boardstop(); //STOP MOVING
  start_command = 'n';
  next_command = 'n';
}

void left() {


  // if (Serial.available() > 0) next_command = Serial.read();// get next command
t_start = millis();

  while (next_command != 'z') {
    runF(pwm_F, true);
    runB(pwm_B, false);
    //read command
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read(); // read command if available
    t_current = millis();
  }

  boardstop();
  start_command = 'n';
  next_command = 'n';
}

void right() {
  //record straight angle


  //  if (Serial.available() > 0) next_command = Serial.read();
t_start = millis();

  while (next_command != 'z') {
    runF(pwm_F, false);
    runB(pwm_B, true);
    next_command = 'n'; //Neutral Command to do nothing
    if (Serial.available() > 0) next_command = Serial.read(); // read command if available
    t_current = millis();
  }

  boardstop();
  start_command = 'n';
  next_command = 'n';
}

//--------------------------------------------------------------------------------
// MOTOR RUN CODE -  - FORWARD AND BACKWARD
//--------------------------------------------------------------------------------

void runB(int speed, boolean rev) {
  //Channel A
  if (rev) digitalWrite(dirMotorB, LOW);
  else digitalWrite(dirMotorB, HIGH);

  digitalWrite(brakeMotorB, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMpinB, speed);   //Spins the motor on Channel A at full speed
}

void runL(int speed, boolean rev) {
  if (rev) digitalWrite(dirMotorL, LOW);
  else digitalWrite(dirMotorL, HIGH);

  digitalWrite(brakeMotorL, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMpinL, speed);   //Spins the motor on Channel A at full speed
}


//--------------------------------------------------------------------------------
// MOTOR RUN CODE - MOTORS ON BOARD 2 - LEFT AND RIGHT
// --------------------------------------------------------------------------------

void runR(int speed, boolean rev) {
  if (rev) digitalWrite(dirMotorR, LOW);
  else digitalWrite(dirMotorR, HIGH);

  digitalWrite(brakeMotorR, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWMpinR, speed);   //Spins the motor on Channel A at speed
}

void runF(int speed, boolean rev) {
  if (rev) digitalWrite(dirMotorF, LOW);
  else digitalWrite(dirMotorF, HIGH);

  digitalWrite(brakeMotorF, LOW);   //Disengage the Brake for Channel B
  analogWrite(PWMpinF, speed);   //Spins the motor on Channel B at speed
}


//--------------------------------------------------------------------------------
// STOPPING FUNCTIONS
//--------------------------------------------------------------------------------

//BOARD STOPPING FUNCTIONS - FORWARDS AND BACKWARDS
void boardstop() {
  digitalWrite(brakeMotorB, HIGH);
  digitalWrite(brakeMotorL, HIGH);
  digitalWrite(brakeMotorR, HIGH);
  digitalWrite(brakeMotorF, HIGH);


  runB(0, true);
  runL(0, true);
  runR(0, true);
  runF(0, true);
}


//--------------------------------------------------------------------------------
// SHIFTING FUNCTIONS
//--------------------------------------------------------------------------------

void shiftLeft() {

  boardstop();
  delay(100);

  runB(shiftpwm_B, false);
  runF(shiftpwm_F, true);
  delay(left_shift_time);
  boardstop();
  start_command = 'n'; // resets the start command
  while (Serial.available()) {
    KylesClear = Serial.read();
  }

}

void shiftRight() {

  boardstop();
  delay(100);

  runB(shiftpwm_B, true);
  runF(shiftpwm_F, false);
  delay(right_shift_time);
  boardstop();
  start_command = 'n'; // resets the start command
  while (Serial.available()) {
    KylesClear = Serial.read();
  }

}

void shiftForward() {

  boardstop();
  delay(100);

  runR(shiftpwm_R, false);
  runL(shiftpwm_L, true);
  delay(forward_shift_time);
  boardstop();
  start_command = 'n'; // resets the start command
  while (Serial.available()) {
    KylesClear = Serial.read();
  }
}

void shiftBackward() {

  boardstop();
  delay(100);

  runR(shiftpwm_R, true);
  runL(shiftpwm_L, false);
  delay(backward_shift_time);
  boardstop();
  start_command = 'n'; // resets the start command to neutral
  while (Serial.available()) {
    KylesClear = Serial.read();
  }
}


//--------------------------------------------------------------------------------
// REALIGNING FUNCTIONS
//--------------------------------------------------------------------------------

void realignRight() {
  boardstop();

  delay(100);
  runB(rotatepwm_B, false);
  runF(rotatepwm_F, false);
  runR(rotatepwm_R, true);
  runL(rotatepwm_L, true);
  delay(realign_right_time);
  boardstop();
  start_command = 'n'; // resets the start command to neutral
  while (Serial.available()) {
    KylesClear = Serial.read();
  }
}

void realignLeft() {
  boardstop();
  delay(100);
  runB(rotatepwm_B, true);
  runF(rotatepwm_F, true);
  runR(rotatepwm_R, false);
  runL(rotatepwm_L, false);
  delay(realign_left_time);
  boardstop();
  start_command = 'n'; // resets the start command to neutral
  while (Serial.available()) {
    KylesClear = Serial.read();
  }
}


//--------------------------------------------------------------------------------
// ARCHIVED CODE
//--------------------------------------------------------------------------------

////time in millis to execute each of the drive functions for 13.3V battery
//#define left_turning_time 650
//#define right_turning_time 700
//#define forwards_time 500
//#define backwards_time 500
//#define right_align_time 90
//#define left_align_time 90
