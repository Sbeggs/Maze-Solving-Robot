


bool facing = true; //true is LZ one two, false is LZ three four
int loadingZone = 1;
int dropoffZone = 2;
int sidesOpenFacing = 13;
int sidesOpenNotFacing = 7;
//FACINGS: Dropoff 1 - 13, dropoff 2 - 13, dropoff 3 - 13, dropoff 4 - 7
//NOT FACINGS: Dropoff 1 - 7, dropoff 2 - 7, dropoff 3 - 7, dropoff 4 is 13


//--------------------------------------------------------------------------------
// MODE CODE - WITH HANDSHAKE AND FINDING BLOCK
//--------------------------------------------------------------------------------

//INFORMATION: --> Modified consistent readings to ignore big changes in the value unless the change persists for two readings in a row
//             --> Heading is bad, shifts might affect it
//

//DECLARING NEW SERIAL COMMUNICATION
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Servo.h>


SoftwareSerial sendUno(10, 11); //Rx/Tx

//SERVO VARIABLES
double downAngle = 110;
double downHalfwayAngle = 40;
double closeAngle = 55;
double openAngle;
double upAngle = 0;
Servo openClose;
Servo upDown;
int servoUpDownPin = 2;
int servoOpenClosePin = 3;


//PINS FOR ULTRASONIC SENSORS
#define TrigRB 36
#define EchoRB 37
#define TrigLB 34
#define EchoLB 35

#define TrigFR 26
#define EchoFR 27
#define TrigBR 28 //was 28
#define EchoBR 29 //was 29

#define TrigFL 24
#define EchoFL 25
#define TrigBL 30
#define EchoBL 31

#define TrigRF 32
#define EchoRF 33
#define TrigLF 38
#define EchoLF 39

#define TrigRFb 50
#define EchoRFb 51
#define TrigLFb 52
#define EchoLFb 53

//SENSOR READING VALUES
#define WaitForEcho 10 // how long the US sensors wait for the echo (in microseconds)
#define WackValue 100000 // the Value above which we consider the value not real
#define DurationCheck 40 // Check to see if old duration is different enough from current reading
#define openDistance 11.5 // Open sides
#define MAX_DISTANCE 500
int DisArray[8];

//WALL AVOIDANCE VARIABLES
#define minMovableDistance 19.5 // was 20
#define continueDistance 8.5 // was 9
#define stoppingDistance 8.5
#define littleDriveLimit 17.5 // was 12.5
#define minWallDistanceF 6.5 // was 4.5 #define minWallDistanceFB 6.5 
#define minWallDistanceB 4.5
#define minWallDistanceLR 4.5
bool shiftNeeded;

int sendSideOpen = 30;
int sendSideClosed = 15;


#define shiftTime 250

//REALIGN VARIABLES
#define minimumDifference 7 // was 15
#define realignDistance 1.5 // was 2.5
#define realignCap 11.5

//INCREMENTAL MOVEMENT VARIABLES
#define driveTime 200
#define littleDriveShift 300
unsigned long t_start;
unsigned long t_current;
bool backwards = false;
bool drivingDirection = true; //true if moving front back, false if left right

//LOCALISATION VARIABLES
int sidesOpen;
int oldSidesOpen = 20;
#define closeSide 10
#define largeOpenSpace 25

//HEADING
int heading = 0; //0 is forward, 1 is right, 2 is left and 3 is backwards
int didntDrive = 0;
char mode = 'm';

//USABLE SENSOR VALUES VARIABLES
int RF;
int LF;
int FR;
int FL;
int BR;
int BL;
int RB;
int LB;
int RFb;
int LFb;

int oldRF;
int oldLF;
int oldFR;
int oldBR;
int oldFL;
int oldBL;
int oldRB;
int oldLB;

int oldRFb;
int oldLFb;

//FOLLOW A PATH VARIABLES
char path[20];
char pathFromLZ [8];
char directionChar;

//DISTANCE ARRAY VARIABLES

// DisArray is indexed as follows *0* is Forward, *1* is  Forward Right, *2* is Forward Left, *3* Rear R, *4* Rear L, *5* is backwards
char Names [16] = {'R', 'F', 'L', 'F', 'F', 'R', 'F', 'L', 'B', 'R', 'B', 'L', 'R', 'B', 'L', 'B'}; // names of Values for`` read outs, just for debugging

//ULTRASONIC SENSOR FUNCTIONAL VARIABLES
long durationRF;
long durationLF;
long durationFR;
long durationBR;
long durationFL;
long durationBL;
long durationRB;
long durationLB;
long durationRFb;
long durationLFb;

NewPing sonar1(TrigRF, EchoRF, MAX_DISTANCE);
NewPing sonar2(TrigLF, EchoLF, MAX_DISTANCE);
NewPing sonar3(TrigRB, EchoRB, MAX_DISTANCE);
NewPing sonar4(TrigLB, EchoLB, MAX_DISTANCE);
NewPing sonar5(TrigFR, EchoFR, MAX_DISTANCE);
NewPing sonar6(TrigBR, EchoBR, MAX_DISTANCE);
NewPing sonar7(TrigFL, EchoFL, MAX_DISTANCE);
NewPing sonar8(TrigBL, EchoBL, MAX_DISTANCE);
NewPing sonar9(TrigRFb, EchoRFb, MAX_DISTANCE);
NewPing sonar10(TrigLFb, EchoLFb, MAX_DISTANCE);

//FINDING AND PICKUP UP BLOCK VARIABLES
int LEDpinLeft = 6;
int LEDpinRight = 5;
bool blockSide;
int i = 0;
double numberOfSteps;
int averageStepDistance = 2;

char KylesClear;
int counter = 0;

//VARIABLES FOR SIDES OPEN
int rightFree = 0;
int leftFree = 0;
int frontFree = 0;
int backFree = 0;


int huggingWallDistance = 4.5;
bool closestWall;
#define adventureTime 180
bool blockFound;
int holdRFb;
int holdLFb;
int holdRF;
int tolerance = 7.5;
int distanceFromBlock;

int blockOrientSides = 50;

int desiredRF;
int desiredLF;
int desiredBR;
int desiredFR;
int desiredFL;
int desiredBL;

bool pickedUpBlock = false;
bool relocalize = false;


void setup() {

  Serial.begin(9600);
  sendUno.begin(9600);
  Serial3.begin(9600);

  openClose.attach(servoOpenClosePin);
  upDown.attach(servoUpDownPin);
  //
  upDown.write(0); //0 is up, 100 is all the way down
  //  delay(3000);
  openClose.write(30);
  //  delay(3000);
  //  upDown.write(0);


  pinMode(LEDpinLeft, OUTPUT);
  pinMode(LEDpinRight, OUTPUT);
  digitalWrite(LEDpinLeft, LOW);
  digitalWrite(LEDpinRight, LOW);

  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  //  upDown.detach();
  //  openClose.detach();

  //  while (!Serial3.available()) {
  //
  //  }
  //
  //  mode = Serial3.read();

  delay(3000);

}


void loop() {

  if (mode == 'L') {
    if (!pickedUpBlock) {
      Serial3.println("Heading to block");
      followPath();
    }
    else if (pickedUpBlock) {
      Serial3.println("Heading to dropoff");
      dropoffRun();
    }
  }

  else {
    Move();
    delay(150);
    reactSides();
    reactAlignment();
    upDown.write(0);
  }

}


//--------------------------------------------------------------
// MAIN FUNCTIONS
//--------------------------------------------------------------

void followPath() {

  doCommand(path[0]);
  upDown.write(0); // everytime we reach the end of our path command we make sure its up
  delay(100);
  doCommand(path[1]);
  delay(100);
  upDown.write(0);
  doCommand(path[2]);
  delay(100);
  upDown.write(0);
  doCommand(path[3]);
  delay(100);
  upDown.write(0);
  doCommand(path[4]);
  delay(100);


  readSensorValues();
  destinationCheck();

  mode = 'n';

  if (relocalize == false) {

    mode = 'L';

    doCommand(path[5]);
    delay(100);

    findBlock();

    for (i = 0; i < 20; i++) {
      digitalWrite(LEDpinRight, LOW);
      digitalWrite(LEDpinLeft, HIGH);
      delay(150);
      digitalWrite(LEDpinRight, HIGH);
      digitalWrite(LEDpinLeft, LOW);
      delay(150);
    }

    pickedUpBlock = true;
    mode = 'n';
  }
}

void dropoffRun() {

  doCommand(path[7]);
  delay(100);
  doCommand(path[8]);
  delay(100);
  doCommand(path[9]);
  delay(100);
  doCommand(path[10]);
  delay(100);
  doCommand(path[11]);
  delay(100);

  readSensorValues();
  dropoffCheck();

  mode = 'n';

  if (relocalize == false) {
    mode = 'L';

    delay(500);

    //putDownBlock();
    openClose.write(0);
    delay(100);

    while (1) {
      digitalWrite(LEDpinRight, LOW);
      digitalWrite(LEDpinLeft, HIGH);
      delay(150);
      digitalWrite(LEDpinRight, HIGH);
      digitalWrite(LEDpinLeft, LOW);
      delay(150);
    }
  }
}




//--------------------------------------------------------------
// HEADING DECISION MAKING FUNCTIONS
//--------------------------------------------------------------
void Move() {

  readSensorValues();
  //  Serial.println(heading);

  if (heading == 0) goingForward();
  else if (heading == 1) goingRight();
  else if (heading == 2) goingLeft();
  else if (heading == 3) goingBackward();
  sendSidesOpen(heading);

}


void goingForward() {

  //  Serial.println("Going Forward");
  if (RF > continueDistance && LF > continueDistance) {
    driveForward();
    drivingDirection = true;
    heading = 0;
  }
  else if (FR > minMovableDistance && BR > minMovableDistance) {
    driveRight();
    drivingDirection = false;
    heading = 1;
  }
  else if (FL > minMovableDistance && BL > minMovableDistance) {
    driveLeft();
    drivingDirection = false;
    heading = 2;
  }
  else if (RB > minMovableDistance && LB > minMovableDistance) {
    driveBackward();
    drivingDirection = true;
    heading = 3;
  }
}

void goingRight() {
  //Serial.println("Going Right");
  if (FR > continueDistance && BR > continueDistance) {
    driveRight();
    drivingDirection = false;
    heading = 1;
  }
  else if (RF > minMovableDistance && LF > minMovableDistance) {
    driveForward();
    drivingDirection = true;
    heading = 0;
  }
  else if (RB > minMovableDistance && LB > minMovableDistance) {
    driveBackward();
    drivingDirection = true;
    heading = 3;
  }
  else heading = 0;
}

void goingLeft() {
  //Serial.println("Going Left");
  if (FL > continueDistance && BL > continueDistance) {
    driveLeft();
    drivingDirection = false;
    heading = 2;
  }
  else if (RF > minMovableDistance && LF > minMovableDistance) {
    driveForward();
    drivingDirection = true;
    heading = 0;
  }
  else if (RB > minMovableDistance && LB > minMovableDistance) {
    driveBackward();
    drivingDirection = true;
    heading = 3;
  }
  else heading = 0;
}

void goingBackward() {
  //Serial.println("Going Back");

  if (RB > continueDistance && LB > continueDistance) {
    driveBackward();
    drivingDirection = true;
    heading = 3;
  }
  else if (FR > minMovableDistance && BR > minMovableDistance) {
    driveRight();
    drivingDirection = false;
    heading = 1;
  }
  else if (FL > minMovableDistance && BL > minMovableDistance) {
    driveLeft();
    drivingDirection = false;
    heading = 2;
  }
  else heading = 0;
}


void reactAlignment() {

  readSensorValues();
  if (drivingDirection) reactAlignmentRL();
  else if (!drivingDirection) reactAlignmentFB();

}

void reactSides() {

  readSensorValues();
  if (drivingDirection) reactSidesRL();
  else if (!drivingDirection) reactSidesFB();

}


//--------------------------------------------------------------
// DRIVING FUNCTIONS
//--------------------------------------------------------------

void driveForward() {
  //Serial.println("w");
  //Serial3.println("w");
  sendUno.println("w");

  t_start = millis();

  while (RF > stoppingDistance && LF > stoppingDistance) { // Could it be an issue with the discrepancy betwen stopping distance and continuing distance
    if (checkSidesRL()) {
      break;
    }
    if (checkAlignmentRL()) {
      break;
    }
    readSensorValues();

    t_current = millis();
    if (t_current - t_start >= driveTime) break;
  }

  //    Serial.print("z");
  sendUno.print("z");
}

void driveBackward() {
  Serial.println("s");
  sendUno.println("s");
  //Serial3.println("s");

  t_start = millis();

  while (RB > stoppingDistance && LB > stoppingDistance) {
    if (checkSidesRL()) break;
    if (checkAlignmentRL()) break;
    readSensorValues();

    t_current = millis();
    if (t_current - t_start >= driveTime) break;
  }

  // Serial.print("z");
  sendUno.print("z");
}

void driveRight() {
  Serial.println("d");
  sendUno.println("d");
  //Serial3.println("d");

  t_start = millis();

  while (FR > stoppingDistance && BR > stoppingDistance) {
    if (checkSidesFB()) break;
    if (checkAlignmentFB()) break;
    readSensorValues();

    t_current = millis();
    if (t_current - t_start >= driveTime) break;
  }

  // Serial.print("z");
  sendUno.print("z");
}

void driveLeft() {
  Serial.println("a");
  sendUno.println("a");
  //Serial3.println("a");

  t_start = millis();

  while (FL > stoppingDistance && BL > stoppingDistance) {
    if (checkSidesFB()) break;
    if (checkAlignmentFB()) break;
    readSensorValues();

    t_current = millis();
    if (t_current - t_start >= driveTime) break;
  }

  // Serial.print("z");
  sendUno.print("z");
}


//--------------------------------------------------------------
// STAY AWAY FROM SIDES FUNCTIONS
//--------------------------------------------------------------

void reactSidesRL() {

  if (FR < minWallDistanceLR || BR < minWallDistanceLR) {
    delay(shiftTime);
    Serial.print("q"); //SHIFT LEFT
    sendUno.print("q");
    delay(shiftTime);
  }

  else if (FL < minWallDistanceLR || BL < minWallDistanceLR) {
    delay(shiftTime);
    Serial.print("e"); //SHIFT RIGHT
    sendUno.print("e");
    delay(shiftTime);
  }

}

void reactSidesFB() {

  if (RF < minWallDistanceF || LF < minWallDistanceF) { // have one for front, have one for back
    delay(shiftTime);
    Serial.print("r"); //SHIFT BACKWARD
    sendUno.print("r");
    delay(shiftTime);
  }
  else if (RB < minWallDistanceB || LB < minWallDistanceB) {
    delay(shiftTime);
    Serial.print("f"); //SHIFT FORWARD
    sendUno.print("f");
    delay(shiftTime);
  }

}

bool checkSidesRL() {

  readSensorValues();

  if (FR < minWallDistanceLR || BR < minWallDistanceLR) {
    return true;
  }
  else if (FL < minWallDistanceLR || BL < minWallDistanceLR) {
    return true;
  }
  else return false;

}

bool checkSidesFB() {

  readSensorValues();

  if (RF < minWallDistanceF || LF < minWallDistanceF) {
    return true;
  }
  else if (RB < minWallDistanceB || LB < minWallDistanceB) {
    return true;
  }
  else return false;

}


//--------------------------------------------------------------
// REALIGNMENT FUNCTIONS
//--------------------------------------------------------------

void reactAlignmentRL() {


  //  if (FL < realignCap && BL < realignCap && FR < realignCap && BR < realignCap){           ALWAYS REALIGN TO CLOSEST WALL USING THIS
  //    if (FL < FR) {
  //
  //    }
  //  }

  if (FL < realignCap && BL < realignCap) {
    if ((abs(FL - BL) > realignDistance)) { //(abs(FL - BL) < minimumDifference) &&
      if (FL > BL) {
        delay(shiftTime);
        Serial.print("x");
        sendUno.print("x");
        delay(shiftTime);
      }
      else {
        delay(shiftTime);
        Serial.print("c");
        sendUno.print("c");
        delay(shiftTime);
      }
      delay(200);
    }
  }

  else if (FR < realignCap && BR < realignCap) {
    if ((abs(FR - BR) > realignDistance)) { //(abs(FR - BR) < minimumDifference) &&
      if (FR > BR) {
        delay(shiftTime);
        Serial.print("c");
        sendUno.print("c");
        delay(shiftTime);
      }
      else {
        delay(shiftTime);
        Serial.print("x");
        sendUno.print("x");
        delay(shiftTime);
      }
    }
    delay(200);
  }
}

void reactAlignmentFB() {
  if (RF < realignCap && LF < realignCap) {
    if ((abs(RF - LF) > realignDistance)) { //(abs(FL - BL) < minimumDifference) &&
      if (RF > LF) {
        delay(shiftTime);
        Serial.print("x");
        sendUno.print("x");
        delay(shiftTime);
      }
      else {
        delay(shiftTime);
        Serial.print("c");
        sendUno.print("c");
        delay(shiftTime);
      }
    }
  }

  else if (RB < realignCap && LB < realignCap) {
    if ((abs(RB - LB) > realignDistance)) { //(abs(FR - BR) < minimumDifference) &&
      if (RB > LB) {
        delay(shiftTime);
        // Serial.print("c");
        sendUno.print("c");
        delay(shiftTime);
      }
      else {
        delay(shiftTime);
        // Serial.print("x");
        sendUno.print("x");
        delay(shiftTime);
      }
    }
  }
}

bool checkAlignmentRL() {

  readSensorValues();
  if (FL < realignCap && BL < realignCap) {
    if ((abs(FL - BL) > realignDistance)) { //(abs(FL - BL) < minimumDifference)
      return true;
    }
  }
  if (BR < realignCap && FR < realignCap) {
    if ((abs(FR - BR) > realignDistance)) { //(abs(FR - BR) < minimumDifference) &&
      return true;
    }
  }
  return false;

}

bool checkAlignmentFB() {

  readSensorValues();
  if (RB < realignCap && LB < realignCap) {
    if ((abs(RB - LB) > realignDistance)) { //(abs(RB - LB) < minimumDifference) &&
      return true;
    }
  }
  if (RF < realignCap && LF < realignCap) {
    if ((abs(RF - LF) > realignDistance)) { //(abs(RF - LF) < minimumDifference) &&
      return true;
    }
  }
  return false;
}

//--------------------------------------------------------------
// READ SENSOR VALUE FUNCTIONS
//--------------------------------------------------------------

void readSensorValues() {
  //      durationRF = sonar1.ping_median(3);
  //      durationLF = sonar2.ping_median(3);
  //      durationRB = sonar3.ping_median(3);
  //      durationLB = sonar4.ping_median(3);
  //      durationFR = sonar5.ping_median(3);
  //      durationBR = sonar6.ping_median(3);
  //      durationFL = sonar7.ping_median(3);
  //      durationBL = sonar8.ping_median(3);
  //
  //      FR = sonar5.convert_cm(durationFR);
  //      BR = sonar6.convert_cm(durationBR);
  //      BL = sonar8.convert_cm(durationBL);
  //      FL = sonar7.convert_cm(durationFL);
  //      RF = sonar1.convert_cm(durationRF);
  //      LF = sonar2.convert_cm(durationLF);
  //      RB = sonar3.convert_cm(durationRB);
  //      LB = sonar4.convert_cm(durationLB);
  //
  FR = sonar5.ping_cm();
  BR = sonar6.ping_cm();
  BL = sonar8.ping_cm();
  FL = sonar7.ping_cm();
  RF = sonar1.ping_cm();
  LF = sonar2.ping_cm();
  RB = sonar3.ping_cm();
  LB = sonar4.ping_cm();

  if (FR == 0) FR = oldFR;
  if (FL == 0) FL = oldFL;
  if (BR == 0) BR = oldBR;
  if (FL == 0) FL = oldFL;
  if (RF == 0) RF = oldRF;
  if (LF == 0) LF = oldLF;
  if (RB == 0) RB = oldRB;
  if (LB == 0) LB = oldLB;

  oldRF = RF;
  oldLF = LF;
  oldFR = FR;
  oldBR = BR;
  oldFL = FL;
  oldBL = BL;
  oldRB = RB;
  oldLB = LB;

  /*  DEBUGGING CODE*/
  //        Serial3.print("RF = ");
  //        Serial3.print(RF);
  //        Serial3.print(" --- ");
  //        Serial3.print("LF = ");
  //        Serial3.print(LF);
  //        Serial3.print("  ");
  //
  //        Serial3.print("FR = ");
  //        Serial3.print(FR);
  //        Serial3.print(" --- ");
  //        Serial3.print("BR = ");
  //        Serial3.print(BR);
  //        Serial3.print("  ");
  //
  //        Serial3.print("FL = ");
  //        Serial3.print(FL);
  //        Serial3.print(" --- ");
  //        Serial3.print("BL = ");
  //        Serial3.print(BL);
  //        Serial3.print("  ");
  //
  //        Serial3.print("RB = ");
  //        Serial3.print(RB);
  //        Serial3.print(" --- ");
  //        Serial3.print("LB = ");
  //        Serial3.print(LB);
  //        Serial3.println("");



  Serial.print("RF = ");
  Serial.print(RF);
  Serial.print(" --- ");
  Serial.print("LF = ");
  Serial.print(LF);
  Serial.print("  ");

  Serial.print("FR = ");
  Serial.print(FR);
  Serial.print(" --- ");
  Serial.print("BR = ");
  Serial.print(BR);
  Serial.print("  ");

  Serial.print("FL = ");
  Serial.print(FL);
  Serial.print(" --- ");
  Serial.print("BL = ");
  Serial.print(BL);
  Serial.print("  ");

  Serial.print("RB = ");
  Serial.print(RB);
  Serial.print(" --- ");
  Serial.print("LB = ");
  Serial.print(LB);
  Serial.println("");

}

void frontSensors() {

  durationRFb = sonar9.ping_median(3);
  durationLFb = sonar10.ping_median(3);

  RFb = sonar9.convert_cm(durationRFb);
  LFb = sonar10.convert_cm(durationLFb);

  //
  //  RFb = sonar9.ping_cm();
  //  LFb = sonar10.ping_cm();

  if (RFb == 0) RFb = oldRFb;
  if (LFb == 0) LFb = oldLFb;

  oldRFb = RFb;
  oldLFb = LFb;

  Serial.print("RFb = ");
  Serial.print(RFb);
  Serial.print(" --- ");
  Serial.print("LFb = ");
  Serial.print(LFb);
  Serial.println("");
}



void sendSidesOpen(int heading) {

  rightFree = 0;
  leftFree = 0;
  frontFree = 0;
  backFree = 0;

  //0 is throw out, 1 is free side, 2 is closed side

  if ((RF > sendSideOpen) && (LF > sendSideOpen)) frontFree = 1;
  if ((RB > sendSideOpen) && (LB > sendSideOpen)) backFree = 1;
  if ((FR > sendSideOpen) && (BR > sendSideOpen)) rightFree = 1;
  if ((FL > sendSideOpen) && (BL > sendSideOpen)) leftFree = 1;

  if ((RF < sendSideClosed) && (LF < sendSideClosed)) frontFree = 2;
  if ((RB < sendSideClosed) && (LB < sendSideClosed)) backFree = 2;
  if ((FR < sendSideClosed) && (BR < sendSideClosed)) rightFree = 2;
  if ((FL < sendSideClosed) && (BL < sendSideClosed)) leftFree = 2;

  Serial3.println(FL);
  Serial3.println(BL);

  if (frontFree == 0) return;
  if (backFree == 0) return;
  if (rightFree == 0) return;
  if (leftFree == 0) return;

  sidesOpen = 0;

  if (frontFree == 2) sidesOpen = sidesOpen + 8;
  if (rightFree == 2) sidesOpen = sidesOpen + 4;
  if (backFree == 2) sidesOpen = sidesOpen + 2;
  if (leftFree == 2) sidesOpen = sidesOpen + 1;

  if (sidesOpen != oldSidesOpen) {
    if (pickedUpBlock == false) {
      arduinoLocalization();
    }
    else if (pickedUpBlock == true) {
      dropoffLocalization();
    }
  }

  oldSidesOpen = sidesOpen;
}




//--------------------------------------------------------------
// FIND BLOCK FUNCTIONS
//--------------------------------------------------------------

void findBlock() {

  Serial3.println("STARTING THE FINDING PROCESS");
  Serial.println("STARTING THE FINDING PROCESS");

  readSensorValues();
  moveClosestWall();

  frontSensors();
  readSensorValues();
  blockFound = false;

  strafeAndScan();

  while (!blockFound) {
    Serial3.println("Next time through");
    Serial.println("Next time through");
    strafeAndScan();
  }
  delay(2000);

  //PUT DOWN GRIPPER HALFWAY AND OPEN IT
  //
  putDownGripper();

  getCloseToBlock();

  Serial3.println("ITS RIGHT IN THE MIDDLE");
  Serial.println("ITS RIGHT IN THE MIDDLE");

  pickUpBlock();
}


void getCloseToBlock() {

  Serial3.println("GET CLOSE TO BLOCK");

  readSensorValues();
  desiredRF = RF - (distanceFromBlock - 5.5);
  desiredLF = LF - (distanceFromBlock - 5.5);

  if (blockSide) {
    for (i = 0; i < 6; i++) {
      sendUno.print("e");
      delay(300);
    }
  }

  else {
    for (i = 0; i < 6; i++) {
      sendUno.print("q");
      delay(300);
    }
  }

  while (RF > desiredRF && LF > desiredLF) {
    sendUno.println("f");
    delay(100);
    readSensorValues();
  }
}

void strafeAndScan() {

  blockFound = false;
  readSensorValues();
  frontSensors();

  holdRFb = RFb - tolerance;
  holdLFb = LFb - tolerance;
  holdRF = RF;

  if (!closestWall) {
    Serial3.println("MOVE RIGHT");

    while ((RFb > holdRFb) && (LFb > holdLFb) && (FR > 4.5) && (BR > 4.5) && (abs(holdRF - RF) < 50)) {
      Serial3.print(RFb);
      Serial3.print(",");
      Serial3.println(LFb);
      sendUno.print("e");
      delay(100);
      readSensorValues();
      frontSensors();
    }

    if (abs(holdRF - RF) > 50) {
      sendUno.println("q");
      delay(300);
      sendUno.println("q");
      delay(300);
      sendUno.println("q");
      delay(300);
    }

    closestWall = true;
  }
  else { //MOVE TO THE LEFT
    Serial3.println("MOVE LEFT");

    while ((RFb > holdRFb) && (LFb > holdLFb) && (FL > 4.5) && (BL > 4.5)) {
      Serial3.print(RFb);
      Serial3.print(",");
      Serial3.println(LFb);
      sendUno.print("q");
      delay(100);
      readSensorValues();
      frontSensors();
    }
    closestWall = false;
  }

  Serial3.println("BROKE OUT");
  Serial3.print(LF);
  Serial3.print("  -  ");
  Serial3.println(LF);
  Serial.println("BROKE OUT");
  Serial.print(LF);
  Serial.print("  -  ");
  Serial.println(LF);

  readSensorValues();

  //FIGURE OUT WHICH SIDE THE BLOCK IS ON AND MEASURE DISTANCE, LIGHT UP APPROPRIATE SIDE LED
  if (RFb < holdRFb && RFb < 24.5) {
    blockSide = true;
    blockFound = true;
    distanceFromBlock = RFb;
    digitalWrite(LEDpinRight, HIGH);
    Serial3.print("BLOCK ON RIGHT - AT DISTANCE: ");
    Serial3.println(distanceFromBlock);
    delay(500);
  }
  else if ((LFb < holdLFb) && (LFb < 24.5)) {
    if (LF > 16) {
      blockSide = false;
      blockFound = true;
      distanceFromBlock = LFb;
      digitalWrite(LEDpinLeft, HIGH);
      Serial3.print("BLOCK ON LEFT - AT DISTANCE: ");
      Serial3.println(distanceFromBlock);
      Serial3.print("FRONT LEFT - AT DISTANCE: ");
      Serial3.println(LF);
      delay(500);
    }
    else {
      sendUno.println("e");
      delay(200);
      sendUno.println("e");
      delay(200);
      sendUno.println("e");
      delay(200);
      sendUno.print("w");
      delay(150);
      sendUno.print("z");
    }
  }
  else {
    reactAlignment();
    sendUno.print("w");
    delay(150);
    sendUno.print("z");
  }

}


void moveClosestWall() { //Returns true if the right side is closer, return false if the left side is closer

  Serial3.println("DOING MOVE CLOSEST WALL");
  Serial.println("DOING MOVE CLOSEST WALL");

  readSensorValues();
  //True is closest to right, false is closest to left

  holdRF = RF;

  if ((FR + BR) > (FL + BL)) { //L IS CLOSER
    while (FL > huggingWallDistance && BL > huggingWallDistance) {
      sendUno.print("q");
      delay(100);
      readSensorValues();
    }
    closestWall = false;
  }

  else  {

    while (FR > huggingWallDistance && BR > huggingWallDistance && (abs(holdRF - RF) < 50)) {
      sendUno.print("e");
      delay(100);
      readSensorValues();
    }

    if (abs(holdRF - RF) > 50) {
      Serial.println("Adjusting Left");
      sendUno.println("q");
      delay(300);
      sendUno.println("q");
      delay(300);
      sendUno.println("q");
      delay(300);
    }

    closestWall = true;
    //Serial.println("HERE");
  }

  //delay(2000);
}



//--------------------------------------------------------------
// GRIPPER FUNCTIONS
//--------------------------------------------------------------

void putDownGripper() {

  //SERVO DOWN AND OPEN
  upDown.write(40);
  delay(2000);
  openClose.write(0);
  delay(2000);
  upDown.write(70);
  delay(2000);

}

void pickUpBlock() {

  //SERVO CLOSE AND UP
  openClose.write(45);
  delay(2000);
  upDown.write(0);
  delay(2000);

}

void putDownBlock() {

  //SERVO DOWN AND OPEN
  upDown.write(70);
  delay(2000);
  openClose.write(0);
  delay(2000);

  //SERVO CLOSE AND UP
  upDown.write(0);
  delay(2000);

}


//--------------------------------------------------------------
// FOLLOW PATH ACTIONS
//--------------------------------------------------------------

void doCommand(char movementCommand) {

  //MOVE UNTIL YOU RUN INTO A WALL COMMANDS
  if (movementCommand == 'W') {
    readSensorValues();
    drivingDirection = true;
    while (RF > stoppingDistance && LF > stoppingDistance) {
      driveForward();
      delay(100);
      reactSides();
      reactAlignment();
    }
  }
  else if (movementCommand == 'A') {
    readSensorValues();
    drivingDirection = false;
    while (FL > stoppingDistance && BL > stoppingDistance) {
      driveLeft();
      delay(100);
      reactSides();
      reactAlignment();
    }
  }
  else if (movementCommand == 'S') {
    readSensorValues();
    drivingDirection = true;
    while (RB > stoppingDistance && LB > stoppingDistance) {
      driveBackward();
      delay(100);
      reactSides();
      reactAlignment();
    }
  }
  else if (movementCommand == 'D') {
    drivingDirection = false;
    readSensorValues();
    while (FR > stoppingDistance && BR > stoppingDistance) {
      driveRight();
      delay(100);
      reactSides();
      reactAlignment();
    }
  }

  //MOVEMENT COMMANDS FOR MOVE UNTIL SIDE IS NOW FREE
  //MOVE FORWARD UNTIL SIDES ARE FREE
  if (movementCommand == 'w') {
    readSensorValues();
    drivingDirection = true;

    while ((BR > littleDriveLimit) || (BL > littleDriveLimit) && RF > minWallDistanceB && LF > minWallDistanceB) {
      sendUno.print("f");
      delay(100);
      reactSides();
      reactAlignment();
    }

    //  Serial3.println("Between loops");

    while ( BL < littleDriveLimit && BR < littleDriveLimit && RF > minWallDistanceB && LF > minWallDistanceB) {
      sendUno.print("f");
      //      Serial3.print(BR);
      //      Serial3.print("   ");
      //      Serial3.println(BL);
      delay(100);
      reactSides();
      reactAlignment();
    }

    sendUno.print("w");
    delay(littleDriveShift);
    sendUno.print("z");
  }

  //MOVE LEFT UNTIL SIDES ARE FREE
  else if (movementCommand == 'a') {
    readSensorValues();
    drivingDirection = false;

    while ((RB > littleDriveLimit) || (RF > littleDriveLimit) && BL > minWallDistanceLR && FL > minWallDistanceLR) {
      //driveLeft();
      sendUno.print("q");

      delay(100);
      reactSides();
      reactAlignment();
    }

    //      Serial3.println("Between loops");

    while (RF < littleDriveLimit && RB < littleDriveLimit && BL > minWallDistanceLR && FL > minWallDistanceLR) {
      //driveLeft();
      sendUno.print("q");
      delay(100);
      reactSides();
      reactAlignment();
    }
    sendUno.print("a");
    delay(littleDriveShift);
    sendUno.print("z");
  }

  //MOVE BACKWARD UNTIL SIDES ARE FREE
  else if (movementCommand == 's') {
    readSensorValues();
    drivingDirection = true;

    while ((FR > littleDriveLimit) || (FL > littleDriveLimit) && RB > minWallDistanceB && LB > minWallDistanceB) {
      //driveBackward();
      sendUno.print("r");
      delay(100);
      reactSides();
      reactAlignment();
    }
    while ((FL < littleDriveLimit) && (FR < littleDriveLimit) && RB > minWallDistanceF && LB > minWallDistanceF) {
      //driveBackward();
      sendUno.print("r");
      delay(100);
      reactSides();
      reactAlignment();
    }
    sendUno.print("s");
    delay(littleDriveShift);
    sendUno.print("z");
  }

  else if (movementCommand == 'd') {
    drivingDirection = false;
    readSensorValues();

    while ((LB > littleDriveLimit) || (LF > littleDriveLimit) && FR > minWallDistanceLR && BR > minWallDistanceLR) {
      //driveRight();
      sendUno.print("e");
      delay(100);
      reactSides();
      reactAlignment();
    }
    while ((LB < littleDriveLimit) && (LF < littleDriveLimit) && FR > minWallDistanceLR && BR > minWallDistanceLR) {
      //driveRight();
      sendUno.print("e");
      delay(100);
      reactSides();
      reactAlignment();
    }
    sendUno.print("d");
    delay(littleDriveShift);
    sendUno.print("z");
  }
}


void arduinoLocalization() {

  for (i = 0; i < 13; i++) {
    digitalWrite(LEDpinRight, LOW);
    digitalWrite(LEDpinLeft, HIGH);
    delay(120);
    digitalWrite(LEDpinRight, HIGH);
    digitalWrite(LEDpinLeft, LOW);
    delay(120);
  }

  Serial3.print("Sides Open = ");
  Serial3.println(sidesOpen);

  if (facing) {

    if (sidesOpen == 9) {
      path[0] = 'S';
      path[1] = 'w';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 7) {
      path[0] = 'w';
      path[1] = 'a';
      path[2] = 'S';
      path[3] = 'A';
      path[4] = 'w';
      mode = 'L';
    }
    else if (sidesOpen == 8) {
      path[0] = 'S';
      path[1] = 'A';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 2) {
      path[0] = 'A';
      path[1] = 'w';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 0) {
      path[0] = 'S';
      path[1] = 'A';
      path[2] = 'w';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 4) {
      path[0] = 'a';
      path[1] = 'S';
      path[2] = 'A';
      path[3] = 'w';
      path[4] = 'n';
      mode = 'L';
    }

    if (loadingZone == 2) {
      path[5] = 'D';
    }
    else path[5] = 'n';
  }

  else {

    if (sidesOpen == 6) {
      path[0] = 'A';
      path[1] = 'n';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 3) {
      path[0] = 'n';
      path[1] = 'n';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 0) {
      path[0] = 'D';
      path[1] = 'S';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 13) {
      path[0] = 's';
      path[1] = 'D';
      path[2] = 'S';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 1) {
      path[0] = 'D';
      path[1] = 'S';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 2) {
      path[0] = 'A';
      path[1] = 'n';
      path[2] = 'n';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }
    else if (sidesOpen == 8) {
      path[0] = 'D';
      path[1] = 'S';
      path[2] = 'A';
      path[3] = 'n';
      path[4] = 'n';
      mode = 'L';
    }

    if (loadingZone == 3) {
      path[5] = 'D';
    }
    else path[5] = 'd';

    path[6] = 'X';
    path[12] = 'P';

  }
}



void dropoffLocalization() {

  if (facing) {

    if (dropoffZone == 1) {

      if (sidesOpen == 9) {
        path[7] = 'S';
        path[8] = 'd';
        path[9] = 'W';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 7) {
        path[7] = 'w';
        path[8] = 'a';
        path[9] = 'S';
        path[10] = 'a';
        path[11] = 'W';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'A';
        path[8] = 'S';
        path[9] = 'd';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'W';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'S';
        path[8] = 'a';
        path[9] = 'W';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 4) {
        path[7] = 'a';
        path[8] = 'S';
        path[9] = 'a';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 12) {
        path[7] = 'A';
        path[8] = 'S';
        path[9] = 'd';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
    }

    if (dropoffZone == 2) {

      if (sidesOpen == 9) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'd';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 7) {
        path[7] = 'w';
        path[8] = 'a';
        path[9] = 'W';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'd';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'D';
        path[8] = 'W';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'W';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 4) {
        path[7] = 'a';
        path[8] = 'W';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 12) {
        path[7] = 'S';
        path[8] = 'd';
        path[9] = 'W';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
    }

    if (dropoffZone == 3) {

      if (sidesOpen == 9) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'D';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 7) {
        path[7] = 'W';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'D';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'D';
        path[8] = 'w';
        path[9] = 'D';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'D';
        path[8] = 'W';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 4) {
        path[7] = 'W';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 12) {
        path[7] = 'S';
        path[8] = 'D';
        path[9] = 'W';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
    }

    if (dropoffZone == 4) {

      if (sidesOpen == 9) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'D';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 7) {
        path[7] = 'n';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'D';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'D';
        path[8] = 'w';
        path[9] = 'D';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'D';
        path[8] = 'S';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 4) {
        path[7] = 'S';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 12) {
        path[7] = 'S';
        path[8] = 'D';
        path[9] = 'S';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
    }
  }


  if (!facing) {

    if (dropoffZone == 1) {

      if (sidesOpen == 6) {
        path[7] = 'W';
        path[8] = 'a';
        path[9] = 'S';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 3) {
        path[7] = 'D';
        path[8] = 'W';
        path[9] = 'a';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'W';
        path[8] = 'd';
        path[9] = 'S';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 13) {
        path[7] = 's';
        path[8] = 'd';
        path[9] = 'W';
        path[10] = 'd';
        path[11] = 'S';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'D';
        path[8] = 'W';
        path[9] = 'a';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'S';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 1) {
        path[7] = 'd';
        path[8] = 'W';
        path[9] = 'd';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
    }

    if (dropoffZone == 2) {

      if (sidesOpen == 6) {
        path[7] = 'W';
        path[8] = 'A';
        path[9] = 'S';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 3) {
        path[7] = 'W';
        path[8] = 'a';
        path[9] = 'S';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'S';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 13) {
        path[7] = 's';
        path[8] = 'd';
        path[9] = 'S';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'A';
        path[8] = 'W';
        path[9] = 'a';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'A';
        path[8] = 'S';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 1) {
        path[7] = 'd';
        path[8] = 'S';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
    }

    if (dropoffZone == 3) {

      if (sidesOpen == 6) {
        path[7] = 'A';
        path[8] = 'W';
        path[9] = 'A';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 3) {
        path[7] = 'W';
        path[8] = 'A';
        path[9] = 'S';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'A';
        path[8] = 'S';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 13) {
        path[7] = 'S';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'A';
        path[8] = 'W';
        path[9] = 'A';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'A';
        path[8] = 's';
        path[9] = 'A';
        path[10] = 'S';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 1) {
        path[7] = 'S';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
    }

    if (dropoffZone == 4) {

      if (sidesOpen == 6) {
        path[7] = 'A';
        path[8] = 'W';
        path[9] = 'A';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 3) {
        path[7] = 'W';
        path[8] = 'A';
        path[9] = 'W';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 0) {
        path[7] = 'A';
        path[8] = 'W';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 13) {
        path[7] = 'n';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 2) {
        path[7] = 'A';
        path[8] = 'W';
        path[9] = 'A';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 8) {
        path[7] = 'A';
        path[8] = 's';
        path[9] = 'A';
        path[10] = 'W';
        path[11] = 'n';
        mode = 'L';
      }
      else if (sidesOpen == 1) {
        path[7] = 'W';
        path[8] = 'n';
        path[9] = 'n';
        path[10] = 'n';
        path[11] = 'n';
        mode = 'L';
      }
    }
  }
}








void destinationCheck() {

  rightFree = 0;
  leftFree = 0;
  frontFree = 0;
  backFree = 0;

  relocalize = false;
  //0 is throw out, 1 is free side, 2 is closed side

  if ((RF > sendSideOpen) && (LF > sendSideOpen)) frontFree = 1;
  if ((RB > sendSideOpen) && (LB > sendSideOpen)) backFree = 1;
  if ((FR > sendSideOpen) && (BR > sendSideOpen)) rightFree = 1;
  if ((FL > sendSideOpen) && (BL > sendSideOpen)) leftFree = 1;

  if ((RF < sendSideClosed) && (LF < sendSideClosed)) frontFree = 2;
  if ((RB < sendSideClosed) && (LB < sendSideClosed)) backFree = 2;
  if ((FR < sendSideClosed) && (BR < sendSideClosed)) rightFree = 2;
  if ((FL < sendSideClosed) && (BL < sendSideClosed)) leftFree = 2;

  if (frontFree == 0) return;
  if (backFree == 0) return;
  if (rightFree == 0) return;
  if (leftFree == 0) return;

  sidesOpen = 0;

  if (frontFree == 2) sidesOpen = sidesOpen + 8;
  if (rightFree == 2) sidesOpen = sidesOpen + 4;
  if (backFree == 2) sidesOpen = sidesOpen + 2;
  if (leftFree == 2) sidesOpen = sidesOpen + 1;

  Serial3.print("Sides Open in destination Check: ");
  Serial3.println(sidesOpen);

  relocalize = true;

  Serial3.print("Relocalize before if statements: ");
  Serial3.println(relocalize);

  if (facing && sidesOpen == 1) { // drop
    relocalize = false;
    for (i = 0; i < 20; i++) {
      digitalWrite(LEDpinRight, LOW);
      digitalWrite(LEDpinLeft, HIGH);
      delay(150);
      digitalWrite(LEDpinRight, HIGH);
      digitalWrite(LEDpinLeft, LOW);
      delay(150);
    }

    Serial3.print("Relocalize in first if statement: ");
    Serial3.println(relocalize);

  }

  else if (!facing && sidesOpen == 3) {
    relocalize = false;
    for (i = 0; i < 20; i++) {
      digitalWrite(LEDpinRight, LOW);
      digitalWrite(LEDpinLeft, HIGH);
      delay(150);
      digitalWrite(LEDpinRight, HIGH);
      digitalWrite(LEDpinLeft, LOW);
      delay(150);
    }

    Serial3.print("Relocalize in second if statement: ");
    Serial3.println(relocalize);

  }
  //    if (sidesOpen == 1){
  //      for (i = 0; i < 20; i++) {
  //        digitalWrite(LEDpinRight, LOW);
  //        digitalWrite(LEDpinLeft, HIGH);
  //        delay(150);
  //        digitalWrite(LEDpinRight, HIGH);
  //        digitalWrite(LEDpinLeft, LOW);
  //        delay(150);
  //      }
  //      relocalize = false;
  //    }
  //     else {
  //      Serial3.println("DID SET IT TRUE in FACING");
  //      relocalize = true;
  //    }
  //}
  //
  //else if (!facing) {
  //
  //
  //}
  ////    if (sidesOpen == 3){
  //      for (i = 0; i < 20; i++) {
  //        digitalWrite(LEDpinRight, LOW);
  //        digitalWrite(LEDpinLeft, HIGH);
  //        delay(150);
  //        digitalWrite(LEDpinRight, HIGH);
  //        digitalWrite(LEDpinLeft, LOW);
  //        delay(150);
  //      }
  //      relocalize = false;
  //    }
  //    else {
  //      relocalize = true;
  //      Serial3.println("SET IT TO TRUE IN !FACING ONE");
  //    }
  //  }

  Serial3.print("Relocalize at end: ");
  Serial3.println(relocalize);

}














void dropoffCheck() {

  rightFree = 0;
  leftFree = 0;
  frontFree = 0;
  backFree = 0;

  relocalize = false;
  //0 is throw out, 1 is free side, 2 is closed side

  if ((RF > sendSideOpen) && (LF > sendSideOpen)) frontFree = 1;
  if ((RB > sendSideOpen) && (LB > sendSideOpen)) backFree = 1;
  if ((FR > sendSideOpen) && (BR > sendSideOpen)) rightFree = 1;
  if ((FL > sendSideOpen) && (BL > sendSideOpen)) leftFree = 1;

  if ((RF < sendSideClosed) && (LF < sendSideClosed)) frontFree = 2;
  if ((RB < sendSideClosed) && (LB < sendSideClosed)) backFree = 2;
  if ((FR < sendSideClosed) && (BR < sendSideClosed)) rightFree = 2;
  if ((FL < sendSideClosed) && (BL < sendSideClosed)) leftFree = 2;

  if (frontFree == 0) return;
  if (backFree == 0) return;
  if (rightFree == 0) return;
  if (leftFree == 0) return;

  sidesOpen = 0;

  if (frontFree == 2) sidesOpen = sidesOpen + 8;
  if (rightFree == 2) sidesOpen = sidesOpen + 4;
  if (backFree == 2) sidesOpen = sidesOpen + 2;
  if (leftFree == 2) sidesOpen = sidesOpen + 1;

  Serial3.print("Sides Open in dropoff Check: ");
  Serial3.println(sidesOpen);

  relocalize = true;

  Serial3.print("Relocalize before if statements: ");
  Serial3.println(relocalize);

  if (facing && sidesOpen == sidesOpenFacing) { // drop
    relocalize = false;
    for (i = 0; i < 20; i++) {
      digitalWrite(LEDpinRight, LOW);
      digitalWrite(LEDpinLeft, HIGH);
      delay(150);
      digitalWrite(LEDpinRight, HIGH);
      digitalWrite(LEDpinLeft, LOW);
      delay(150);
    }

    Serial3.print("Relocalize in first if statement: ");
    Serial3.println(relocalize);

  }

  else if (!facing && sidesOpen == sidesOpenNotFacing) {
    relocalize = false;
    for (i = 0; i < 20; i++) {
      digitalWrite(LEDpinRight, LOW);
      digitalWrite(LEDpinLeft, HIGH);
      delay(150);
      digitalWrite(LEDpinRight, HIGH);
      digitalWrite(LEDpinLeft, LOW);
      delay(150);
    }

    Serial3.print("Relocalize in second if statement: ");
    Serial3.println(relocalize);

  }
  //    if (sidesOpen == 1){
  //      for (i = 0; i < 20; i++) {
  //        digitalWrite(LEDpinRight, LOW);
  //        digitalWrite(LEDpinLeft, HIGH);
  //        delay(150);
  //        digitalWrite(LEDpinRight, HIGH);
  //        digitalWrite(LEDpinLeft, LOW);
  //        delay(150);
  //      }
  //      relocalize = false;
  //    }
  //     else {
  //      Serial3.println("DID SET IT TRUE in FACING");
  //      relocalize = true;
  //    }
  //}
  //
  //else if (!facing) {
  //
  //
  //}
  ////    if (sidesOpen == 3){
  //      for (i = 0; i < 20; i++) {
  //        digitalWrite(LEDpinRight, LOW);
  //        digitalWrite(LEDpinLeft, HIGH);
  //        delay(150);
  //        digitalWrite(LEDpinRight, HIGH);
  //        digitalWrite(LEDpinLeft, LOW);
  //        delay(150);
  //      }
  //      relocalize = false;
  //    }
  //    else {
  //      relocalize = true;
  //      Serial3.println("SET IT TO TRUE IN !FACING ONE");
  //    }
  //  }

  Serial3.print("Relocalize at end: ");
  Serial3.println(relocalize);

}


