//pinout
int IRF = A0; //forward IR (analog)
int IRR = A1; //right IR (analog)
int IRL = A2; //left IR (analog)
int ACW = 13; //Motor A pin 1
int ACCW = 12; //Motor A pin 2
int BCW = 11; //Motor B pin 1
int BCCW = 10; //Motor B pin 2
int CCW = 9; //Motor C pin 1
int CCCW = 8; //Motor C pin 2
int yLED = 3; //yellow LED for color reflectance
int photoR = A3; //photoresistor for color reflectance
int trig = 7; //shared trigger pin for ultrasonic distance sensor
int echoR = 6; //echo pin for Right Ultrasonic
int echoL = 4; //echo pin for left ultrasonic
int outTurr = 2; //digital output pin to turn on turret
int motPWM = 5; //pwm pin to set motor speed
int dist; //store current distance value
int cRead; //store current color
char angOff; //store current angle offset from horizontal
boolean wall = false;
boolean remCon = true;
boolean cont = true;
boolean turn = true;
boolean inter = false; //in intersection
boolean found = false;

int com = 1; //1=forw, 2=back, 3=stop, 4=CW, 5=CCW
int mode = 2; //1=Remote control, 2=Computer Control, 3=wall hugging mode, 4=free range mode
char mState; //mode state for computer control

unsigned long sTime;
unsigned long fTime;
long delT;

int motSet = 85; //set motor speed
char bState; //robot State for balloon detection

//////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  pinMode(IRF, INPUT); //IR analog
  pinMode(IRR, INPUT); //IR analog right side
  pinMode(IRL, INPUT); //IR analog left side
  pinMode(ACW, OUTPUT); //Motor A clockwise
  pinMode(ACCW, OUTPUT); //Motor A counterclockwise
  pinMode(BCW, OUTPUT); //Motor B clockwise
  pinMode(BCCW, OUTPUT); //Motor B counterclockwise
  pinMode(CCW, OUTPUT); //Motor C clockwise
  pinMode(CCCW, OUTPUT); //Motor C counterclockwise
  pinMode(trig, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoL, INPUT); // Sets the echoPin as an Input
  pinMode(echoR, INPUT); // Sets the echoPin as an Input
  pinMode(motPWM, OUTPUT); //Sets speed of motors
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  pinMode(outTurr, OUTPUT); //set turrent pin as output (send to other Arduino)
  digitalWrite(outTurr, LOW);
  pinMode(yLED, OUTPUT);
  digitalWrite(yLED, HIGH); //turn on yellow color sensor
  pinMode(photoR, INPUT);
}
//////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (Serial.available() > 0) { //if there is serial data
    mState = Serial.read(); //input from serial as a string
    Serial.print("Received: ");
    Serial.println(mState);
    Serial.println();
    translateCC(mState); //set command for input
    delay(50);
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void setRobot (int command) {      //function for setting Robot action

  switch (command) {
    case 1: //forward
      digitalWrite(ACW, HIGH);
      digitalWrite(ACCW, LOW);
      digitalWrite(BCW, LOW);
      digitalWrite(BCCW, HIGH);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, LOW);
      break;
    case 2: //backwards
      digitalWrite(ACW, LOW);
      digitalWrite(ACCW, HIGH);
      digitalWrite(BCW, HIGH);
      digitalWrite(BCCW, LOW);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, LOW);
      break;
    case 3: //stop
      digitalWrite(ACW, LOW);
      digitalWrite(ACCW, LOW);
      digitalWrite(BCW, LOW);
      digitalWrite(BCCW, LOW);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, LOW);
      break;
    case 4: //CW rotation
      digitalWrite(ACW, HIGH);
      digitalWrite(ACCW, LOW);
      digitalWrite(BCW, HIGH);
      digitalWrite(BCCW, LOW);
      digitalWrite(CCW, HIGH);
      digitalWrite(CCCW, LOW);
      break;
    case 5: //CCW rotation
      digitalWrite(ACW, LOW);
      digitalWrite(ACCW, HIGH);
      digitalWrite(BCW, LOW);
      digitalWrite(BCCW, HIGH);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, HIGH);
      break;
    case 6: //CW rotation LR
      digitalWrite(ACW, HIGH);
      digitalWrite(ACCW, LOW);
      digitalWrite(BCW, HIGH);
      digitalWrite(BCCW, LOW);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, LOW);
      break;
    case 7: //CCW rotation LR
      digitalWrite(ACW, LOW);
      digitalWrite(ACCW, HIGH);
      digitalWrite(BCW, LOW);
      digitalWrite(BCCW, HIGH);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, LOW);
      break;
    default: //stop
      digitalWrite(ACW, LOW);
      digitalWrite(ACCW, LOW);
      digitalWrite(BCW, LOW);
      digitalWrite(BCCW, LOW);
      digitalWrite(CCW, LOW);
      digitalWrite(CCCW, LOW);
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////
long getDist(int echo) {    // get distance from ultrasonic distance sensor
  long distance;
  // Clears the trigPin
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echo, HIGH, 5000);
  // Calculating the distance
  if (duration == 0) {
    distance = 255;
  } else {
    distance = duration * 0.034 / 2;
  }
  // Prints the distance on the Serial Monitor
  return distance;
}

//////////////////////////////////////////////////////////////////////////////////////
void FWD() { //drive forward
  analogWrite(motPWM, motSet); //Set motor speed
  setRobot(1);
}

//////////////////////////////////////////////////////////////////////////////////////
long FWDLine() { //drive forward until you hit a line
  Serial.println("Moving forward");
  sTime = millis();
  int count = 0;
  FWD();
  delay(300);
  while (wall == false) {
    FWD();
    if (analogRead(IRF) < 300) {
      wall = true;
      STP();
      fTime = (long)(millis() - sTime);
      //Serial.print("Stopping: ");
      //Serial.print(fTime);
      break;
    } else if (analogRead(IRL) < 650 && analogRead (IRR) > 650) {
      if (count < 6) {
        CWMicro();
        count++;
      }
      count++;
    } else if (analogRead(IRR) < 650 && analogRead (IRL) > 650) {
      if (count < 6) {
        CCWMicro();
        count++;
      }
    } else {
      count = 0;
    }
    if (count >= 6) {
      count = 0;
      FWD();
      delay(250);
      STP();
    }
    if (Serial.available() > 0) { //if there is serial data
      angOff = Serial.read(); //input from serial as an integer
      switch (angOff) {
        case '1': //no issue
          break;
        case '2': //clockwise
          CWMicro();
          //Serial.println("Adjust right from pi");
          break;
        case '3': //counterclockwise
          CCWMicro();
          //Serial.println("Adjust left from pi");
          break;
        default:
          break;
      }
    }
  }
  delay(250);
  FWD();
  delay(70);
  STP();
  wall = false;
  return fTime;
}

//////////////////////////////////////////////////////////////////////////////////////
void BWD() { //drive backwards
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  setRobot(2);
  //delay(1000);
  //setRobot(3);
}
//////////////////////////////////////////////////////////////////////////////////////
void STP() { //stop robot
  setRobot(3);
}

//////////////////////////////////////////////////////////////////////////////////////
void CW90() { //CW 90 deg - NOT NEEDED WITH 3 IRs
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  setRobot(4);
  delay(1400);
  setRobot(3);
}

//////////////////////////////////////////////////////////////////////////////////////
void CW90IR() { //CW 90 deg
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  turn = true;
  while (turn) {
    setRobot(4);
    if (analogRead(IRL) < 650) {
      delay(200);
      setRobot(3);
      turn = false;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void CCW90() { //CCW90 deg - NOT NEEDED WITH 3 IRs
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  setRobot(5);
  delay(1400);
  setRobot(3);
}

//////////////////////////////////////////////////////////////////////////////////////
void CCW90IR() { //CCW 90 deg
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  turn = true;
  while (turn) {
    setRobot(5);
    if (analogRead(IRR) < 650) {
      delay(200);
      setRobot(3);
      turn = false;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void CW180() { //CW180 deg
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  //Serial.println("Turn 180");
  setRobot(4);
  delay(2800);
  setRobot(3);
}

//////////////////////////////////////////////////////////////////////////////////////
void CWMicro() { //small adjustment in CW direction
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  delay(2);
  setRobot(6);
  delay(100);
  setRobot(3);
}

//////////////////////////////////////////////////////////////////////////////////////
void CCWMicro() { //small adjustment in CCW direction
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  delay(2);
  setRobot(7);
  delay(100);
  setRobot(3);
}

//////////////////////////////////////////////////////////////////////////////////////
void CWLR() { //turn CW larger radius
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  setRobot(6);
}

//////////////////////////////////////////////////////////////////////////////////////
void CCWLR() { //turn CCW larger radius
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  setRobot(7);
}

//////////////////////////////////////////////////////////////////////////////////////
void chooseTurn() { //depending on ultrasonic outputs, determine which way to turn
  int wallR = getDist(echoR);
  delay(250);
  int wallL = getDist(echoL);
  //Serial.println(wallR);
  //Serial.println(wallL);
  BWD();
  delay(300);
  STP();
  if (wallR > wallL) {
    //Serial.print("Turning clockwise");
    CW90();
    delay(250);
  }
  if (wallR < wallL) {
    //Serial.println("Turning counterclockwise");
    CCW90();
    delay(250);
  }
  if (wallR == wallL) {
    //Serial.println("Turning counterclockwise");
    CW90();
    delay(250);
  }
  delay(200);
}

//////////////////////////////////////////////////////////////////////////////////////
void ballF() { //follow the balloon
  analogWrite(motPWM, motSet); //Set motor speed to slower.
  remCon = true;
  while (remCon == true) {
    if (Serial.available() > 0) {
      bState = Serial.read() - '0';
      //Serial.print("You sent me: ");
      //Serial.println(bState);
      switch (bState) {
        case 0:
          remCon = false;
          STP();
          break;
        case 1:
          analogWrite(motPWM, motSet); //Set motor speed to slower.
          FWD();
          break;
        case 2:
          analogWrite(motPWM, motSet); //Set motor speed to slower.
          CCWLR();
          break;
        case 3:
          analogWrite(motPWM, motSet); //Set motor speed to slower.
          CWLR();
          break;
        default:
          STP();
          break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void charge() { //charge balloon
  //Serial.println("Charging");
  digitalWrite(outTurr, HIGH); //turn on stepper motor from other Arduino
  analogWrite(motPWM, motSet + 35); //Set motor speed to slower.
  delay(800); //wait for lance to activate
  FWD(); //go foward
  digitalWrite(outTurr, LOW); //ensure stepper does only one rev
  delay(100);
  STP(); //stop robot
  analogWrite(motPWM, motSet + 35); //Set motor speed to slower.
}

//////////////////////////////////////////////////////////////////////////////////////
int ballCheck() { //checks state of balloon
  Serial.println('o');
  if (Serial.available() > 0) {
    bState = Serial.read();
  } else {
    bState = '0';
  }
  return bState;
}

//////////////////////////////////////////////////////////////////////////////////////
int colCheck() { //checks color state in front of robot
  //digitalWrite(yLED, HIGH);
  delay(100);
  cRead = analogRead(photoR);
  delay(100);
  //digitalWrite(yLED, LOW);
  return cRead;
}

//////////////////////////////////////////////////////////////////////////////////////
void translateCC(char codeCC) { // takes action based on serial code received
  switch (codeCC)
  {
    case 'F': // forward
      analogWrite(motPWM, motSet); //Set motor speed to slower.
      Serial.println("Forward"); //full forward
      FWD();
      break;
    case 'B': // backward
      analogWrite(motPWM, motSet); //Set motor speed to slower.
      Serial.println("Backward"); //full reverse
      BWD();
      break;
    case 'S': // stop
      Serial.println("Stop"); //full reverse
      STP();
      break;
    case 'R': // clockwise 90deg
      analogWrite(motPWM, motSet); //Set motor speed to slower.
      Serial.println("90 deg CW"); //clockwise 90deg
      CW90IR();
      break;
    case 'r': // counterclockwise 90deg
      analogWrite(motPWM, motSet); //Set motor speed to slower.
      Serial.println("90 deg CCW"); //clockwise 180 deg
      CCW90IR();
    case 'v'://test clockwise rotation
      CW90();
      break;
    case '>': // clockwise small adjustment
      Serial.println("CW Adjust"); //clockwise 180 deg
      CWMicro();
      break;
    case '<': // clockwise small adjustment
      Serial.println("CCW Adjust"); //clockwise 180 deg
      CCWMicro();
      break;
    case 'P': // FWD until line
      analogWrite(motPWM, motSet); //Set motor speed to slower.
      Serial.println("FWD line"); //full forward until line
      delT = FWDLine();
      break;
    case '*': // charge
      Serial.println("Charge"); //full forward until line
      charge();
      break;
    case 'M': // run maze solving program
      delay(1000);
      found = false;
      Serial.println("Maze Solver"); //run centering program
      digitalWrite(yLED, HIGH); //turn on yellow color sensor
      analogWrite(motPWM, motSet); //Set motor speed to slower.
      while (found == false) {
        bState = ballCheck;
        Serial.println(bState);
        if (bState == '1') {
          Serial.println("YES");
        } else {
          Serial.println("NO");
        }
        delT = FWDLine();
        Serial.print("DelT: ");
        Serial.println(delT);
        Serial.println();
        delay(250);
        cRead = colCheck();
        Serial.print("Color: ");
        Serial.println(cRead);
        if (cRead < 200) { //purple line
          Serial.println("P: Begin Inter");
          inter = true; //enter intersection
          FWD();
          delay(1000);
          delT = FWDLine(); //continue to end of intersection
          cRead = colCheck(); //check color again
          if (cRead < 200) { //purple line again - IN CENTRAL INTERSECTION
            Serial.print("DelT: ");
            Serial.println(delT);
            Serial.println("P: End Central Inter");
            BWD();
            delay(300);
            STP();
            CW90(); //turn
            delT = FWDLine(); //continue to corner of intersection
            cRead = colCheck(); //check color again
            if (cRead < 200) { //purple line again
              Serial.println("P: Side Central Inter");
              FWD();
              delay(150);
              delT = FWDLine(); //continue to yellow
              Serial.println("Y: At 1st");
              charge();
              BWD();
              delay(350);
              STP();
              CW180();
              delT = FWDLine();
              cRead = colCheck(); //check color again
              if (cRead > 200) { //purple line again
                Serial.println("Hit wall");
                STP();
              } else {
                Serial.println("Exit Central Inter");
                inter = false;
              }
            }
          } else {
            Serial.println("Y: Alley Inter");
            BWD();
            delay(300);
            STP();
            CCW90();
            STP();
            delT = FWDLine();
            cRead = colCheck();
            if (cRead > 200) { //purple line
              Serial.println("P: End Alley Inter");
            }
          }
        } else {
          Serial.println("Y");
          bState = ballCheck();
          chooseTurn();
        }
      }
      digitalWrite(yLED, LOW);
      break;
    case 'I': // check IR sensor values
      Serial.println("IR CHECK"); //IR sensor check
      Serial.print("Forward: ");
      Serial.println(analogRead(IRF));
      Serial.print("Right: ");
      Serial.println(analogRead(IRR));
      Serial.print("Left: ");
      Serial.println(analogRead(IRL));
      Serial.println();
      break;
    case 'D': // check distance values
      Serial.print("DIST CHECK"); //IR sensor check
      Serial.println();
      Serial.print("Right: ");
      Serial.print(getDist(echoR));
      delayMicroseconds(10);
      Serial.print(", Left: ");
      Serial.println(getDist(echoL));
      //Serial.print("Side: ");
      //Serial.println(analogRead(IR2));
      break;
    case '-': // check color values
      Serial.print("COLOR CHECK"); //IR sensor check
      Serial.println();
      cRead = colCheck();
      Serial.println(cRead);
      Serial.println();
      break;
    case '+': // check balloon values
      Serial.print("BALLOON CHECK"); //IR sensor check
      Serial.println();
      bState = ballCheck();
      Serial.println(bState);
      Serial.println();
      break;
    case 'c': //7 - clockwise large radius rotation
      Serial.println("Clockwise large radius");
      analogWrite(motPWM, motSet); //Set motor speed.
      CWLR();
      break;
    case 'C': //8 - counterclockwise large radius rotation
      Serial.println("Counterclockwise large radius");
      analogWrite(motPWM, motSet); //Set motor speed.
      CCWLR();
      break;
    case 'T': // balloon following
      Serial.println("Balloon following");
      ballF();
      break;
    default:
      Serial.println("UNKOWN");
      break;
  }
}
