#define left_motor_positive 5
#define left_motor_negative 4
#define right_motor_positive 3
#define right_motor_negative 2
#define en1 11
#define en2 10

#define frontsensor 13
int fs = 0;

#define S1 A5
#define S2 A4
#define S3 A3
#define S4 A2
#define S5 A1
#define S6 A0

#define F1 6
#define F2 7
#define F3 8

#define w 0
#define b 1

#define led 12
#define push_button 9


int IR[6] = {0, 0, 0, 0, 0, 0};
int IRF[3] = {0, 0, 0};

int initial_motor_speed = 235;
int rotating_speed = 80;
int forward_speed = 235;
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;

int trigpin = 9;
int echopin = 8;
long duration = 0;
long distance = 0;

int move_certain = 50;


int error;
float kp = 78; //proportiona constant
float ki = 0;
float kd = 125;
float P, I, D, previousError = 0;;
int pid_value;

char mode;
int Status = 0;
int buttom_reading;

void led_signal(int times);

void calculatePID();
void PIDmotor_control();

void readIRvalueAlpha();
void readIRvalueAlpha1();
void readIRvalue();     //to read sensor value and calculate error as well mode
void readIRvalue1();
void readIRvalue2();
void Set_motion();
void Set_motion1();

void dryrun();
void actualrun();

void recIntersection(char);
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
void setmotionactual();
void mazeTurn (char dir);

void forward(int spd1, int spd2);
void left(int spd);
void right(int spd);
void stop_motor();
void goAndTurnLeft();
void maze_end();
void move_inch();
void backward(int spd1, int spd2);
void USsensor();

int i = 0;
int j = 0;

void loopreducing();
void finalreducing();
void manage();
void manageR();
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 2; i <= 5; i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(F1, INPUT);
  pinMode(F2, INPUT);
  pinMode(F3, INPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(push_button, INPUT);
  pinMode(echopin, INPUT);
  pinMode(trigpin, OUTPUT);
  pinMode(frontsensor, INPUT);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
  //pinMode(frontir, INPUT);
}

void loop()
{ 
  readIRvalueAlpha();
  if (mode == 'F' && error == 0) {
    digitalWrite(led, HIGH);
  }
  digitalWrite(led, LOW);
  // put your main code here, to run repeatedly:

  buttom_reading = digitalRead(push_button);
  if (buttom_reading == HIGH && Status == 0)
  {
    led_signal(5);
    dryrun();
  }
  else if (buttom_reading == HIGH && Status == 1)
  {
    led_signal(5);
    actualrun();
    Status = 1;
    pathIndex = 0;
  }
}

void led_signal(int times)
{
  for (int i = 0; i <= times; i++)
  {
    digitalWrite(led, HIGH);
    delay(50);
    digitalWrite(led, LOW);
    delay(50);
  }
}

//dryrun begins------------------------------------------------------------------------------------------------------------------
void dryrun()
{
  while (Status == 0)
  {
    readIRvalueAlpha();
    //USsensor();
    set_motion();
  }
}

//ReadIRvalue begins----------------------------------------------------------------------------------------------------------------------------------
void readIRvalueAlpha() {
  readIRvalue();
  readIRvalue1();
  readIRvalue2();

  //display();
}
void readIRvalueAlpha1()
{
  IR[0] = digitalRead(S1);
  IR[1] = digitalRead(S2);
  IR[2] = digitalRead(S3);
  IR[3] = digitalRead(S4);
  IR[4] = digitalRead(S5);
  IR[5] = digitalRead(S6);
  IRF[0] = digitalRead(F1);
  IRF[1] = digitalRead(F2);
  IRF[2] = digitalRead(F3);
  //fs = digitalRead(frontsensor);

  //  if (fs == w) {
  //    mode = "O";
  //    error = 0;
  //  }

  if ( IR[0] == w && IR[1] == w && IR[2] == w && IR[4] == b && IR[5] == b && IRF[0] == b && IRF[1] == b && IRF[2] == b) //for pure left
  {
    mode = 'L';
    error = 0;
  }
  else if (IR[3] == w && IRF[0] == w && IRF[1] == b && IRF[2] == b) //FOR PURE LEFT AT 135
  {
    mode = 'L';
    error = 0;
  }
  else if (IR[0] == w && IR[1] == w && IR[2] == w && IR[4] == b && IR[5] == b && IRF[0] == b && IRF[1] == b && IRF[2] == w) //pure left(90) and right(45)
  {
    mode = 'L';
    error = 0;
  }
  else if (IR[0] == w && IR[1] == w && IR[2] == w && IR[4] == b && IR[5] == b && IRF[0] == b && IRF[1] == w && IRF[2] == b) //straight and left(90)
  {
    mode = 'L';
    error = 0;
  }

  else if ( IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == w && IR[5] == w && IRF[0] == b && IRF[1] == b && IRF[2] == b ) // AT T
  {
    mode = 'L';
    error = 0;
  }

  else if (IR[0] == b && IR[1] == b && IR[3] == w && IR[4] == w && IR[5] == w && IRF[0] == b && IRF[1] == b && IRF[2] == b) //pure right(90)
  {
    mode = 'R';
    error = 0;
  }
  else if (IR[0] == b && IR[1] == b && IR[3] == w && IR[4] == w && IR[5] == w && IRF[0] == w && IRF[1] == b && IRF[2] == b) //left(135) and pure right(90)
  {
    mode = 'L';
    error = 0;
  }
  else if (IR[2] == w && IRF[0] == b && IRF[1] == b && IRF[2] == w) //pure right(45)
  {
    mode = 'R';
    error = 0;
  }
  else if (IR[0] == b && IR[1] == b && IR[3] == w && IR[4] == w && IR[5] == w && IRF[0] == b && IRF[1] == w && IRF[2] == b) //straight right
  {
    mode = 'S';
    error = 0;
  }
  else if (IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == w && IR[5] == w && IRF[0] == b && IRF[1] == w && IRF[2] == b) //cross
  {
    mode = 'L';
    error = 0;
  }
  else if ((IR[2] == w || IR[3] == w ) && IRF[0] == w && IRF[1] == b && IRF[2] == w) //case Y
  {
    mode = 'L';
    error = 0;
  }

  else if (IR[0] == b && IR[1] == b && IR[3] == b && IR[4] == b && IR[5] == b && IRF[0] == b && IRF[1] == b && IRF[2] == b) //dead end
  {
    mode = 'N';
    error = 0;
  }

  else if (IR[1] == w && IR[2] == b && IR[3] == b && IR[4] == b )
  {
    error = -2;
    mode = 'F';
  }
  else if (IR[1] == w && IR[2] == w && IR[3] == b && IR[4] == b )
  {
    error = -1;
    mode = 'F';
  }
  else if (IR[1] == b && IR[2] == w && IR[3] == w && IR[4] == b)
  {
    error = 0;
    mode = 'F';
  }

  else if (IR[1] == b && IR[2] == b && IR[3] == w && IR[4] == w)
  {
    error = 1;
    mode = 'F';
  }
  else if (IR[1] == b && IR[2] == b && IR[3] == b && IR[4] == w)
  {
    error = 2;
    mode = 'F';
  }

}


void readIRvalue() {
  IR[0] = digitalRead(S1);
  IR[1] = digitalRead(S2);
  IR[2] = digitalRead(S3);
  IR[3] = digitalRead(S4);
  IR[4] = digitalRead(S5);
  IR[5] = digitalRead(S6);

  IRF[0] = digitalRead(F1);
  IRF[1] = digitalRead(F2);
  IRF[2] = digitalRead(F3);

  //fs = digitalRead(frontsensor);
}
void readIRvalue1() {
  //------------------------------------------------------------------------------------------
  if (IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == b && IR[4] == b && IR[5] == b)
  {
    mode = 'N';  //NO LINE
    error = 0;
  }
  //------------------------------------------------------------------------------------------
  else if ( IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == w && IR[5] == w)
  {
    mode = 'S';//Stop Condition
    error = 0;
  }
  //-----------------------------Check for Right & Left-------------------------
  else if ( IR[0] == b && IR[1] == b && IR[4] == w && IR[5] == w)
  {
    mode = 'R';//90 degree turn
    error = 0;
  }
  else if ( IR[0] == w && IR[1] == w && IR[4] == b && IR[5] == b)
  {
    mode = 'L'; //90 degree turn
    error = 0;
  }
  //---------------------------------------Forward------------------------------------------
  else if (IR[0] == b && IR[1] == w && IR[2] == b && IR[3] == b && IR[4] == b && IR[5] == b)
  {
    error = -3 ;
    mode = 'F';
  }
  else if (IR[0] == b && IR[1] == w && IR[2] == w && IR[3] == b && IR[4] == b && IR[5] == b)
  {
    error = -2;
    mode = 'F';
  }
  else if (IR[0] == b && IR[1] == b && IR[2] == w && IR[3] == b && IR[4] == b && IR[5] == b)
  {
    error = -1;
    mode = 'F';
  }
  else if (IR[0] == b && IR[1] == b && IR[2] == w && IR[3] == w && IR[4] == b && IR[5] == b)
  {
    error = 0;
    mode = 'F';
  }
  else if (IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == w && IR[4] == b && IR[5] == b)
  {
    error = 1;
    mode = 'F';
  }
  else if (IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == w && IR[4] == w && IR[5] == b)
  {
    error = 2;
    mode = 'F';
  }
  else if (IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == b && IR[4] == w && IR[5] == b)
  {
    error = 3;
    mode = 'F';
  }
}
void readIRvalue2() {
  //----------------------------------------------------------------------------------------------------------------------------
  if (mode == 'F' && (error == 0 || error == 1 || error == -1)) {

    if (IRF[0] == b && IRF[1] == w && IRF[2] == w) {
//      move_inch();
//      readIRvalue();
//      readIRvalue1();
      do {
        calculatePID();
      PIDmotor_control();
      readIRvalue();
      readIRvalue1();
      }while((mode == 'R' && (IRF[0] != b && IRF[1] != b && IRF[2] != b)) || (mode != 'R' && (IRF[0] != b && IRF[1] != w && IRF[2] != b)) || (mode != 'F' && (IRF[0] != w && IRF[1] != b && IRF[2] != b)) );
      if (mode == 'R' && (IRF[0] == b && IRF[1] == b && IRF[2] == b)) {
        mode = 'R';
      }
      else if (mode == 'R' && (IRF[0] == b && IRF[1] == w && IRF[2] == b)) {
        mode = 'Z';
      }
      else if (mode == 'F' && (IRF[0] == w && IRF[1] == b && IRF[2] == b)) {
        mode = 'L';
      }
    }
    else if (IRF[0] == w && IRF[1] == w && IRF[2] == b) {
      move_inch();
      readIRvalue();
      readIRvalue1();
      if (mode == 'L' && (IRF[0] == b && IRF[1] == b && IRF[2] == b)) {
        mode = 'L';
      }
      else if (mode == 'L' && (IRF[0] == b && IRF[1] == w && IRF[2] == b)) {
        mode = 'L';
      }
      else if (mode == 'F' && (IRF[0] == b && IRF[1] == b && IRF[2] == w)) {
        mode = 'L';
      }
    }

    else if (IRF[0] == b && IRF[1] == b && IRF[2] == w) {
      move_inch();
      readIRvalue();
      readIRvalue1();
      if (mode == 'F' && (IRF[0] == b && IRF[1] == b && IRF[2] == b)) {
        mode = 'R';
      }
    }
    else if (IRF[0] == w && IRF[1] == b && IRF[2] == b) {
      move_inch();
      readIRvalue();
      readIRvalue1();
      if (mode == 'F' && (IRF[0] == b && IRF[1] == b && IRF[2] == b)) {
        mode = 'L';
      }
    }
    if (IRF[0] == w && IRF[1] == b && IRF[2] == w) {
      move_inch();
      readIRvalue();
      readIRvalue1();

      mode = 'L';

    }
  }
}



//set motion begins------------------------------------------------------------------------------------------------------------------------------
void set_motion1()
{
  switch (mode)
  {
    case 'O':
      stop_motor();
      backward(forward_speed, forward_speed);
      delay(10);
      goAndTurnLeft();
      recIntersection('B');
    case 'L':
      // led_signal(1);
      goAndTurnLeft();
      recIntersection('L');
      break;

    case 'R':
      //led_signal(1);
      goAndTurnRight();
      recIntersection('R');
      break;

    case 'Z':
      //led_signal(1);
      recIntersection('S');
      break;

    case 'S':
      move_inch();
      readIRvalueAlpha();
      if (mode == 'S')
      {
        maze_end();
      }
      break;

    case 'N':
      stop_motor();
      goAndTurnLeft();
      recIntersection('B');
      break;

    case 'F':
      calculatePID();
      PIDmotor_control();
      break;

  }
}

void set_motion()
{
  switch (mode)
  {
    case 'N':
      stop_motor();
      goAndTurnLeft();
      recIntersection('B');
      break;
    case 'S':
      move_inch();
      readIRvalue();
      readIRvalue1();
      if (mode == 'S')
      {
        maze_end();
      }
      else
      {
        goAndTurnLeft();
        recIntersection('L');
      }
      break;
    case'R':
      goAndTurnRight();
      recIntersection('R');
      break;
    case 'Z':
      recIntersection('S');
      break;
    case 'L':
      goAndTurnLeft();
      recIntersection('L');
      break;
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
  }
}


void forward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, LOW);
  Serial.print("FORWARD MOTION....");
  Serial.println(spd1);
  Serial.println(spd2);
}

void backward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, HIGH);
  Serial.print("FORWARD MOTION....");
  Serial.println(spd1);
  Serial.println(spd2);
}

void left(int spd)
{
  analogWrite(en1, spd+10);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, HIGH);
  Serial.println("Left_turn.........");
  Serial.println(spd);
}

void right(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd-10);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, HIGH);
  Serial.println("Right turn mode.......");
  Serial.println(spd);
}

void stop_motor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
  Serial.println("stop motor.......");
}

void move_extra()
{
  forward(forward_speed, forward_speed);
  delay(move_certain);
  backward(forward_speed, forward_speed);
  stop_motor();
}

void move_inch()
{
  forward(forward_speed, forward_speed);
  delay(move_certain);
  backward(forward_speed, forward_speed);
  stop_motor();
}

void goAndTurnLeft()
{
  stop_motor();
  backward(forward_speed, forward_speed);
  delay(1);
  stop_motor();
  //while(1);
  move_extra();
  readIRvalueAlpha();
  left(rotating_speed);
  delay(250);
  do {
    left(rotating_speed);
    readIRvalueAlpha();
  } while (IR[2] != w && IR[3] != w && IRF[1] != w);
  stop_motor();
  right(rotating_speed);
  delay(5);
  stop_motor();
  left(rotating_speed);
  delay(5);
  stop_motor();
  //while(1);
  //delay(50);
}

void goAndTurnRight()
{
  stop_motor();
  backward(forward_speed, forward_speed);
  delay(1);
  stop_motor();
  //while(1);
  move_extra();
  readIRvalueAlpha();
  right(rotating_speed);
  delay(250);
  do {
    right(rotating_speed);
    readIRvalueAlpha();
  } while (IR[2] != w && IR[3] != w && IRF[1] != w);
  stop_motor();
  left(rotating_speed);
  delay(5);
  stop_motor();
  right(rotating_speed);
  delay(5);
  stop_motor();\
  //while(1);
  //delay(50);

  //
  //  stop_motor();
  //  backward(forward_speed, forward_speed);
  //  delay(50);
  //  stop_motor();
  //  move_extra();
  //  readIRvalue();
  //  right(rotating_speed);
  //  delay(425);
  //  do {
  //    right(rotating_speed);
  //    readIRvalue();
  //  }  while (IR[2] != w && IR[3] != w && IRF[1] != w);
  //  stop_motor();
  //  left(rotating_speed);
  //  delay(50);
  //  stop_motor();
  //
}

void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
  //led_end(5000);
}


void led_end(int timess)
{
  Serial.println("led signal on......");
  digitalWrite(led, HIGH);
  delay(timess);
  digitalWrite(led, LOW);
}


void calculatePID()
{
  P = error;
  I = I + error;
  D = error - previousError;
  pid_value = (kp * P) + (ki * I) + (kd * D);
  previousError = error;
}

void PIDmotor_control()
{
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);

}


//actualrun-----------------------------------------------------------------------------------------------------
void actualrun(void)
{
  while (Status == 1)
  {
    readIRvalueAlpha();
    setmotionactual();
  }
}

//actual runfunctions--------------------------------------------------------------------------------------------
void setmotionactual()
{
  switch (mode)
  {
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
    case 'N':
      if (pathIndex >= pathLength)
        maze_end();
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      break;
    case 'L':
      if (pathIndex >= pathLength)
        maze_end();
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      break;
    case 'R':
      if (pathIndex >= pathLength)
        maze_end();
      else
      {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
      break;
  }

}

void recIntersection(char Direction)
{
  path[pathLength] = Direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
}

void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (pathLength < 3 || path[pathLength - 2] != 'B')
    return;

  int totalAngle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[pathLength - i])
    {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle)
  {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'R';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'L';
      break;
  }
  // The path is now two steps shorter.
  pathLength -= 2;
}

void mazeTurn (char dir)
{
  switch (dir)
  {
    case 'L': // Turn Left
      goAndTurnLeft();
      break;

    case 'R': // Turn Right
      goAndTurnRight();
      break;

    case 'B': // Turn Back
      goAndTurnLeft();
      break;

    case 'S': // Go Straight
      move_extra();
      break;
  }
}

//loop fixing functions--------------------------------------------------------------------------------------------
void loopreducing() {
  i = 0;
  while ((i + 4) < pathLength) {
    if (path[i + 1] == 'R' && path[i + 2] == 'R' && path[i + 3] == 'R' && path[i] == 'S' && path[i + 4] == 'L') {
      path[i] = 'B';
      j = i + 1;
      manage();
    }
    else if (path[i + 1] == 'R' && path[i + 2] == 'R' && path[i + 3] == 'R' && path[i] == 'L' && path[i + 4] == 'S') {
      path[i] = 'B';
      j = i + 1;
      manage();
    }
    else if (path[i + 1] == 'R' && path[i + 2] == 'R' && path[i + 3] == 'R' && path[i] == 'L' && path[i + 4] == 'L') {
      path[i] = 'R';
      j = i + 1;
      manage();
    }
    else if (path[i + 1] == 'R' && path[i + 2] == 'R' && path[i + 3] == 'R' && path[i + 4] == 'R' && path[i] == 'L' && path[i + 5] == 'L') {
      path[i] = 'B';
      j = i + 1;
      manageR();
    }
    i++;
  }
}

void finalreducing() {
  j = 0;
  while ((j + 2) < pathLength) {
    // only simplify the path if the second-to-last turn was a 'B'
    if (path[j + 1] != 'B') {
      j++;
      return;
    }
    int totalAngle = 0;
    int i;
    for (i = 0; i <= 2; i++)
    {
      switch (path[j + i])
      {
        case 'R':
          totalAngle += 90;
          break;
        case 'L':
          totalAngle += 270;
          break;
        case 'B':
          totalAngle += 180;
          break;
      }
    }

    // Get the angle as a number between 0 and 360 degrees.
    totalAngle = totalAngle % 360;

    // Replace all of those turns with a single one.
    switch (totalAngle)
    {
      case 0:
        path[j] = 'S';
        break;
      case 90:
        path[j] = 'R';
        break;
      case 180:
        path[j] = 'B';
        break;
      case 270:
        path[j] = 'L';
        break;
    }
    int k = j + 1;
    while ((k + 2) < pathLength) {
      path[k] = path[k + 2];
      k++;
    }
    // The path is now two steps shorter.
    pathLength -= 2;
    j++;
  }
}

void manage() {
  while ((j + 4) < pathLength) {
    path[j] = path[j + 4];
    j++;
  }
  pathLength = pathLength - 4;
}

void manageR() {
  while ((j + 5) < pathLength) {
    path[j] = path[j + 5];
    j++;
  }
  pathLength = pathLength - 5;
}
