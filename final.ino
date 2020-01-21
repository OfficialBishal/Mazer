#define left_motor_positive 2
#define left_motor_negative 3
#define right_motor_positive 4
#define right_motor_negative 5

#define en1 10
#define en2 11

#define left_turn_sensor 6
#define far_left_sensor A0
#define near_left_sensor A2
#define centre A3
#define near_right_sensor A4
#define far_right_sensor A5
#define right_turn_sensor 7

#define led 8
#define push_button 9

#define w 1
#define b 0

int IR[5] = {0, 0, 0, 0, 0};
int IRA1 = 0, IRA2 = 0;

int initial_motor_speed =200;
int rotating_speed = 100;
int forward_speed = 200;
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;

int move_certain =15;

int error;
int kp =42; //proportiona constant
int ki = 0; //integral   constant
int kd = 0; //defferential constant
int pid_value;
int previousError = 0;

int i=0;
int j=0;

void loopreducing();
void finalreducing();
void manage();
void manageR();

char mode;
int Status = 0;
int buttom_reading;

void led_signal(int times);
void led_end(int timess);

void calculatePID();
void PIDmotor_control();

void readIRvalue();     //to read sensor value and calculate error as well mode
void Set_motion();

void dryrun();
void actualrun();

void recIntersection(char);
void ledBlink(int times);
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
void setmotionactual();
void mazeTurn (char dir);

void forward(int spd1, int spd2);
void left(int spd);
void right(int spd);
void stop_motor();
void take_turn();
void goAndTurnLeft();
void move_extra();
void maze_end();
void move_inch();
void backward(int spd1, int spd2);
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
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
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(push_button, INPUT);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
  buttom_reading = digitalRead(push_button);
  if (buttom_reading == HIGH && Status == 0)
  {
    led_signal(5);
    delay(1000);
    dryrun();
  }
  else if (buttom_reading == HIGH && Status == 1)
  {
    led_signal(5);
    delay(1000);
    //---------For reducing Loop------
    //loopreducing();
    //finalreducing();
    //---------------------------------
    actualrun();
    Status = 1;
    pathIndex = 0;
  }
}

//dryrun begins------------------------------------------------------------------------------------------------------------------
void dryrun()
{
  while (Status == 0)
  {
    readIRvalue();
    set_motion();
  }
}

void readIRvalue()
{
  IR[0] = digitalRead(far_left_sensor);
  IR[1] = digitalRead(near_left_sensor);
  IR[2] = digitalRead(centre);
  IR[3] = digitalRead(near_right_sensor);
  IR[4] = digitalRead(far_right_sensor);
  IRA1 = digitalRead(left_turn_sensor);
  IRA2 = digitalRead(right_turn_sensor);
  if (IRA1==w && IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == w && IRA2==w)
  {
    mode = 'N';  //NO LINE
    error = 0;
  }
  if (IRA1==b && IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == b && IR[4] == b && IRA2==b)
  {
    mode = 'S';
    error = 0;
  }
  if ( IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == b && IR[4] == b)
  {
    mode = 'S';
    error = 0;
  }
 
  else if (IRA1 == w && IR[0] == w && IR[3] == b && IR[4] == b && IRA2 == b)
  {
    mode = 'R';
    error = 0;
  }
  else if (IRA1 == b && IR[0] == b && IR[1] == b && IR[4] == w && IRA2 == w)
  {
    mode = 'L';
    error = 0;
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == b && IRA2 == w)
  {
    error = 5 ;   
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == b && IR[4] == b && IRA2 == w)
  {
    error = 3;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == b && IR[4] == w && IRA2 == w)
  {
    error = 2;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == w && IR[2] == b && IR[3] == b && IR[4] == w && IRA2 == w)
  {
    error = 1;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == w && IR[2] == b && IR[3] == w && IR[4] == w && IRA2 == w)
  {
    error = 0;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == b && IR[2] == b && IR[3] == w && IR[4] == w && IRA2 == w)
  {
    error = -1;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == w && IR[1] == b && IR[2] == w && IR[3] == w && IR[4] == w && IRA2 == w)
  {
    error = -2;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == b && IR[1] == b && IR[2] == w && IR[3] == w && IR[4] == w && IRA2 == w)
  {
    error = -3;
    mode = 'F';
  }
  else if (IRA1 == w && IR[0] == b && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == w && IRA2 == w)
  {
    error = -5;
    mode = 'F';
  }


}


void set_motion()
{
  readIRvalue();
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
      move_extra();
      readIRvalue();
      if (mode == 'N')
      {
        goAndTurnRight();
        recIntersection('R');
      }
      else
      {
        recIntersection('S');
      }
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
  analogWrite(en1, spd);
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
  analogWrite(en2, spd);
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
  delay(20.5);
  backward(forward_speed, forward_speed);
  stop_motor();
}

void goAndTurnLeft()
{ stop_motor();
  move_extra();
  readIRvalue();
  left(rotating_speed);
 delay(425);
  do {
    left(rotating_speed);
    readIRvalue();
  } while (mode != 'F' && (error != 0 || error != 1 || error != -1 || error != -2  ));
  stop_motor();
   right(rotating_speed);
  delay(50);
 // stop_motor();
//delay(50);
}

void goAndTurnRight()
{  stop_motor();
  move_extra();
  readIRvalue();
  right(rotating_speed);
  delay(425);
  do {
    right(rotating_speed);
    readIRvalue();
  }  while (mode != 'F' && (error != 0 || error != 1 || error != -1 ||error != 2 ));
  stop_motor();
  left(rotating_speed);
  delay(50);
  //stop_motor();
  
}

void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
  //led_end(5000);
}

void led_signal(int times)
{
  Serial.println("led signal on......");
  for (int i = 0; i <= times; i++)
  {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }

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
  pid_value = (kp * error);
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
    readIRvalue();
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
void loopreducing(){
  i = 0;
  while((i+4) < pathLength){
    if(path[i+1]== 'R' && path[i+2]== 'R' && path[i+3]== 'R' && path[i]== 'S' && path[i+4]== 'L'){
      path[i]= 'B';
      j=i+1;
      manage();
    }
    else if(path[i+1]== 'R' && path[i+2]== 'R' && path[i+3]== 'R' && path[i]== 'L' && path[i+4]== 'S'){
      path[i]= 'B';
      j=i+1;
      manage();
    }
    else if(path[i+1]== 'R' && path[i+2]== 'R' && path[i+3]== 'R' && path[i]== 'L' && path[i+4]== 'L'){
      path[i]= 'R';
      j=i+1;
      manage();
    }
    else if(path[i+1]== 'R' && path[i+2]== 'R' && path[i+3]== 'R' && path[i+4]== 'R' && path[i]== 'L' && path[i+5]== 'L'){
      path[i]= 'B';
      j=i+1;
      manageR();
    }
    i++;  
  }
}

void finalreducing(){
  j = 0;
  while((j+2) < pathLength){
    // only simplify the path if the second-to-last turn was a 'B'
  if (path[j + 1] != 'B'){
    j++;
    return;
  }
  int totalAngle = 0;
  int i;
  for (i = 0; i <= 2; i++)
  {
    switch (path[j+i])
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
  while((k+2) < pathLength){
    path[k] = path[k+2];
    k++;  
  }
  // The path is now two steps shorter.
  pathLength -= 2;
  j++;
  }
}

void manage(){
  while((j+4) < pathLength){
    path[j] = path[j+4];
    j++;  
  }
  pathLength = pathLength - 4;
}

void manageR(){
  while((j+5) < pathLength){
    path[j] = path[j+5];
    j++;  
  }
  pathLength = pathLength - 5;
}
