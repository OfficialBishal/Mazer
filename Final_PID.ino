#define left_motor_positive 4
#define left_motor_negative 5
#define right_motor_positive 2
#define right_motor_negative 3
#define en1 11
#define en2 10

#define S1 A5
#define S2 A4
#define S3 A3
#define S4 A2
#define S5 A1
#define S6 A0

#define w 0
#define b 1

#define led 6
#define push_button 7


int IR[6] = {0, 0, 0, 0, 0, 0};

int initial_motor_speed = 150;
int rotating_speed = 60;
int forward_speed = 150;
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;


int error; 
float kp = 32; //proportiona constant
float ki = 0;
float kd = 10;
float P,I,D,previousError=0;;
int pid_value;

char mode;
int Status = 0;
int buttom_reading;


void led_signal(int times);

void calculatePID();
void PIDmotor_control();

void readIRvalue();     //to read sensor value and calculate error as well mode
void Set_motion();

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

void loop() 
{
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
     readIRvalue();
     set_motion();
  }
}
//ReadIRvalue begins----------------------------------------------------------------------------------------------------------------------------------
void readIRvalue()
{
  IR[0] = digitalRead(S1);
  IR[1] = digitalRead(S2);
  IR[2] = digitalRead(S3);
  IR[3] = digitalRead(S4);
  IR[4] = digitalRead(S5);
  IR[5] = digitalRead(S6);
  //--------------------------------------------------------------------------------------
  if (IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == b && IR[4] == b && IR[5] == b)
  {
    mode = 'N';  //NO LINE
    error = 0;
  }
  //--------------------------------------------------------------------------------------
  else if ( IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == w && IR[5] == w)
  {
    mode = 'S';//Stop Condition
    error = 0;
  }
  //--------------------------------------------------------------------------------------

  else if ( IR[0] == b && IR[1] == b && IR[2] == b && IR[3] == w && IR[4] == w && IR[5] == w)
  {
    mode = 'R';//90 degree turn
    error = 0;
  }
  else if ( IR[0] == b && IR[1] == b && IR[4] == w && IR[5] == w)
  {
    mode = 'R';//90 degree turn
    error = 0;
  }
 /* else if ( IR[0] == b && IR[1] == b && IR[2] == w && IR[3] == w && IR[4] == w && IR[5] == b)
  {
    mode = 'R';//45 degree turn
    error = 0;
  }*/
  //-------------------------------------------------------------------------------------
  else if ( IR[0] == w && IR[1] == w && IR[2] == w && IR[3] == b && IR[4] == b && IR[5] == b)
  {
    mode = 'L'; //90 degree turn
    error = 0;
  }
    else if ( IR[0] == w && IR[1] == w && IR[4] == b && IR[5] == b)
  {
    mode = 'L'; //90 degree turn
    error = 0;
  }
  /*else if ( IR[0] == b&& IR[1] == w && IR[2] == w && IR[3] == w && IR[4] == b && IR[5] == b)
  {
    mode = 'L';   //45 degree turn
    error = 0;
  }*/
  //-------------------------------------------------------------------------------------
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
  else if (IR[0] == b && IR[1] == b&& IR[2] == b && IR[3] == b && IR[4] == w && IR[5] == b)
  {
    error = 3;
    mode = 'F';
  }
  //-------------------------------------------------------------------------------------
}


//set motion begins------------------------------------------------------------------------------------------------------------------------------
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
      move_inch();
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

//-----------------------------------------------------------------------------------------------------------
void move_inch()
{
  forward(forward_speed, forward_speed);
  delay(40);
  stop_motor();
}
//----------------------------------------------------------------------------------------------------------

void stop_motor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}
//-----------------------------------------------------------------------------------------------------------
void goAndTurnLeft()
{ 
  previousError = 0;
  left(rotating_speed);
  delay(100);
  do
  {
    left(rotating_speed);
    readIRvalue();
  } while (mode != 'F' && (error != 0 || error != 1 || error != -1 ||error != -2 ||error != -3 ));
  stop_motor();
}

//------------------------------------------------------------------------------------------------
void goAndTurnRight()
{
  previousError = 0;
  right(rotating_speed);
  delay(100);
  do 
  {
    right(rotating_speed);
    readIRvalue();
  }  while (mode != 'F' && (error != 0 || error != 1 || error != -1 ||error != 2 ||error != 3 ));
  stop_motor();
  
}
//-----------------------------------------------------------------------------------------------
void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
}
//------------------------------------------------------------------------------------------------
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  pid_value = (kp*P) + (ki*I) + (kd*D);
  previousError = error;
  
}
//-----------------------------------------------------------------------------------------------
void PIDmotor_control()
{
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);
}

//-------------------------------------------------------------------------------------------------------
void forward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, LOW);
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
}
//--------------------------------------------------------------------------------------
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
      move_inch();
      break;
  }
}
//--------------------------------------------------------------------------------------------------
