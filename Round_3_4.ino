#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN_L1  4
#define ECHO_PIN_L1  5

#define TRIGGER_PIN_L2  6
#define ECHO_PIN_L2  7

#define TRIGGER_PIN_R1  2
#define ECHO_PIN_R1  3

#define TRIGGER_PIN_R2  28
#define ECHO_PIN_R2  29

#define TRIGGER_PIN_F 10
#define ECHO_PIN_F  11

#define MAX_DISTANCE 500

#define sumo_p 34
#define sumo_n 37
#define sumo 12

Servo myservo;

int counter = 0;
int counter_1 = 0;

int pos = 75;
int turn_pos_hard = 45;
int turn_pos_L = 29;
int Position;
int position_angle = 0;
double jbes = 0;
double pos_distance;
int pos_cube = 30;

int pwm = 95;
int pwm_back = 125;

double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0;

double distance = 0, real_distance = 0 , Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0, dis_R1 = 0, dis_R2 = 0, dis_F = 0;

double angle = 0, Angle = 0 , aangle = 0, AAngle = 0, highest_angle = 0.24;

double conskp = 0.5, kp_angle = 1;

double Setpoint = 39;
String data;
double j = 0;
double a = 0;

NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);


NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);


NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);


NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);


NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);


double angle_cal (double L1 , double L2)
{
  Angle =  atan ((L1 - L2 ) / 13 );
  if (Angle > highest_angle)
  {
    Angle =  highest_angle;
  }
  if (Angle < ((-1)*highest_angle))
  {
    Angle = (-1) * highest_angle;
  }
  return Angle;
  //Radian
}

double PI_angle_Left(double L1 , double L2)
{
  angle =  angle_cal (L1, L2);
  Angle = (angle * 180) / 3.14;
  Angle = Angle * kp_angle;
  position_angle = pos - Angle;
  if (position_angle - pos > -1 && position_angle < 1) {
    position_angle = 0;
  }
  myservo.write(position_angle);
}

double PI_angle_Right(double R1 , double R2) {
  angle =  angle_cal (R1, R2);
  Angle = (angle * 180) / 3.14;
  Angle = Angle * kp_angle;
  position_angle = pos + Angle;
  if (position_angle - pos > -1 && position_angle < 1) {
    position_angle = 0;
  }
  myservo.write(position_angle);
}

double Distance(double L1 , double L2)
{
  distance = (L1 + L2) / 2;
  angle = angle_cal (L1, L2);
  real_distance = distance * cos(angle);
  return real_distance;
}

double P_distanceL (double real_distance)
{
  pos_distance = conskp * (real_distance - Setpoint);
  jbes = pos - pos_distance;
  myservo.write(jbes);
}

double P_distanceR (double real_distance)
{
  pos_distance = conskp * (real_distance - Setpoint);
  jbes = pos + pos_distance;
  myservo.write(jbes);
}

void Run_Sumo (int pwm)
{
  digitalWrite (sumo_p , HIGH);
  digitalWrite (sumo_n , LOW);
  analogWrite (sumo, pwm);
}

void Back_Sumo (int pwm)
{
  digitalWrite (sumo_p , LOW);
  digitalWrite (sumo_n , HIGH);
  analogWrite (sumo, pwm);
}

void Stop_Sumo (int pwm)
{
  digitalWrite (sumo_p , LOW);
  digitalWrite (sumo_n , LOW);
  analogWrite (sumo, pwm);
}

double get_distance_F ()
{
  double duration_F = sonar_F.ping();
  return (duration_F / 2) * 0.0343 ;
}

double get_distance_L1 ()
{
  double duration_L1 = sonar_L1.ping();
  return ((duration_L1 / 2) * 0.0343) + 6 ;
}

double get_distance_L2()
{
  double duration_L2 = sonar_L2.ping();
  return ((duration_L2 / 2) * 0.0343) + 6 ;
}

double get_distance_R1 ()
{
  double duration_R1 = sonar_R1.ping();
  return ((duration_R1 / 2) * 0.0343) + 6 ;
}

double get_distance_R2()
{
  double duration_R2 = sonar_R2.ping();
  return ((duration_R2 / 2) * 0.0343) + 6 ;
}

double straight_section_L (double Real_Dis_L , double Real_Dis_R , double Dis_L1 , double Dis_L2)
{
  if (Real_Dis_L < Setpoint)
  {
    P_distanceL(Real_Dis_L);
  }

  if (Real_Dis_R < Setpoint)
  {
    P_distanceR(Real_Dis_R);
  }

  if (Real_Dis_L > Setpoint && Real_Dis_R > Setpoint)
  {
    PI_angle_Left(Dis_L1, Dis_L2);
  }
}

double straight_section_R (double Real_Dis_L , double Real_Dis_R , double Dis_R1, double Dis_R2)
{
  if (Real_Dis_L < Setpoint)
  {
    P_distanceL(Real_Dis_L);
  }

  if (Real_Dis_R < Setpoint)
  {
    P_distanceR(Real_Dis_R);
  }

  if (Real_Dis_L > Setpoint && Real_Dis_R > Setpoint)
  {
    PI_angle_Right(Dis_R1, Dis_R2);
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN_L1, OUTPUT);
  pinMode(ECHO_PIN_L1, INPUT);
  pinMode(TRIGGER_PIN_L2, OUTPUT);
  pinMode(ECHO_PIN_L2, INPUT);
  pinMode(TRIGGER_PIN_R1, OUTPUT);
  pinMode(ECHO_PIN_R1, INPUT);
  pinMode(TRIGGER_PIN_R2, OUTPUT);
  pinMode(ECHO_PIN_R2, INPUT);
  pinMode(TRIGGER_PIN_F, OUTPUT);
  pinMode(ECHO_PIN_F, INPUT);
  pinMode(sumo_p, OUTPUT);
  pinMode(sumo_n, OUTPUT);
  pinMode(sumo, OUTPUT);
  myservo.attach (13);
}



void loop() {

  dis_L2 = get_distance_L2();

  dis_R2 = get_distance_R2();
  delay(10);

  dis_L1 = get_distance_L1();

  dis_R1 = get_distance_R1();

  dis_F = get_distance_F();


  Real_dis_L = Distance(dis_L1, dis_L2);
  Real_dis_R = Distance(dis_R1, dis_R2);
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('#');
    j = data.toDouble();
    a = j;
  }


  if (a == 3) {
    Run_Sumo(pwm);
    jbes = pos - pos_cube;
    myservo.write(jbes);
    delay(300);
    myservo.write(pos);
    delay(100);
  }

  if (a == 6) {
    Run_Sumo(pwm);
    jbes = pos + pos_cube;
    myservo.write(jbes);
    delay(30);
  }

  if (a == 8) {
    if (dis_L1 < 85 && dis_R1 < 85)
    {
      //counter = 0;
      Run_Sumo(pwm);
      if (Real_dis_L < Setpoint)
      {
        P_distanceL(Real_dis_L);
      }
      if (Real_dis_R < Setpoint)
      {
        P_distanceR(Real_dis_R);
      }

      if (Real_dis_L > Setpoint && Real_dis_R > Setpoint)
      {
        PI_angle_Left(dis_L1, dis_L2);
      }
    }
    if (dis_F < 100 && dis_F > 30 && dis_L1 > 80 ) { //&& counter == 0
      if (dis_L1 > 80 && dis_R1 < 80) {
        PI_angle_Right(dis_R1, dis_R2);
      }
    }
    if (dis_F < 100 && dis_F > 30 && dis_R1 > 80 ) { //&& counter == 0
      if (dis_R1 > 80 && dis_L1 < 80) {
        PI_angle_Left(dis_L1, dis_L2);
      }
    }
    if (dis_F < 15 && dis_L1 > 80 && dis_R1 < 80)
    {
      counter = 1;
      dis_R2 = get_distance_R2();
      delay (10);
      dis_R1 = get_distance_R1();
      aangle = angle_cal(dis_R1, dis_R2);
      AAngle = (aangle * 180) / 3.14;
      Position = pos + turn_pos_hard;
      myservo.write(Position);
      Back_Sumo (pwm_back);
      delay (1700);

      while (AAngle < -5 && counter == 1) {
        Back_Sumo (pwm_back);
        dis_R2 = get_distance_R2();
        delay (10);
        dis_R1 = get_distance_R1();
        aangle = angle_cal(dis_R1, dis_R2);
        AAngle = (aangle * 180) / 3.14;

        Position = pos + turn_pos_hard;
        myservo.write(Position);

      }
      dis_R2 = get_distance_R2();
      delay (10);
      dis_R1 = get_distance_R1();
      aangle = angle_cal(dis_R1, dis_R2);
      AAngle = (aangle * 180) / 3.14;

      dis_L2 = get_distance_L2();
      delay(10);
      dis_L1 = get_distance_L1();

      Real_dis_L = Distance(dis_L1, dis_L2);

      while (Real_dis_L > 60) {
        Run_Sumo(pwm);
        dis_R2 = get_distance_R2();
        dis_L1 = get_distance_L1();
        delay(10);
        dis_R1 = get_distance_R1();
        dis_L2 = get_distance_L2();
        Real_dis_L = Distance(dis_L1, dis_L2);
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
          j = data.toDouble();
          a = j;
        }
        if (a == 3) {
          Run_Sumo(pwm);
          jbes = pos - pos_cube;
          myservo.write(jbes);
          delay(300);
          myservo.write(pos);
          delay(100);
        }

        if (a == 6) {
          Run_Sumo(pwm);
          jbes = pos + pos_cube;
          myservo.write(jbes);
          delay(30);
        }
        if (a != 3 && a != 6) {
          if (Real_dis_R > Setpoint)
          {
            PI_angle_Right(dis_R1, dis_R2);
          }
        }
      }

    }
    if (dis_F < 15 && dis_R1 > 80 && dis_L1 < 80)
    {
      counter = 1;
      dis_L2 = get_distance_L2();
      delay (10);
      dis_L1 = get_distance_L1();
      aangle = angle_cal(dis_L1, dis_L2);
      AAngle = (aangle * 180) / 3.14;
      Position = pos - turn_pos_hard;
      myservo.write(Position);
      Back_Sumo (pwm_back);
      delay (1700);

      while (AAngle < - 5 && counter == 1) {
        Back_Sumo (pwm_back);
        dis_L2 = get_distance_L2();
        delay (10);
        dis_L1 = get_distance_L1();
        aangle = angle_cal(dis_L1, dis_L2);
        AAngle = (aangle * 180) / 3.14;

        Position = pos - turn_pos_hard;
        myservo.write(Position);

      }
      dis_R2 = get_distance_R2();
      delay (10);
      dis_R1 = get_distance_R1();
      aangle = angle_cal(dis_R1, dis_R2);
      AAngle = (aangle * 180) / 3.14;

      dis_L2 = get_distance_L2();
      delay(10);
      dis_L1 = get_distance_L1();

      Real_dis_R = Distance(dis_R1, dis_R2);

      while (Real_dis_R > 60) {
        Run_Sumo(pwm);
        dis_R2 = get_distance_R2();
        dis_L1 = get_distance_L1();
        delay(10);
        dis_R1 = get_distance_R1();
        dis_L2 = get_distance_L2();
        Real_dis_R = Distance(dis_R1, dis_R2);
        if (Serial.available() > 0) {
          data = Serial.readStringUntil('#');
          j = data.toDouble();
          a = j;
        }
        if (a == 3) {
          Run_Sumo(pwm);
          jbes = pos - pos_cube;
          myservo.write(jbes);
          delay(300);
          myservo.write(pos);
          delay(100);
        }

        if (a == 6) {
          Run_Sumo(pwm);
          jbes = pos + pos_cube;
          myservo.write(jbes);
          delay(30);
        }
        if (a != 3 && a != 6) {
          if (Real_dis_L > Setpoint)
          {
            PI_angle_Left(dis_L1, dis_L2);
          }
        }
      }

    }


  }
}
