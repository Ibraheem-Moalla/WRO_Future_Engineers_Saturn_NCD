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
int current = 0;
int last = 0;
int counter = 0;
int counter_1 = 0;
Servo myservo;
int pos = 75, turn_pos = 39;
int turn_pos2 = 39;
int iteration = 3;
double pos_distance;
double jbes = 0;
int pwm_2 = 120;
int pwm = 100;
int pwm_1 = 90;
double kp_angle = 1;
int Position;
int position_angle = 0;
double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0;

double distance = 0, real_distance = 0 , Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0, dis_R1 = 0, dis_R2 = 0, dis_F = 0, D1 = 0, D2 = 0;

double angle = 0, Angle = 0, temp_angle = 0;

double conskp = 7;

double Setpoint = 15, setpoint_angle = 0.225;

double highest_angle = 0.24;


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

double PI_angle_Left(double L1 , double L2) {

  angle =  angle_cal (L1, L2);

  Angle = (angle * 180) / 3.14;

  //if (Angle > 5 && Angle <-5){
  Angle = Angle * kp_angle;
  position_angle = pos - Angle;
  /*if (position_angle > 60) {position_angle = 60;}
    if (position_angle <0) { position_angle = 0;}*/
  if (position_angle - pos > -1 && position_angle < 1) {
    position_angle = 0;
  }


  //delay(500);
  //
  myservo.write(position_angle);
  /*}
    else {
    myservo.write(123);
    }

  */
}

double PI_angle_Right(double R1 , double R2) {

  angle =  angle_cal (R1, R2);

  Angle = (angle * 180) / 3.14;

  //if (Angle > 5 && Angle <-5){
  Angle = Angle * kp_angle;
  position_angle = pos + Angle;
  /*if (position_angle > 60) {position_angle = 60;}
    if (position_angle <0) { position_angle = 0;}*/
  if (position_angle - pos > -1 && position_angle < 1) {
    position_angle = 0;
  }


  //delay(500);
  //
  myservo.write(position_angle);
  /*}
    else {
    myservo.write(123);
    }

  */
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

double turn_left(double dis_FF)
{
  Position = pos - turn_pos2;
  myservo.write(Position);
  delay(50);
  while (dis_FF < 150) {
    myservo.write(Position);
    duration_F = sonar_F.ping();
    dis_FF = (duration_F / 2) * 0.0343;
    delay(10);
    //counter=counter+1;
  }
}


double turn_right(double dis_FF)
{
  //  Are 60 drgrees enough?!
  Position = pos + turn_pos;
  myservo.write(Position);
  delay(50);
  while (dis_FF < 150) {
    myservo.write(Position);
    duration_F = sonar_F.ping();
    dis_FF = (duration_F / 2) * 0.0343;
    delay(10);
  }
  //counter=counter+1;

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


  duration_L2 = sonar_L2.ping();
  dis_L2 = ((duration_L2 / 2) * 0.0343) + 6;
  delay(10);

  duration_R2 = sonar_R2.ping();
  dis_R2 = ((duration_R2 / 2) * 0.0343) + 6;
  delay(10);


  duration_L1 = sonar_L1.ping();
  dis_L1 = ((duration_L1 / 2) * 0.0343) + 6;
  delay(10);



  duration_R1 = sonar_R1.ping();
  dis_R1 = ((duration_R1 / 2) * 0.0343) + 6;
  delay(10);


  duration_F = sonar_F.ping();
  dis_F = (duration_F / 2) * 0.0343;
  delay(10);


  Real_dis_L = Distance(dis_L1, dis_L2);
  //   Serial.print("Counter = ");
  //   Serial.println(counter_1);
  Serial.print("dis = ");
  Serial.println(dis_F);

  //Radian or degree ???!!


  Real_dis_R = Distance(dis_R1, dis_R2);
if (counter_1 < 11){
  if (counter == 0 && dis_F > 100) {
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm_2);
    if (Real_dis_L < 15)
    {
      P_distanceL(Real_dis_L);
    }

    if (Real_dis_R < 15)
    {
      P_distanceR(Real_dis_R);
    }

    if (Real_dis_L > 15 && Real_dis_R > 15)
    {
      PI_angle_Left(dis_L1, dis_L2);
    }

  }
  if (counter == 0 && dis_F < 100) {
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm_1);
    duration_L1 = sonar_L1.ping();
    dis_L1 = ((duration_L1 / 2) * 0.0343) + 6;

    duration_R1 = sonar_R1.ping();
    dis_R1 = ((duration_R1 / 2) * 0.0343) + 6;

    duration_F = sonar_F.ping();
    dis_F = (duration_F / 2) * 0.0343;
    delay(10);
    if (dis_L1 > 85) {
      Position = pos - turn_pos2;
      myservo.write(Position);
      delay(800);
      while (dis_F < 150) {
        myservo.write(Position);
        duration_F = sonar_F.ping();
        dis_F = (duration_F / 2) * 0.0343;
        delay(10);
        counter = 1;
      }
    }
    if (dis_R1 > 85) {
      Position = pos + turn_pos;
      myservo.write(Position);
      delay(800);
      while (dis_F < 150) {
        myservo.write(Position);
        duration_F = sonar_F.ping();
        dis_F = (duration_F / 2) * 0.0343;
        delay(10);
        counter = 2;
      }
    }
    if (dis_L1 < 85 && dis_R1 < 85 && dis_F > 60) {
      if (Real_dis_L < 15)
      {
        P_distanceL(Real_dis_L);
      }

      if (Real_dis_R < 15)
      {
        P_distanceR(Real_dis_R);
      }

      if (Real_dis_L > 15 && Real_dis_R > 15)
      {
        PI_angle_Left(dis_L1, dis_L2);
      }
    }
  }


  if (counter == 2 && dis_F > 100) {
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm);
    if (Real_dis_L < 15)
    {
      P_distanceL(Real_dis_L);
    }

    if (Real_dis_R < 15)
    {
      P_distanceR(Real_dis_R);
    }

    if (Real_dis_L > 15 && Real_dis_R > 15)
    {
      PI_angle_Left(dis_L1, dis_L2);
    }
  }


  if (counter == 2 && dis_F < 100)
  {
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm_1);
    duration_R1 = sonar_R1.ping();
    dis_R1 = ((duration_R1 / 2) * 0.0343) + 6;

    duration_F = sonar_F.ping();
    dis_F = (duration_F / 2) * 0.0343;
    delay(10);
    if (dis_R1 > 85) {
      Position = pos + turn_pos;
      myservo.write(Position);
      delay(800);
      while (dis_F < 150) {
        myservo.write(Position);
        duration_F = sonar_F.ping();
        dis_F = (duration_F / 2) * 0.0343;
        delay(10);
      }
      counter_1 = counter_1 + 1;
    }
    if (dis_R1 < 85 && dis_F > 60) {
      if (Real_dis_L < 15)
      {
        P_distanceL(Real_dis_L);
      }

      if (Real_dis_R < 15)
      {
        P_distanceR(Real_dis_R);
      }

      if (Real_dis_L > 15 && Real_dis_R > 15)
      {
        PI_angle_Left(dis_L1, dis_L2);
      }
    }
  }

  if (counter == 1 && dis_F > 100) {
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm);

    if (Real_dis_L < 15)
    {
      P_distanceL(Real_dis_L);
    }

    if (Real_dis_R < 15)
    {
      P_distanceR(Real_dis_R);
    }

    if (Real_dis_L > 15 && Real_dis_R > 15)
    {
      PI_angle_Right(dis_R1, dis_R2);
    }
  }
  if (counter == 1 && dis_F < 100)
  {
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm_1);
    duration_L1 = sonar_L1.ping();
    dis_L1 = ((duration_L1 / 2) * 0.0343) + 6;

    duration_F = sonar_F.ping();
    dis_F = (duration_F / 2) * 0.0343;
    delay(10);
    if (dis_L1 > 85) {
      Position = pos - turn_pos2;
      myservo.write(Position);
      delay(800);
      while (dis_F < 150) {
        myservo.write(Position);
        duration_F = sonar_F.ping();
        dis_F = (duration_F / 2) * 0.0343;
        delay(10);
      }
       counter_1 = counter_1 + 1;
    }
    if (dis_L1 < 85 && dis_F > 60) {
      if (Real_dis_L < 15)
      {
        P_distanceL(Real_dis_L);
      }

      if (Real_dis_R < 15)
      {
        P_distanceR(Real_dis_R);
      }

      if (Real_dis_L > 15 && Real_dis_R > 15)
      {
        PI_angle_Right(dis_R1, dis_R2);
      }
    }
  }}
if (counter_1 == 11 && counter == 1){
if (dis_F > 135){
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm);
  PI_angle_Right(dis_R1, dis_R2);
}
if (dis_F < 135){
  digitalWrite (sumo_p , LOW);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm_1);
}
  
}
if (counter_1 == 11 && counter == 2){
if (dis_F > 135){
    digitalWrite (sumo_p , HIGH);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm);
  PI_angle_Left(dis_L1, dis_L2);
}
if (dis_F < 135){
  digitalWrite (sumo_p , LOW);
    digitalWrite (sumo_n , LOW);
    analogWrite (sumo, pwm_1);
}
  
}



}
