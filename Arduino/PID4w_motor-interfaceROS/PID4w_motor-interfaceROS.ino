#include "PinZone.h"

#include <IntervalTimer.h>
#include <Encoder.h>
#include <DFRobot_WT61PC.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>



float yaw, last_yaw;
// ---------------- ROS Zone ----------------
ros::NodeHandle nh;

// Odom
geometry_msgs::Twist msg_motor;
geometry_msgs::Twist msg_imu;

ros::Publisher ard_encoder("ard_encoder", &msg_motor);
ros::Publisher ard_yaw("ard_imu", &msg_imu);

// msg general
std_msgs::String msg_gen;
ros::Publisher ard_msg("ard_msg", &msg_gen);


// ---------------- ROS Zone ----------------

/*-------------------------- Robot information --------------------------*/

float L = 0.265;  // L length from 2 wheel (m)
float R = 0.066;  // R length of wheel (m)
double vel_wL = 0;
double vel_wR = 0;

double kp_L = 1, ki_L = 0, kd_L = 0;  //setpoint 56 v = 0.3
double kp_R = 1, ki_R = 0, kd_R = 0;  //setpoint 56 v = 0.3
double sumE_FL, sumE_FR = 0;
double preE_FL, preE_FR = 0;
double vpFL, vpFR;  //ความเร็วที่วัดได้
double rpmFL = 0;
double rpmFR = 0;
double eFL, eFR;
int speedFL, speedFR;

long pulseFL_now = 0;
long pulseFR_now = 0;

long pulseFL_old = 0;
long pulseFR_old = 0;

long pulseFL_persec = 0;
long pulseFR_persec = 0;

double radFL_persec = 0;
double radFR_persec = 0;
double wL, wR;
double rpmFL_Kine = 0;
double rpmFR_Kine = 0;

/*-------------------------- Robot information --------------------------*/
String test_msg = "";

void roverCallBack(const geometry_msgs::Twist& cmd_vel) {
  double linearX = cmd_vel.linear.x;
  double AngularZ = cmd_vel.angular.z;

  wL = (linearX - (AngularZ * (L / 2))) / R;
  wR = (linearX + (AngularZ * (L / 2))) / R;
  // RPM = (rad/s) * (60/2π).
  rpmFL_Kine = wL * (60.0 / (2 * PI));
  rpmFR_Kine = wR * (60.0 / (2 * PI));
  vel_wL = rpmFL_Kine;
  vel_wR = rpmFR_Kine;
  
}

ros::Subscriber<geometry_msgs::Twist> Motorsub("/cmd_vel", roverCallBack);


IntervalTimer myTimer;

DFRobot_WT61PC sensor(&Serial5);

Encoder encL(E1_A, E1_B);
Encoder encR(E2_A, E2_B);

void messageCb(const std_msgs::Empty& toggle_msg) {
  encL.readAndReset();
  encR.readAndReset();
}

ros::Subscriber<std_msgs::Empty> Clearsub("ard_clearEncoder", &messageCb);

void setup() {
  // Serial.begin(9600);
  Serial5.begin(9600);
  setupPinOUTPUT();
  setupPinINPUT();
  sensor.modifyFrequency(FREQUENCY_10HZ);
  myTimer.begin(interrupt_calPulse, 100000);

  nh.initNode();
  nh.advertise(ard_encoder);
  nh.advertise(ard_yaw);
  nh.advertise(ard_msg);
  nh.subscribe(Motorsub);
  nh.subscribe(Clearsub);
}



unsigned long timer = 0;

void loop() {
  main_ros();
  controlMotorPID(-vel_wL, vel_wR);
  // controlMotorPID(-100,100);
}


void main_ros() {
  // ------------------ IMU yaw axis {yaw , vyaw} ------------------
  yaw = deg2rad(Get_IMU());
  float vyaw = yaw - last_yaw;
  msg_imu.linear.x = yaw;
  msg_imu.linear.y = vyaw;
  ard_yaw.publish(&msg_imu);

  last_yaw = yaw;


  // --------------------- MOTOR ----------------------
  msg_motor.linear.x = encL.read();
  msg_motor.linear.y = encR.read();

  ard_encoder.publish(&msg_motor);


  // --------------------- MSG --------------------------
  //  test_msg/

  char format_data[50];
  String sw_param = "rpmFL_Kine : " + String(rpmFL_Kine) + " rpmFR_Kine : " + String(rpmFR_Kine);
  // test_msg.toCharArray(format_data, 100);
  sw_param.toCharArray(format_data, 20);
  msg_gen.data = format_data;
  ard_msg.publish(&msg_gen);

  // ---------------------


  nh.spinOnce();
}

void controlMotor(double vFL, double vFR) {
  motorL(vFL);
  motorR(vFR);
}

void controlMotorPID(double sFL, double sFR) {
  /*
    PID RPM of wheel
  */

  vpFL = rpmFL;
  vpFR = rpmFR;


  eFL = sFL - vpFL;
  eFR = sFR - vpFR;


  speedFL = (kp_L * eFL) + (ki_L * sumE_FL) + (kd_L * (eFL - preE_FL));
  speedFR = (kp_R * eFR) + (ki_R * sumE_FR) + (kd_R * (eFR - preE_FR));



  if (speedFL > 255) speedFL = 255;
  if (speedFL < -255) speedFL = -255;

  if (speedFR > 255) speedFR = 255;
  if (speedFR < -255) speedFR = -255;


  if (sFL == 0) speedFL = 0;
  if (sFR == 0) speedFR = 0;

  controlMotor(speedFL, speedFR);
  //Serial.println(String(speedFL)+" "+String(speedFR));
  preE_FL = eFL;
  preE_FR = eFR;


  if (preE_FL == 0) {
    sumE_FL = 0;
  } else {
    sumE_FL += eFL;
  }

  if (preE_FR == 0) {
    sumE_FR = 0;
  } else {
    sumE_FR += eFR;
  }
}
void interrupt_calPulse() {
  pulseFL_now = encL.read() - pulseFL_old;
  pulseFR_now = encR.read() - pulseFR_old;


  pulseFL_persec = pulseFL_now * 10.0;
  pulseFR_persec = pulseFR_now * 10.0;


  radFL_persec = (pulseFL_persec * 6.283) / 2464.0;
  radFR_persec = (pulseFR_persec * 6.283) / 2464.0;


  rpmFL = radFL_persec * (60.0 / (2 * PI));
  rpmFR = radFR_persec * (60.0 / (2 * PI));

  pulseFL_old = encL.read();
  pulseFR_old = encR.read();
}

float Get_IMU() {
  float my_yaw = 0;
  if (sensor.available()) {
    my_yaw = sensor.Angle.Z;
  }
  return my_yaw;
}

void printlnSensor() {
  if (sensor.available()) {
    Serial.print("Acc\t");
    Serial.print(sensor.Acc.X);
    Serial.print("\t");
    Serial.print(sensor.Acc.Y);
    Serial.print("\t");
    Serial.println(sensor.Acc.Z);  //acceleration information of X,Y,Z
    Serial.print("Gyro\t");
    Serial.print(sensor.Gyro.X);
    Serial.print("\t");
    Serial.print(sensor.Gyro.Y);
    Serial.print("\t");
    Serial.println(sensor.Gyro.Z);  //angular velocity information of X,Y,Z
    Serial.print("Angle\t");
    Serial.print(sensor.Angle.X);
    Serial.print("\t");
    Serial.print(sensor.Angle.Y);
    Serial.print("\t");
    Serial.println(sensor.Angle.Z);  //angle information of X, Y, Z
    Serial.println(" ");
  } else {
    Serial.println("ERR");
  }
}
void printlnEncoder() {
  Serial.print("EN FL : ");
  Serial.print(encL.read());
  Serial.print("  , EN FR : ");
  Serial.print(encR.read());
  Serial.println();
}

// -------------------------- Math Zone --------------------------

float rad2deg(float rad) {
  float deg = rad * (180 / PI);
  return deg;
}
float deg2rad(float deg) {
  float rad = deg * (PI / 180);
  return rad;
}
