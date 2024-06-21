#include <Encoder.h>
#include <DFRobot_WT61PC.h>
//#include <IntervalTimer.h>
// MotorL
#define pinML_INA 22
#define pinML_INB 23

// MotorR
#define pinMR_INA 19
#define pinMR_INB 18

//sw
#define sw 23
//Ultrasonics
#define trigPin_F 17  //RX
#define echoPin_F 16  //TX

#define trigPin_B 15  //RX
#define echoPin_B 14  //TX
//Define variables
long duration_F;
int distance_F;

long duration_B;
int distance_B;
//Input
String state_test;
int value_test = 0;
int dt = 0;
int state_rotate = 0;
double zeta_rotate = 0;
//IMU
DFRobot_WT61PC sensor(10,11);
//Encoder
long positionLeft = -999;
long positionRight = -999;
Encoder knobLeft(8, 7);
Encoder knobRight(5, 6);

long pulseFL_now = 0;
long pulseFR_now = 0;

long pulseFL_old = 0;
long pulseFR_old = 0;

long pulseFL_persec = 0;
long pulseFR_persec = 0;

double radFL_persec = 0;
double radFR_persec = 0;

double Pulse_forwart = 0;
double Pulse_rotate = 0;

double vpFL, vpFR;  //ความเร็วที่วัดได้
double rpmFL = 0;
double rpmFR = 0;
double rpmFL_Kine = 0;
double rpmFR_Kine = 0;

double eFL, eFR;
int speedFL, speedFR;
// PID

// double kp_L = 0.55, ki_L = 0.000005, kd_L = 0;  //setpoint 40
// double kp_R = 1.2, ki_R = 0.00005, kd_R = 0.1;  //setpoint 40

double kp_L = 1.0, ki_L = 0.0001, kd_L = 0.1;  //setpoint 56 v = 0.3
double kp_R = 1.0, ki_R = 0.0001, kd_R = 0.1;  //setpoint 56 v = 0.3

// double kp_L = 0.11, ki_L = 0.000005, kd_L = 0.005;  //setpoint 18
// double kp_R = 0.11, ki_R = 0.000005, kd_R = 0.005;  //setpoint 18
// double kp_L = 0, ki_L = 0, kd_L = 0;
// double kp_R = 0, ki_R = 0, kd_R = 0;

double sumE_FL, sumE_FR = 0;
double preE_FL, preE_FR = 0;

//kinematics
float L = 0.26;   // L length from 2 wheel (m)
float R = 0.051;  // R length of wheel (m)
double linearX;
double zeta;
double wL, wR;
float rad;
float valdegree;
//input_test
double value_test2;
int time_delay = 0;

//solfstart
bool ch_rec = false;
double SL = 0, SR = 0;

//timer
IntervalTimer myTimer;



void setup() {
  Serial.begin(115200);
  myTimer.begin(interrupt_calPulse, 100000);
  Serial.println("ลุยเลยอีหนู:");
  //sw
  pinMode(sw, INPUT_PULLUP);
  //Motor
  pinMode(pinML_INA, OUTPUT);
  pinMode(pinML_INB, OUTPUT);


  pinMode(pinMR_INA, OUTPUT);
  pinMode(pinMR_INB, OUTPUT);
 
  //IMU
  Serial5.begin(9600);
  sensor.modifyFrequency(FREQUENCY_10HZ);
  //Ultrasonics
  pinMode(trigPin_F, OUTPUT);
  pinMode(echoPin_F, INPUT);

  pinMode(trigPin_B, OUTPUT);
  pinMode(echoPin_B, INPUT);
  //Begin Serial communication
}
//ccw- ,cw+
void loop() {
void Read_IMU();
}

void ReadEncoder() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
}

void motorL(int pwm) {
  //int output = map(pwm, 0, 100, 0, 255);
  if (pwm == 0) {
    digitalWrite(pinML_INA, 1);
    digitalWrite(pinML_INB, 1);
  } else if (pwm > 0) {
    digitalWrite(pinML_INA, 1);
    digitalWrite(pinML_INB, 0);
    //d = false;
  } else if (pwm < 0) {
    digitalWrite(pinML_INA, 0);
    digitalWrite(pinML_INB, 1);
    //d = true;
  }

  // if (SOFT_START() == true) {
  //   analogWrite(pinML_PWM, abs(SL));
  // } else {
  //   analogWrite(pinML_PWM, abs(pwm));
  // }
}
void motorR(int pwm) {
  //int output = map(pwm, 0, 100, 0, 255);
  if (pwm == 0) {
    digitalWrite(pinMR_INA, 1);
    digitalWrite(pinMR_INB, 1);
  } else if (pwm > 0) {
    digitalWrite(pinMR_INA, 0);
    digitalWrite(pinMR_INB, 1);
    //d = false;
  } else if (pwm < 0) {
    digitalWrite(pinMR_INA, 1);
    digitalWrite(pinMR_INB, 0);
    //d = true;
  }

  // if (SOFT_START() == true) {
  //   analogWrite(pinMR_PWM, abs(SR));
  // } else {
  //   analogWrite(pinMR_PWM, abs(pwm));
  // }
}

void spinL(int speedFW) {

  motorL(-speedFW);
  motorR(speedFW);
}
void spinR(int speedFW) {

  motorL(speedFW);
  motorR(-speedFW);
}
void BW(int speedFW) {

  motorL(-speedFW);
  motorR(-speedFW);
}
void stopBot() {

  motorL(0);
  motorR(0);
}
void controlMotorPID(double sFL, double sFR) {
  /*
    PID RPM of wheel
  */

  vpFL = -rpmFL;
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
void controlMotor(double vFL, double vFR) {
  motorL(vFL);
  motorR(vFR);
}
void interrupt_calPulse() {
  pulseFL_now = knobLeft.read() - pulseFL_old;
  pulseFR_now = knobRight.read() - pulseFR_old;


  pulseFL_persec = pulseFL_now * 10.0;
  pulseFR_persec = pulseFR_now * 10.0;


  radFL_persec = (pulseFL_persec * 6.283) / 249000.0;
  radFR_persec = (pulseFR_persec * 6.283) / 249000.0;


  rpmFL = radFL_persec * (60.0 / (2 * PI));
  rpmFR = radFR_persec * (60.0 / (2 * PI));




  pulseFL_old = knobLeft.read();
  pulseFR_old = knobRight.read();
}
void input_test() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    state_test = msg;
    state_test = msg.substring(0, 2);
    value_test2 = atof(msg.substring(2, msg.length()).c_str());
  }
  if (state_test == "va") {
    linearX = value_test2;

  } else if (state_test == "wa") {
    valdegree = value_test2;
    rad = (valdegree * PI) / 180;  //Tranfrom Degree to Rad
    zeta = rad;
  } else if (state_test == "pl") {
    kp_L = value_test2;

  } else if (state_test == "il") {
    ki_L = value_test2;

  } else if (state_test == "dl") {
    kd_L = value_test2;

  } else if (state_test == "pr") {
    kp_R = value_test2;

  } else if (state_test == "ir") {
    ki_R = value_test2;

  } else if (state_test == "dr") {
    kd_R = value_test2;

  } else if (state_test == "dt") {
    dt = value_test2;
  } else if (state_test == "rg") {

    controlMotor(value_test2, value_test2);
  } else if (state_test == "st") {
    state_rotate = value_test2;
  } else if (state_test == "zt") {
    zeta_rotate = value_test2;
  } else if (state_test == "sl") {
    speedFL = value_test2;
  } else if (state_test == "sr") {
    speedFR = value_test2;
  } else if (state_test == "ok") {
    //Inverse_Kinematics();
    controlMotorPID(56, 56);
    //position_forward(dt, linearX, zeta);
    //position_rotate(state_rotate, zeta_rotate, linearX, zeta);
  } else if (state_test == "rs") {
    linearX = 0;
    zeta = 0;
    vpFL = 0;
    vpFR = 0;
    speedFL = 0;
    speedFR = 0;
    rpmFL_Kine = 0;
    rpmFR_Kine = 0;

    controlMotorPID(0, 0);
  }
}



void Inverse_Kinematics() {

  wL = (linearX - (zeta * (L / 2))) / R;
  wR = (linearX + (zeta * (L / 2))) / R;
  // RPM = (rad/s) * (60/2π).
  rpmFL_Kine = wL * (60.0 / (2 * PI));
  rpmFR_Kine = wR * (60.0 / (2 * PI));

  controlMotorPID(rpmFL_Kine, rpmFR_Kine);

  // wL_factor = wL * factor;
  // wR_factor = wR * factor;
}
void Read_Ultrasonics() {
  digitalWrite(trigPin_F, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_F, LOW);

  duration_F = pulseIn(echoPin_F, HIGH);

  distance_F = duration_F * 0.034 / 2;

  digitalWrite(trigPin_B, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin_B, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_B, LOW);

  duration_B = pulseIn(echoPin_B, HIGH);

  distance_B = duration_B * 0.034 / 2;


  Serial.println(String(distance_F) + " " + String(distance_B));
}
void Read_IMU() {
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
    Serial.println("wait for connect...");
  }
}
void position_forward(double distance, double valX, double valZ) {
  // linearX = valX;
  // valdegree = valZ;
  // rad = (valdegree * PI) / 180;  //Tranfrom Degree to Rad
  // zeta = rad;
  //Pulse_for_run = distance * 780099.6272;  //m

  Pulse_forwart = distance * 7800.996272;  //cm

  if (abs(double(pulseFL_old)) < Pulse_forwart && double(pulseFR_old) < Pulse_forwart) {
    Inverse_Kinematics();
  } else {
    linearX = 0;
    zeta = 0;
    vpFL = 0;
    vpFR = 0;
    speedFL = 0;
    speedFR = 0;
    rpmFL_Kine = 0;
    rpmFR_Kine = 0;
    controlMotorPID(0, 0);
    // value_test = 1;
  }
}
void position_rotate(int state, double rotate, double valX, double valZ) {
  double n = (0.26 * rotate) / 36;
  Pulse_rotate = n * 249000;
  if (state == 0) {
    if (-(abs(double(pulseFL_old))) < double(Pulse_rotate) && pulseFL_old < Pulse_rotate) {
      Inverse_Kinematics();
    } else {
      linearX = 0;
      zeta = 0;
      vpFL = 0;
      vpFR = 0;
      speedFL = 0;
      speedFR = 0;
      rpmFL_Kine = 0;
      rpmFR_Kine = 0;
      controlMotorPID(0, 0);
      // value_test = 1;
    }
  }
  if (state == 1) {
    if (abs(double(pulseFL_old)) < -(double(Pulse_rotate)) && pulseFL_old < Pulse_rotate) {
      Inverse_Kinematics();
    } else {
      linearX = 0;
      zeta = 0;
      vpFL = 0;
      vpFR = 0;
      speedFL = 0;
      speedFR = 0;
      rpmFL_Kine = 0;
      rpmFR_Kine = 0;
      controlMotorPID(0, 0);
      // value_test = 1;
    }
  }
}

bool SOFT_START() {
  if (ch_rec == false) {
    if (SL < speedFL) {
      SL += speedFL * 0.000015;
    }
    if (SR < speedFR) {
      SR += speedFR * 0.000015;
    }
    if (SR >= speedFR && SL >= speedFL) {
      ch_rec = true;
    }
    Serial.println("SL : " + String(SL) + " ; SR : " + String(SR));
  }
  return ch_rec == false;
}
