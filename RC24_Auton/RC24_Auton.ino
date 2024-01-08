#include <IBusBM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS4Controller.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define FM_L1 4 //2
#define FM_L2 19 //15
#define FM_LPWM 18

#define FM_R1 32 //19
#define FM_R2 33 //4
#define FM_RPWM 14

#define RM_L1 15
#define RM_L2 27 
#define RM_LPWM 5 // 5

#define RM_R1 26
#define RM_R2 25
#define RM_RPWM 13
  
#define LIFT_1 14 
#define LIFT_2 15

#define INTAKE_1 2 //BOOTSTRAP PIN
#define INTAKE_2 12 //BOOTSTRAP PIN
#define INTAKE_PWM 13

#define FLAG_CLAW 7
#define ROCK_CLAW1 1
#define ROCK_CLAW2 2

#define SERVO_SPD_MPLIER (1/40)

#define TOLERANCE 30
#define NEUTRAL_PPM 1500
#define HIGH_PPM 2000
#define LOW_PPM 1000
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PS4_THRESH 15
#define CLAW_SPEED 100

const byte channelAmount = 10;

int state = 0;
unsigned channelValues[10] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 15000, 1500, 1500};
const int maxSpd = 255;
const int minSpd = 0;
int signals[7] = {0,0,0,0,0,0,0}; //[Pivot Turn, Forward/Backward, Rock Claw, Left/Right, Lift, Intake, Flag Claw]
int mecWhlCalcHolder[4] = {0,0,0,0};
int speeds[7] = {0,0,0,0,0,0,0}; //[FM_R, FM_L, RM_R, RM_L, LIFT, INTAKE, ROCK_CLAW]
double rock_claw_pos = 0.0; //0.0 - 1.0
// int dir[2] = {0,0};
int SERVOMIN[13] = {400,150,400,400,400,400,400,400,400,400,400,400,400};
int SERVOMAX[13] = {600,1000,600,600,600,600,600,600,600,600,600,600,600};
IBusBM IBus;

byte motorStates = B11111111;
byte liftIntakeStates = B00001111;

//Left Stick Y-Axis (Channel 2) Controls Forward/Backward Movement
//Left Stick X-Axis (Channel 4) Controls L/R Strafing
//Right Stick X-Axis (Channel 1) Controls CW/CCW Rotation

void analogMixing(){

  motorStates = B11111111;

  mecWhlCalcHolder[0] = signals[1]-signals[3]-signals[0];
  mecWhlCalcHolder[1] = signals[1]+signals[3]+signals[0];
  mecWhlCalcHolder[2] = signals[1]+signals[3]-signals[0];
  mecWhlCalcHolder[3] = signals[1]-signals[3]+signals[0];
  
  //Right Front Wheel
  if(mecWhlCalcHolder[0] > 0){
      motorStates &= B10111111;
  }else if(mecWhlCalcHolder[0] < 0){
      motorStates &= B01111111;
  }else{
      motorStates &= B00111111;
  }
  speeds[0] = min(abs(mecWhlCalcHolder[0]),255);

  //Left Front Wheel
  if(mecWhlCalcHolder[1] > 0){
      motorStates &= B11101111;
  }else if(mecWhlCalcHolder[1] < 0){
      motorStates &= B11011111;
  }else{
      motorStates &= B11001111;
  }
  speeds[1] = min(abs(mecWhlCalcHolder[1]),255);

  //Right Back Wheel
  if(mecWhlCalcHolder[2] > 0){
      motorStates &= B11111011;
  }else if(mecWhlCalcHolder[2] < 0){
      motorStates &= B11110111;
  }else{
      motorStates &= B11110011;
  }
  speeds[2] = min(abs(mecWhlCalcHolder[2]),255);

  //Left Back Wheel
  if(mecWhlCalcHolder[3] > 0){
      motorStates &= B11111110;
  }else if(mecWhlCalcHolder[3] < 0){
      motorStates &= B11111101;
  }else{
      motorStates &= B11111100;
  }
  speeds[3] = min(abs(mecWhlCalcHolder[3]),255);

//  Serial.println("Speeds: "+String(speeds[0])+"\t"+String(speeds[1])+"\t"+String(speeds[2])+"\t"+String(speeds[3])+"\t");
  
}

void lift(){
  //move lift
  speeds[4] = (int)(signals[4]/255.0*4095);
  if(speeds[4]>0){
    pwm.setPin(LIFT_1, speeds[4]);
    pwm.setPin(LIFT_2, 0);
  }else if(speeds[4]<0){
    pwm.setPin(LIFT_1, 0);
    pwm.setPin(LIFT_2, -speeds[4]);
  }else{
    pwm.setPin(LIFT_1, 4095);
    pwm.setPin(LIFT_2, 4095);
  }
}

void intake(){
  //move intake
  speeds[5] = (int)(signals[5]/255.0*4095);
  Serial.println(String(speeds[5]));
  if(speeds[5]>0){
    digitalWrite(INTAKE_1, HIGH);
    digitalWrite(INTAKE_2, LOW);
    pwm.setPin(INTAKE_PWM, speeds[5]);
  }else{
    digitalWrite(INTAKE_1, LOW);
    digitalWrite(INTAKE_2, HIGH);
    pwm.setPin(INTAKE_PWM, -speeds[5]);
  }
}

void runServo(int servonum, double pos){

  //speed from 0 to 255

  uint16_t val = pos*(SERVOMAX[servonum]-SERVOMIN[servonum])+SERVOMIN[servonum];

  Serial.println(String(servonum)+" Servo pos: "+String(val));

  pwm.setPin(servonum, val);

}

void motorLogicCheck(){
  /*Tracking Output State Values*/
  Serial.print("Calculated Values (-255 to 255): ");
  for(int i = 0; i < 4; i++){
    Serial.print(mecWhlCalcHolder[i]);
    Serial.print("\t");
  }
  Serial.println();
  
  Serial.print("Base Motor Logic: ");
  for(int i = 7; i >= 0; i--){
    Serial.print(bitRead(motorStates, i));
  }
  Serial.println();

  Serial.print("Lift/Intake Motor Logic: ");
  for(int i = 3; i >= 0; i--){
    Serial.print(bitRead(liftIntakeStates, i));
  }
  Serial.println();
}

void runMotors(){

  digitalWrite(FM_R1, bitRead(motorStates, 7));
  digitalWrite(FM_R2, bitRead(motorStates, 6));
  analogWrite(FM_RPWM, speeds[0]);

  digitalWrite(FM_L1, bitRead(motorStates, 5));
  digitalWrite(FM_L2, bitRead(motorStates, 4));
  analogWrite(FM_LPWM, speeds[1]);

  digitalWrite(RM_R1, bitRead(motorStates, 3));
  digitalWrite(RM_R2, bitRead(motorStates, 2));
  analogWrite(RM_RPWM, speeds[2]);

  digitalWrite(RM_L1, bitRead(motorStates, 1));
  digitalWrite(RM_L2, bitRead(motorStates, 0));
  analogWrite(RM_LPWM, speeds[3]);
  
}

void stopMotors(){
  digitalWrite(FM_R1, HIGH);
  digitalWrite(FM_R2, HIGH);
  analogWrite(FM_RPWM, 0);

  digitalWrite(FM_L1, HIGH);
  digitalWrite(FM_L2, HIGH);
  analogWrite(FM_LPWM, 0);

  digitalWrite(RM_R1, HIGH);
  digitalWrite(RM_R2, HIGH);
  analogWrite(RM_RPWM, 0);

  digitalWrite(RM_L1, HIGH);
  digitalWrite(RM_L2, HIGH);
  analogWrite(RM_LPWM, 0);
}

void moveMotors(int speed_x, int speed_y, int speed_r, float duration){

  signals[0] = speed_r;
  signals[1] = speed_x;
  signals[3] = speed_y;
  analogMixing();
  runMotors();
  delay(duration*1000); //ms
  stopMotors();

}

void lift_auto(int speed, float duration){
  signals[4] = speed;
  lift();
  delay(duration*1000);
  signals[4] = 0;
  lift();
}

void intake_auto(int speed){
  signals[5] = speed;
  Serial.println(String(speed));
  intake();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(FM_L1, OUTPUT);
  pinMode(FM_L2, OUTPUT);
  pinMode(FM_R1, OUTPUT);
  pinMode(FM_R2, OUTPUT);
  pinMode(RM_L1, OUTPUT);
  pinMode(RM_L2, OUTPUT);
  pinMode(RM_R1, OUTPUT);
  pinMode(RM_R2, OUTPUT);

  pinMode(FM_RPWM, OUTPUT);
  pinMode(FM_LPWM, OUTPUT);
  pinMode(RM_RPWM, OUTPUT);
  pinMode(RM_LPWM, OUTPUT);

  pinMode(INTAKE_1, OUTPUT);
  pinMode(INTAKE_2, OUTPUT);

  Serial.begin(115200);

  //Fly Sky Controller
  // IBus.begin(Serial2,1); 

  //Servo stuff
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //PS4 Controller
  // PS4.begin();

  Serial.println("Starting Auton");

  delay(1000);

  auton();

}

void auton(){

  float TIME_MULTI = 1.0;

  int low_speed = 100;
  int medium_speed = 150;
  int top_speed = 200;

  int stop_time = 2000;

  //180cm in 3s
  //270 degrees in 3s

  

  Serial.println("Intake ON");
  intake_auto(-medium_speed);

  delay(stop_time);

  Serial.println("Reversing");
  moveMotors(-low_speed,0,0,1.3*TIME_MULTI);

  delay(stop_time);

  Serial.println("Rotate CCW");
  moveMotors(0,0,-low_speed,TIME_MULTI);

  delay(stop_time);

  Serial.println("Forward");
  moveMotors(low_speed,0,0,1*TIME_MULTI);

  delay(stop_time);

  Serial.println("Reversing");
  moveMotors(-low_speed,0,0,6*TIME_MULTI);

  delay(stop_time);

  Serial.println("Forward");
  moveMotors(low_speed,0,0,0.5*TIME_MULTI);

  delay(stop_time);

  Serial.println("Rotate CCW");
  moveMotors(0,0,-low_speed,TIME_MULTI);

  delay(stop_time);

  Serial.println("Reversing");
  moveMotors(-low_speed,0,0,1.5*TIME_MULTI);

  delay(stop_time);

  Serial.println("Depositing");
  intake_auto(medium_speed);

  delay(stop_time);
}

void loop() {

    //Fly Sky Controller
    
    //Channel 1: Pivot Turn
    //Channel 2: Forward Backward
    //Channel 3: EMPTY
    //Channel 4: Left Right
    //Channel 5: Rock Claw
    //Channel 6: Lift
    //Channel 7: Intake In
    //Channel 8: Intake Out 
    //Channel 9: Flag Claw
    //Channel 10: 

    //Speed ReseT

  //Lift
  // speeds[4] = (int)(signals[4]/255.0*4095);

  //Intake
  // speeds[5] = (int)(signals[5]/255.0*4095);

  //REVERSE KEY, FLIP R Middle switch towards operator to reverse the "front" of robot **UNCOMMENT TO IMPLEMENT**
//    if(channelValues[7] > HIGH_PPM-30){
//      for(byte key = 0; key < 4; ++key){
//        signals[key] = -1*signals[key];
//      }
//    }
  Serial.println("ALL DONE!");
  
  // Serial.println(String(signals[0])+"\t"+String(signals[1])+"\t"+String(signals[2])+"\t"+String(signals[3])+"\t"+String(signals[4])+"\t"+String(signals[5])+"\t"+String(signals[6]));
  delay(10);
}
