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
//#define SERVOMAX 600
//#define SERVOMIN 150
#define PS4_THRESH 15

//const byte interruptPin = 16;
const byte channelAmount = 10;
//PPMReader ppm(interruptPin, channelAmount);
int state = 0;
unsigned channelValues[10] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 15000, 1500, 1500};
const int maxSpd = 255;
const int minSpd = 0;
//bool lStickNeutral = true;
//bool rStickNeutral = true;
int signals[7] = {0,0,0,0,0,0,0}; //[Pivot Turn, Forward/Backward, Rock Claw, Left/Right, Lift, Intake, Flag Claw]
int mecWhlCalcHolder[4] = {0,0,0,0};
int speeds[7] = {0,0,0,0,0,0,0}; //[FM_R, FM_L, RM_R, RM_L, LIFT, INTAKE, ROCK_CLAW]
float rock_claw_pos = 0.0; //0.0 - 1.0
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
  mecWhlCalcHolder[0] = signals[1]-signals[3]-signals[0];
  mecWhlCalcHolder[1] = signals[1]+signals[3]+signals[0];
  mecWhlCalcHolder[2] = signals[1]+signals[3]-signals[0];
  mecWhlCalcHolder[3] = signals[1]-signals[3]+signals[0];
  
  //Right Front Wheel
  if(mecWhlCalcHolder[0] > 0){
      motorStates &= B10111111;
//      Serial.println("Problem Script Working");
  }else if(mecWhlCalcHolder[0] < 0){
      motorStates &= B01111111;
  }else{
      motorStates &= B00111111;
//      Serial.println("Channel 1 zeroed");
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

int addSpeeds(int A, int B){
  //**PURE ADDITIVE IMPLEMENTATION**
  return min(A+B, 255);
  //**END**//
  //**SUBTRACTIVE IMPLEMENTATION**
  // return A+B
  //**END**//
}



//void forwardBackward(){ 
//
//  if(signals[1] != 0){
//    if(signals[1] < 0){
//      speeds[1] = abs(signals[1]);
//      speeds[3] = abs(signals[1]);
//      speeds[5] = abs(signals[1]);
//      speeds[7] = abs(signals[1]);
//      
//     //**SUBTRACTIVE IMPLEMENTATION**//
////      speeds[0] = -1*abs(signals[1]);
////      speeds[2] = -1*abs(signals[1]);
////      speeds[4] = -1*abs(signals[1]);
////      speeds[6] = -1*abs(signals[1]);
//     //**END**//
//    }else{
//      speeds[0] = abs(signals[1]);
//      speeds[2] = abs(signals[1]);
//      speeds[4] = abs(signals[1]);
//      speeds[6] = abs(signals[1]);
//
//      //**SUBTRACTIVE IMPLEMENTATION**//
////      speeds[1] = -1*abs(signals[1]);
////      speeds[3] = -1*abs(signals[1]);
////      speeds[5] = -1*abs(signals[1]);
////      speeds[7] = -1*abs(signals[1]);
//      //**END**//
//    }
//  }
//}

//void strafe(){
//  if(signals[3] != 0){
//    if(signals[3] < 0){
//      speeds[0] = addSpeeds(abs(signals[3]), speeds[0]);
//      speeds[3] = addSpeeds(abs(signals[3]), speeds[3]);
//      speeds[5] = addSpeeds(abs(signals[3]), speeds[5]);
//      speeds[6] = addSpeeds(abs(signals[3]), speeds[6]);
//
//      //**SUBTRACTIVE IMPLEMENTATION**//
////      speeds[1] = addSpeeds(-1*abs(signals[3]), speeds[1]);
////      speeds[2] = addSpeeds(-1*abs(signals[3]), speeds[2]);
////      speeds[4] = addSpeeds(-1*abs(signals[3]), speeds[4]);
////      speeds[7] = addSpeeds(-1*abs(signals[3]), speeds[7]);
//     //**END**//
//    }else{
//      speeds[1] = addSpeeds(abs(signals[3]), speeds[1]);
//      speeds[2] = addSpeeds(abs(signals[3]), speeds[2]);
//      speeds[4] = addSpeeds(abs(signals[3]), speeds[4]);
//      speeds[7] = addSpeeds(abs(signals[3]), speeds[7]);
//
//      //**SUBTRACTIVE IMPLEMENTATION**//
////      speeds[0] = addSpeeds(-1*abs(signals[3]), speeds[0]);
////      speeds[3] = addSpeeds(-1*abs(signals[3]), speeds[3]);
////      speeds[5] = addSpeeds(-1*abs(signals[3]), speeds[5]);
////      speeds[6] = addSpeeds(-1*abs(signals[3]), speeds[6]);
//     //**END**//
//    }
//  }
//}

//void rotate(){
//  if(signals[0] != 0){
//    if(signals[0] < 0){
//      speeds[0] = addSpeeds(abs(signals[0]), speeds[0]);
//      speeds[3] = addSpeeds(abs(signals[0]), speeds[3]);
//      speeds[4] = addSpeeds(abs(signals[0]), speeds[5]);
//      speeds[7] = addSpeeds(abs(signals[0]), speeds[6]);
//
//      //**SUBTRACTIVE IMPLEMENTATION**//
////      speeds[1] = addSpeeds(-1*abs(signals[0]), speeds[1]);
////      speeds[2] = addSpeeds(-1*abs(signals[0]), speeds[2]);
////      speeds[5] = addSpeeds(-1*abs(signals[0]), speeds[4]);
////      speeds[6] = addSpeeds(-1*abs(signals[0]), speeds[7]);
//      //**END**//
//    }else{
//      speeds[1] = addSpeeds(abs(signals[0]), speeds[1]);
//      speeds[2] = addSpeeds(abs(signals[0]), speeds[2]);
//      speeds[5] = addSpeeds(abs(signals[0]), speeds[4]);
//      speeds[6] = addSpeeds(abs(signals[0]), speeds[7]);
//
//      //**SUBTRACTIVE IMPLEMENTATION**//
////      speeds[0] = addSpeeds(-1*abs(signals[0]), speeds[0]);
////      speeds[3] = addSpeeds(-1*abs(signals[0]), speeds[3]);
////      speeds[4] = addSpeeds(-1*abs(signals[0]), speeds[5]);
////      speeds[7] = addSpeeds(-1*abs(signals[0]), speeds[6]);
//      //**END**//
//    }
//  }
//}

void lift(){
  //move lift
  if(speeds[4]>0){
//    analogWrite(LIFT_1, speeds[4]);  
//    analogWrite(LIFT_2, 0);
    pwm.setPWM(LIFT_1, 0, speeds[4]);
    pwm.setPWM(LIFT_2, 0, 0);
  }else if(speeds[4]<0){
//    analogWrite(LIFT_1, 0);
//    analogWrite(LIFT_2, -speeds[4]);
    pwm.setPWM(LIFT_1, 0, 0);
    pwm.setPWM(LIFT_2, 0, -speeds[4]);
  }else{
//    digitalWrite(LIFT_1,1);
//    digitalWrite(LIFT_2,1);
    pwm.setPWM(LIFT_1, 4096, 0);
    pwm.setPWM(LIFT_2, 4096, 0);
  }
}
//
void intake(){
  //move intake
  if(speeds[5]>0){
    digitalWrite(INTAKE_1, HIGH);
    digitalWrite(INTAKE_2, LOW);
//    analogWrite(INTAKE_PWM, speeds[5]);
    pwm.setPWM(INTAKE_PWM, 0, speeds[5]);
  }else{
    digitalWrite(INTAKE_1, LOW);
    digitalWrite(INTAKE_2, HIGH);
//    analogWrite(INTAKE_PWM, -speeds[5]);
    pwm.setPWM(INTAKE_PWM, 0, -speeds[5]);
  }
}

void runServo(int servonum, float pos){

  //speed from 0 to 255

  uint16_t val = pos*(SERVOMAX[servonum]-SERVOMIN[servonum])+SERVOMIN[servonum];

//  Serial.println(String(servonum)+" Servo pos: "+String(val));

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

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  // pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
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
 
//  pinMode(LIFT_1, OUTPUT);
//  pinMode(LIFT_2, OUTPUT);

  pinMode(INTAKE_1, OUTPUT);
  pinMode(INTAKE_2, OUTPUT);
//  pinMode(INTAKE_PWM, OUTPUT);

  Serial.begin(115200);

  //Fly Sky Controller
  IBus.begin(Serial2,1); 

  //Servo stuff
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //PS4 Controller
  PS4.begin();

  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
//  for(byte channel = 1; channel <= channelAmount; ++channel){
//    unsigned bufferValue = channelValues[channel-1];
//    channelValues[channel-1] = ppm.latestValidChannelValue(channel, bufferValue);
//  }

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
    
    for(int channel = 0;channel < channelAmount; ++channel){
      channelValues[channel] = IBus.readChannel(channel);
      if(channelValues[channel] > 2000 || channelValues[channel] < 1000){
        channelValues[channel] = 1500;
      }
    }

    //Speed Reset
    for(int i = 0; i < sizeof(speeds)/sizeof(speeds[0]); i++){
      speeds[i] = 0;
    }
    motorStates = B11111111;
//    liftIntakeStates = B00001111;
  /*
    Channel 1: R Stick X-axis (Left = Low, Right = High)
    Channel 2: L Stick Y-axis (Down = Low, Up = High)
    Channel 3: R Stick Y-axis (Down = Low, Up = High)
    Channel 4: L Stick X-axis (Left = Low, Right = High)
    Channel 5: L Bumper Dial (Right = Low, Left = High)
    Channel 6: Back Buttons L AND R
    Channel 7: R Middle Switch (Up = Low, Down = High)
    Channel 8: L Middle Switch (Up = Low, Down = High)
    Channel 9: None
    Channel 10: None
    */
  // lStickNeutral = (channelValues[3] < NEUTRAL_PPM + TOLERANCE) && (channelValues[3] > NEUTRAL_PPM - TOLERANCE) && (channelValues[1] < NEUTRAL_PPM + TOLERANCE) && (channelValues[1] > NEUTRAL_PPM - TOLERANCE);
  // rStickNeutral = (channelValues[0] < NEUTRAL_PPM + TOLERANCE) && (channelValues[0] > NEUTRAL_PPM - TOLERANCE);

  //Movements
  for(byte key = 0; key < 4; ++key){
    if(abs((int)channelValues[key]-NEUTRAL_PPM)>=TOLERANCE){
      signals[key] = round(((float)((int)channelValues[key]-NEUTRAL_PPM)/(HIGH_PPM-NEUTRAL_PPM))*255); 
    }else{
      signals[key] = 0;
    }   
  }

  //Lift
  signals[4] = round(((float)((int)channelValues[5]-NEUTRAL_PPM)/(HIGH_PPM-NEUTRAL_PPM))*255);

  //Big Claw
  signals[2] = round(((float)((int)channelValues[4]-NEUTRAL_PPM)/(HIGH_PPM-NEUTRAL_PPM))*255);

  if (PS4.isConnected()) {
    if(abs(PS4.LStickY())>PS4_THRESH){
      signals[4] = (int)(PS4.LStickY()/128.0*255);
    }else{
      signals[4] = 0;
    }
    if(abs(PS4.RStickY())>PS4_THRESH){
      signals[2] = (int)(PS4.RStickY()/128.0*255);
    }else{
      signals[2] = 0;
    }
    signals[5] = PS4.R2Value()-PS4.L2Value();
    if(PS4.R1()){
      signals[6] = 255;
    }

    if(PS4.L1()){
      signals[6] = 0;
    }
  }else{
    //Intake Motor
    signals[5] = (int)(channelValues[6]>NEUTRAL_PPM)*255-(int)(channelValues[7]>NEUTRAL_PPM)*255;
  
    //Flag Servo
    signals[6] = (int)(channelValues[8]>NEUTRAL_PPM)*255;
  }

  //Big Rock Claw Servo (Position)
  rock_claw_pos += signals[2]/255.0*0.05;
  Serial.println("rock claw pos: "+String(rock_claw_pos));
  if(rock_claw_pos>1){
    rock_claw_pos = 1.0;
  }else if(rock_claw_pos<0){
    rock_claw_pos = 0.0;
  }

  //Lift
  speeds[4] = (int)(signals[4]/255.0*4095);

  //Intake
  speeds[5] = (int)(signals[5]/255.0*4095);

  //REVERSE KEY, FLIP R Middle switch towards operator to reverse the "front" of robot **UNCOMMENT TO IMPLEMENT**
//    if(channelValues[7] > HIGH_PPM-30){
//      for(byte key = 0; key < 4; ++key){
//        signals[key] = -1*signals[key];
//      }
//    }
  //Principle: So long as both control voltages are equal or close to equal there should be a braking effect in the motor, so should just add the voltages tgt up to 255 (I think?). 
  //Alternate implementation would be to subtract the analog value on the respective "0"s in addition to adding on the "1"s and set any -ve values to 0 before the final motor run step
  analogMixing();
//  forwardBackward();
//  strafe();
//  rotate();
  lift();
  intake();
//  motorLogicCheck();
  if(motorStates == B11111111){
    motorStates = B00000000;
  }
  runMotors();

  //Big Rock Claw Servo
  runServo(ROCK_CLAW1, rock_claw_pos);
//  runServo(ROCK_CLAW2,1.0-rock_claw_pos);

  //Flag Claw Servo, 0 or 1 (open or close)
//  runServo(FLAG_CLAW, signals[6]/255);

//  uint16_t val = 400;
//  
//  if(signals[6]==255){
//    val = 600;
//  }

//  pwm.setPWM(FLAG_CLAW,0,signals[6]/255);

  
  
  Serial.println(String(signals[0])+"\t"+String(signals[1])+"\t"+String(signals[2])+"\t"+String(signals[3])+"\t"+String(signals[4])+"\t"+String(signals[5])+"\t"+String(signals[6]));
  delay(10);
}
