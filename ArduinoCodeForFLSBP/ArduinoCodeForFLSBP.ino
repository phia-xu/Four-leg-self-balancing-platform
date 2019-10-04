#include <Wire.h>
#include <JY901.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
/*
 * Test on Leonardo
 * Leonard -----   JY61
 * GND    <----->  GND
 * 5v     <----->  VCC
 * 0(RX)  <----->  TX
 *         -----   STM32            
 *        <----->  TX
 *        <----->  RX
 */
SoftwareSerial servoSerial(2,3);  //RX, TX

float pitch, roll;
String comdata = "";
int method_type = 0, command_verify = 0, degradation_failure = 0, time_flag = 1;
String   stuck_leg = "NO";
float stuck_angle = 0.0, stuck_time = 0.0;
unsigned long start_time, now_time, dtime;

int s1_ini = 1500, s2_ini = 1550, s5_ini = 1400, s6_ini = 1300;
int s3_ini = 1550, s4_ini = 1250, s7_ini = 1450, s8_ini = 1550;
int s1, s2, s3, s4, s5, s6, s7,s8;
int s_upp = 1800, s_low = 1200;

double Setpoint, Input_p, Output_p, Input_r, Output_r;
double kp = 0.04, ki = 0.005, kd = 0.001;
PID myPIDp(&Input_p, &Output_p, &Setpoint, kp, ki, kd, REVERSE);
PID myPIDr(&Input_r, &Output_r, &Setpoint, kp, ki, kd, REVERSE);

//获取姿态信息
void get_attitude(){
  float _pitch = (float)JY901.stcAngle.Angle[0]/32768*180;
  float _roll = (float)JY901.stcAngle.Angle[1]/32768*180;
  pitch = (_pitch + 90)/100;
  roll = (_roll + 90)/100;
  Serial.print(pitch,4);
  Serial.print(roll,4);
}
//获取上位机命令
void get_command(){
  comdata = "";
  while (Serial.available() > 0){
    comdata += char(Serial.read());
    delay(2);
  }
  if (comdata[0] == 'S' && comdata[-1] == 'E'){
    command_verify == 1;// 开始
  }
  else if(comdata[0] == 'P' && comdata[-1] == 'E'){
    command_verify == 2;// 暂停
  }
  else if(comdata[0] == 'R' && comdata[-1] == 'E'){
    command_verify == 3;// 复位
  }
}
//上位机命令字符串解算
void command_decomposite(){
  method_type = (int)comdata[1];
  stuck_leg = comdata[2:4];
  degradation_failure = comdata[-2];
  stuck_angle = (float)comdata[4:6]+0.1*(float)comdata[6:8];
  stuck_time = (unsigned long)comdata[8:9]+0.1*(unsigned long)comdata[9:10];  
}
//舵机动作位置限幅
void servo_limiting(){
  if (s1 > s_upp){ s1 = s_upp;}
  else if(s1 < s_low){  s1 = s_low;}
  if (s2 > s_upp){ s2 = s_upp;}
  else if(s2 < s_low){  s2 = s_low;}
  if (s3 > s_upp){ s3 = s_upp;}
  else if(s3 < s_low){  s3 = s_low;}
  if (s4 > s_upp){ s4 = s_upp;}
  else if(s4 < s_low){  s4 = s_low;}
  if (s5 > s_upp){ s5 = s_upp;}
  else if(s5 < s_low){  s5 = s_low;}
  if (s6 > s_upp){ s6 = s_upp;}
  else if(s6 < s_low){  s6 = s_low;}
  if (s7 > s_upp){ s7 = s_upp;}
  else if(s7 < s_low){  s7 = s_low;}
  if (s8 > s_upp){ s8 = s_upp;}
  else if(s8 < s_low){  s8 = s_low;}
}
//舵机指令发送
void servo_command_send(){
   servo_limiting();
   servoSerial.println("#001P"+String(s1)+"T1000!");   servoSerial.println("#002P"+String(s2)+"T1000!");  //s1 s2
    servoSerial.println("#005P"+String(s5)+"T1000!");   servoSerial.println("#006P"+String(s6)+"T1000!");  //s5 s6
    servoSerial.println("#003P"+String(s3)+"T1000!");   servoSerial.println("#004P"+String(s4)+"T1000!");  //s3 s4
    servoSerial.println("#007P"+String(s7)+"T1000!");   servoSerial.println("#008P"+String(s8)+"T1000!");  //s7 s8
   
}
//ladrc 
//int LADRC(double v){
//   
//}
//舵机控制
void control_servo(unsigned long stuck_time, String stuck_leg){
  int stuck_pwm;
  now_time = micros();
  dtime = now_time - start_time;
  s1 = s1 + Output_r;   s2 = s2 + Output_r;   s5 = s5 + Output_r;   s6 = s6 + Output_r;
  s3 = s3 + Output_p;   s4 = s4 + Output_p;   s7 = s7 + Output_p;   s8 = s8 + Output_p; 
  if (dtime > stuck_time){   
    stuck_pwm = int(stuck_angle/360*3000);
    if (stuck_leg == "P1"){
      s3 = s3_ini + stuck_pwm;    s4 = s4_ini + stuck_pwm;    // s3 s4
    }
    else if (stuck_leg == "P2"){
      
    }
    else if (stuck_leg == "R1"){
      
    }
    else if (stuck_leg == "R2"){
      
    }
  }  
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);   // Leonard --- PC
  Serial1.begin(115200);  // Leonard --- JY61  

  Setpoint = 0;
  myPIDp.SetOutputLimits(-200,200);
  myPIDp.SetMode(AUTOMATIC);
  myPIDp.SetSampleTime(100);
  myPIDr.SetOutputLimits(-200,200);
  myPIDr.SetMode(AUTOMATIC);
  myPIDr.SetSampleTime(100);

  s1 = s1_ini;  s2 = s2_ini;  s3 = s3_ini;  s4 = s4_ini;  
  s5 = s5_ini;  s6 = s6_ini;  s7 = s7_ini;  s8 = s8_ini;
}

void loop() {
  get_attitude();
  get_command();
  command_decomposite();
  
  if (command_verify == 1){
    if (time_flag == 1){
      start_time = micros();
      time_flag = 0;
    }  
    switch (method_type){
      case 0:
        Input_p = pitch;
        myPIDp.Compute();
        Input_r = roll;
        myPIDr.Compute();
        break;
      case 1:
        
//        ADRC;
        break;
      case 2:
//        MADRC
        break;
    }  
    control_servo(stuck_time, stuck_leg);    
    servo_command_send();
  }
  else if(command_verify == 2){
    servo_command_send();  
  }
  else if(command_verify == 3){
    s1 = s1_ini;  s2 = s2_ini;  s3 = s3_ini;  s4 = s4_ini;  
    s5 = s5_ini;  s6 = s6_ini;  s7 = s7_ini;  s8 = s8_ini;
    servo_command_send();    
  }
  delay(50);
}

// JY61串口中断
void serialEvent1()
{
  while (Serial1.available()){
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
}


