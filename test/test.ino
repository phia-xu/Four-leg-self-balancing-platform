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
 * 9       <----->  TX
 * 10      <----->  RX
 */
SoftwareSerial servoSerial(9,10);  //RX, TX

float pitch, roll;
String comdata = "", servotime = "T1000!";
int method_type = 0, command_verify = 0, degradation_failure = 0, time_flag = 1;
String   stuck_leg = "NO";
float stuck_angle = 0.0;
unsigned long start_time, now_time, dtime,stuck_time ;

int s1_ini = 1500, s2_ini = 1550, s5_ini = 1400, s6_ini = 1500; //6-1300
int s3_ini = 1550, s4_ini = 1250, s7_ini = 1450, s8_ini = 1550; //8-1300
int s1, s2, s3, s4, s5, s6, s7,s8;
int s_upp = 1800, s_low = 1200;

double Setpoint, Input_p, Output_p, Input_r, Output_r;
double kp = 0.4, ki = 0.05, kd = 0.01;
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
  delay(50);
}
//获取上位机命令
void get_command(){
  String comdata = "";
  while (Serial.available() > 0){
    comdata += char(Serial.read());
    delay(2);
  }
  if (comdata[0] == 'S' && comdata[11] == 'E'){
    command_verify == 1;// 开始
  }
  else if(comdata[0] == 'P' && comdata[1] == 'E'){
    command_verify == 2;// 暂停
  }
  else if(comdata[0] == 'R' && comdata[1] == 'E'){
    command_verify == 3;// 复位
  }
}
//上位机命令字符串解算
void command_decomposite(){
  method_type = (int)comdata[1];
  degradation_failure = comdata[10];
  stuck_leg = comdata.substring(2,4);
  stuck_angle = 10*(comdata[4]-'0')+(comdata[5]-'0')+0.1*(comdata[6]-'0')+0.01*(comdata[7]-'0');
  stuck_time = (comdata[8]-'0')+0.1*(comdata[9]-'0');
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
  servoSerial.println("{G0000#001P"+String(s1)+servotime+
                        "#002P"+String(s2)+servotime+
                        "#003P"+String(s3)+servotime+
                        "#004P"+String(s4)+servotime+
                        "#005P"+String(s5)+servotime+
                        "#006P"+String(s6)+servotime+
                        "#007P"+String(s7)+servotime+
                        "#008P"+String(s8)+servotime+"}");
  delay(1500);

}
//ladr
//int LADRC(double v){
//
//}
//舵机控制
void control_servo(unsigned long stuck_time, String stuck_leg){
  s1 = s1 + int(Output_r);   s2 = s2 + int(Output_r);   s5 = s5 - int(Output_r);   s6 = s6 -int(Output_r);
  s3 = s3 + int(Output_p);   s4 = s4 +int(Output_p);   s7 = s7 - int(Output_p);   s8 = s8 - int(Output_p);
  if (stuck_leg != "NO"){
    int stuck_pwm;
    now_time = micros();
    dtime = now_time - start_time;
    if (dtime > stuck_time){
      stuck_pwm = int(stuck_angle/270*2000);
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
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);   // Leonard --- PC
  Serial1.begin(115200);  // Leonard --- JY61
  servoSerial.begin(115200); //Leonard --- STM32

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
//  servoSerial.println("#003P1400T1000!");
//  delay(1500);
//  servoSerial.println("#003P1600T1000!");
//  delay(1500);
//  get_attitude();
  get_command();
  Serial.println("++++++++");
  delay(1500);
  Serial.println(comdata);
  delay(3000);
  Serial.println("---");
  delay(1500);
  
//  if (command_verify == 1){
//    command_decomposite();
//    if (time_flag == 1){
//      start_time = micros();
//      time_flag = 0;
//    }
//    switch (method_type){
//      case 0:
//        Input_p = pitch;
//        myPIDp.Compute();
//        Input_r = roll;
//        myPIDr.Compute();
//        break;
////      case 1:
//////        ADRC;
////        break;
////      case 2:
//////        MADRC
////        break;
//    }
//    control_servo(stuck_time, stuck_leg);
//    servo_command_send();
//  }
//  else if(command_verify == 2){
//    servo_command_send();
//  }
//  else if(command_verify == 3){
//    s1 = s1_ini;  s2 = s2_ini;  s3 = s3_ini;  s4 = s4_ini;
//    s5 = s5_ini;  s6 = s6_ini;  s7 = s7_ini;  s8 = s8_ini;
//    servo_command_send();
//  }
//  delay(50);
}

// JY61串口中断
void serialEvent1()
{
  while (Serial1.available()){
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
}


