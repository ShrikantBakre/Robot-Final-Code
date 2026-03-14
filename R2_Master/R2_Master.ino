#include <CytronMotorDriver.h>
#include <ps5Controller.h>
#include <ESP32Servo.h>
#include "HardwareSerial.h"
#include "Arduino_NineAxesMotion.h"
#include <Wire.h>

TaskHandle_t Task1 ;
TaskHandle_t Task2 ;
CytronMD M4(PWM_DIR, 13, 12) ;
CytronMD M3(PWM_DIR, 14, 27) ;
CytronMD M1(PWM_DIR, 26, 25) ;
CytronMD M2(PWM_DIR, 32, 33) ;
Servo SL_Servo ;
Servo SR_Servo ;
HardwareSerial espSerial(0);
NineAxesMotion mySensor ; 

int front() ;
int back() ;
int left() ;
int right() ;
int  aclk() ;
int  clk() ;
int M_stop() ;
void allStop() ;
double pid(int) ;
int acc(int) ;
int d_acc(int) ;

#define rly 19
#define in1 35
#define in2 34

int start , mode ;
int chassiSpd = 60 , chassis_Curr_Spd ;
bool curr_shr , pre_shr ;
bool curr_L1 , pre_L1 ;
bool curr_R1 , pre_R1 ;
bool curr_opt , pre_opt ;
bool curr_tri , pre_tri ;
bool curr_cir , pre_cir ;
bool curr_sqr  , pre_sqr ;
int o_angle , p_angle , Chassis_setpoint ;
int bldc ;
int perr ;
int p , x , y ;
double Chassis_kp = 3 , Chassis_ki = 0 , Chassis_kd = 1 ;
unsigned long lastStreamTime = 0 ;
const int streamPeriod = 20 ;   
int acc_curr_time , acc_pre_time ;
int d_acc_curr_time , d_acc_pre_time ;
int stop_flag , curr_chassis_flag , pre_chassis_flag ;
int bldc_spd = 1200 ;
int val_1 , val_2 ;

void setup() {
  xTaskCreatePinnedToCore(MainCode, "Task1", 10000, NULL, 1, &Task1, 0) ;
  xTaskCreatePinnedToCore(Orientation, "Task2", 10000, NULL, 1, &Task2, 1) ;
  // Serial.begin(115200) ;
  espSerial.begin(115200) ;
  // ps5.begin("A0:FA:9C:0D:D5:BE");
  ps5.begin("7C:66:EF:3C:EF:D5");
  // ps5.begin("D0:BC:C1:A3:40:83");
  pinMode( rly , OUTPUT ) ;
  pinMode( in1 , INPUT ) ;
  pinMode( in2 , INPUT ) ;
  digitalWrite(rly , LOW) ;
	ESP32PWM::allocateTimer(3) ;
  SL_Servo.setPeriodHertz(50);
  SR_Servo.setPeriodHertz(50);
  SL_Servo.attach(2, 1000, 2000) ; 
  SR_Servo.attach(4, 1000, 2000) ; 
  SL_Servo.writeMicroseconds(1000) ;
  SR_Servo.writeMicroseconds(1000) ; 
  Wire.begin() ;
  mySensor.initSensor() ;
  mySensor.setOperationMode(OPERATION_MODE_NDOF) ;
  mySensor.setUpdateMode(MANUAL) ;
  delay(300) ;
}

void loop() {
    
}

void MainCode( void * parameters ) {
  while(1){
    if(ps5.isConnected()){
      Serial.print("Connected") ;
      Serial.print("\t") ;

      if(ps5.PSButton() ){
        allStop() ;
        abort() ;
      }
      else{
        
        curr_shr = ps5.Share() ;
        if(curr_shr != pre_shr && curr_shr > 0) ++start ;
        pre_shr = curr_shr ;

        if(start % 2 ==1){
          curr_opt = ps5.Options() ;
          if(curr_opt != pre_opt && curr_opt > 0) ++mode ;
          pre_opt = curr_opt ;

          if(mode % 2 == 0){
            digitalWrite( rly , HIGH ) ;
            curr_L1 = ps5.L1() ;
            curr_R1 = ps5.R1() ;
            if( curr_L1 != pre_L1 && curr_L1 > 0)      chassiSpd -= 5 ;
            else if( curr_R1 != pre_R1 && curr_R1 > 0) chassiSpd += 5 ;
            pre_L1 = curr_L1 ;
            pre_R1 = curr_R1 ;
            Serial.print(chassiSpd) ;
            Serial.print("\t") ;
            chassiSpd = constrain(chassiSpd , -150 , 150 ) ;
            Serial.print(chassis_Curr_Spd) ;
            Serial.print("\t") ;
            p = pid (Chassis_kp , Chassis_ki , Chassis_kd , p_angle , 0 ) ;
            x = constrain(chassis_Curr_Spd + p , 0 , 150 ) ;
            y = constrain(chassis_Curr_Spd - p , 0 , 150 ) ;
            
            Serial.print(x)    ;
            Serial.print("\t") ;
            Serial.print(y)    ;
            Serial.print("\t") ;

            if( ps5.L2Value()>100 || ps5.R2Value()>100 || ps5.Right() ||  ps5.Left() || ps5.Left() || ps5.Up() || ps5.Down() ){

              chassis_Curr_Spd = acc ( chassis_Curr_Spd , chassiSpd , 1 , 5  ) ;
              if(     ps5.L2Value()>100)  {aclk(map(ps5.L2Value() , 100 , 255 , 0 , 25)) ; Chassis_setpoint = o_angle ; curr_chassis_flag = 1 ; chassis_Curr_Spd = 0 ;}
              else if(ps5.R2Value()>100)  {clk (map(ps5.R2Value() , 100 , 255 , 0 , 25)) ; Chassis_setpoint = o_angle ; curr_chassis_flag = 2 ; chassis_Curr_Spd = 0 ;}
              else if( ps5.Right() )      {right(x,y) ; stop_flag = 1 ; curr_chassis_flag = 3 ; }
              else if( ps5.Left()  )      {left (x,y) ; stop_flag = 2 ; curr_chassis_flag = 4 ; }
              else if(   ps5.Up()  )      {front(x,y) ; stop_flag = 3 ; curr_chassis_flag = 5 ; }
              else if(  ps5.Down() )      {back (x,y) ; stop_flag = 4 ; curr_chassis_flag = 6 ; }
            }
            else { 
              // curr_chassis_flag = 0 ;
              Chassis_setpoint = o_angle ;
              chassis_Curr_Spd = d_acc ( chassis_Curr_Spd , 0 , 1 , 7  )  ;
              M_stop( chassis_Curr_Spd , stop_flag ) ;
            }

            if( pre_chassis_flag != curr_chassis_flag ) chassis_Curr_Spd = 0 ;

            pre_chassis_flag = curr_chassis_flag ;
            espSerial.println("a") ;
          }
          else{
            digitalWrite(rly , LOW) ;
            M_stop( chassis_Curr_Spd , 0 ) ;
            if(ps5.Touchpad())  espSerial.println("A") ;
            else                espSerial.println("a") ;
          }

          val_1 = digitalRead( in1 ) ;
          val_2 = digitalRead( in2 ) ;
          curr_tri = ps5.Triangle() ;
          curr_cir = ps5.Circle() ;
          if     (curr_tri == 1 && pre_tri == 0) bldc_spd += 5 ;
          else if(curr_cir == 1 && pre_cir == 0) bldc_spd -= 5 ;
          pre_tri = curr_tri ;
          pre_cir = curr_cir ;
          
          curr_sqr = ps5.Square() ;
          if(curr_sqr == 1 && pre_sqr == 0 ) ++bldc ;
          pre_sqr = curr_sqr ;
          
          if(bldc % 2 == 1){
            SL_Servo.writeMicroseconds(bldc_spd) ;
            SR_Servo.writeMicroseconds(bldc_spd) ; 
          }
          else{
            SL_Servo.writeMicroseconds(1000) ;
            SR_Servo.writeMicroseconds(1000) ; 
          }

          if( ps5.RStickY() > 100 && val_2 == 1)       espSerial.println("b") ;
          else if( ps5.RStickY() < -100 && val_1 == 1) espSerial.println("c") ;
          else                            espSerial.println("d") ; 
        }
        else{
          allStop() ;
        }
      }
    }
    else{
      allStop() ;
      Serial.print("Not_Connected") ;
    }
    Serial.print("\n") ;
    Serial.print(o_angle) ;
    Serial.print("\t") ;
    Serial.print(p_angle) ;
    Serial.print("\t") ;
    Serial.print(Chassis_setpoint) ;
    Serial.print("\t") ;
  }
}

void Orientation( void * parameters ){
  while(1){  
    if ((millis() - lastStreamTime) >= streamPeriod){ 
      mySensor.updateEuler() ;
      mySensor.updateCalibStatus() ;
      o_angle = mySensor.readEulerHeading() ;
      if      (o_angle == 360)                       o_angle = 0 ;
      else if (o_angle > 180)                        o_angle = o_angle - 360 ; 
      if      (o_angle - Chassis_setpoint < -180)    p_angle = o_angle - Chassis_setpoint + 360 ;
      else if (o_angle - Chassis_setpoint > 180 )    p_angle = o_angle - Chassis_setpoint - 360 ;
      else                                           p_angle = o_angle - Chassis_setpoint ;
    }
  }
}

int acc(int spd , int target , int step , int time ) {
  acc_curr_time = millis();
  if (acc_curr_time - acc_pre_time >= time && spd + step < target ){
    spd += step ;
    acc_pre_time = acc_curr_time ;

    if(spd + step >= target) spd = target ;
  }
  if(spd == target) acc_pre_time = acc_curr_time ;
  Serial.print(acc_curr_time) ;
  Serial.print("\t") ;
  Serial.print(acc_pre_time) ;
  Serial.print("\t") ;
  Serial.print(spd) ;
  Serial.print("\t") ;
  Serial.print(target) ;
  Serial.print("\t") ;
  Serial.print(step) ;
  Serial.print("\t") ;
  return spd ;
}

int d_acc( int spd , int target , int step , int time ) {
  d_acc_curr_time = millis();
  if (d_acc_curr_time - d_acc_pre_time >= time && spd - step > target ){
    spd -= step ;
    d_acc_pre_time = d_acc_curr_time ;

    if(spd - step <= target) {
      spd = target ;
      stop_flag = 0 ;
    }
  }
  if(spd == target) d_acc_pre_time = d_acc_curr_time ;
  Serial.print(d_acc_curr_time) ;
  Serial.print("\t") ;
  Serial.print(d_acc_pre_time) ;
  Serial.print("\t") ;
  Serial.print(spd) ;
  Serial.print("\t") ;
  Serial.print(target) ;
  Serial.print("\t") ;
  Serial.print(step) ;
  Serial.print("\t") ;
  return spd ;
}

int pid(double kp , double ki , double kd , int av , int sp){
  int err = av - sp ;
  int prop = err ;
  int integral = err + perr ;
  int deri = err-perr ;
  int rslt = prop*kp + integral*ki + deri*kd ;
  perr = err ;
  return(rslt) ;
}

void back(int a , int b){
  M1.setSpeed(b)      ;    M2.setSpeed(b)     ;
  M3.setSpeed(-a)       ;    M4.setSpeed(-a)      ;
  Serial.print("Back") ;   Serial.print("\t")  ;
}

void front(int a , int b){
  M1.setSpeed(-a)        ;   M2.setSpeed(-a)      ;
  M3.setSpeed(b)       ;   M4.setSpeed(b)     ;
  Serial.print("Front") ;    Serial.print("\t")  ;
}

void left(int a , int b){
  M1.setSpeed(-a)      ;    M2.setSpeed(b)      ;
  M3.setSpeed(b)      ;    M4.setSpeed(-a)      ;
  Serial.print("left") ;    Serial.print("\t")  ;
}

void right(int a , int b){
  M1.setSpeed(b)        ;   M2.setSpeed(-a)     ;
  M3.setSpeed(-a)        ;   M4.setSpeed(b)     ;
  Serial.print("right") ;   Serial.print("\t")  ;
}

void aclk(int x){
  M1.setSpeed(-x)       ;   M2.setSpeed(-x)     ;
  M3.setSpeed(-x)       ;   M4.setSpeed(-x)     ;
  Serial.print("clk")  ;   Serial.print("\t")  ;
}

void clk(int x){
  M1.setSpeed(x)        ;   M2.setSpeed(x)      ;
  M3.setSpeed(x)        ;   M4.setSpeed(x)      ;
  Serial.print("aclk")   ;   Serial.print("\t")  ;
}

void M_stop(int x , int flag ){
  switch(flag){
    case 0 :  
      M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(100)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(100) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(100)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(100) ;
      // delay(500) ;
      break ;
    case 1 :  
      M1.setSpeed(x)  ;     M2.setSpeed(-x) ;
      M3.setSpeed(-x) ;     M4.setSpeed(x)  ;
      break ;
      
    case 2 :  
      M1.setSpeed(-x) ;     M2.setSpeed(x)  ;
      M3.setSpeed(x)  ;     M4.setSpeed(-x) ;
      break ;
      
    case 3 :  
      M1.setSpeed(-x) ;     M2.setSpeed(-x) ;
      M3.setSpeed(x)  ;     M4.setSpeed(x)  ;
      break ;
      
    case 4 :  
      M1.setSpeed(x)   ;    M2.setSpeed(x)  ;
      M3.setSpeed(-x)  ;    M4.setSpeed(-x) ;
      break ;
      
    default :  
      M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(100)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(100) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(100)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(100) ;
      // delay(500) ;
      break ;
  }
  Serial.print("stop") ;    Serial.print("\t") ;
}

void allStop(){
  M_stop(chassis_Curr_Spd , stop_flag ) ;
  digitalWrite( rly , LOW ) ;
  espSerial.println("a") ;
  espSerial.println("d") ;
  SL_Servo.writeMicroseconds(1000) ;
  SR_Servo.writeMicroseconds(1000) ; 
}