#include <ps5Controller.h>
#include <CytronMotorDriver.h>
#include "HardwareSerial.h"
#include "Arduino_NineAxesMotion.h"
#include <Wire.h>

TaskHandle_t Task1 ;
TaskHandle_t Task2 ;
CytronMD M2(PWM_DIR, 25, 26) ;
CytronMD M3(PWM_DIR, 27, 14) ;
CytronMD M4(PWM_DIR, 12, 13) ;
CytronMD M1(PWM_DIR, 32, 33) ;
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
void drible() ;
void ballHandling() ;
double pid(int) ;
int acc(int) ;
int d_acc(int) ;
void var_reset() ;

#define rly 19

int start , mode ;
int chassiSpd = 100 , chassis_Curr_Spd ;
bool curr_shr , pre_shr ;
bool curr_L1 , pre_L1 ;
bool curr_R1 , pre_R1 ;
bool curr_opt , pre_opt ;
bool curr_tou , pre_tou ;
bool curr_sqr , pre_sqr ;
bool curr_cro , pre_cro ;
bool curr_ps_opt , pre_ps_opt ;
bool curr_ps , pre_ps ;
int t_count , drible_preTime , drible_currTime ;
bool driblePermission = 0 ;
int o_angle , p_angle , Chassis_setpoint , l_angle ;
int perr ;
int p , x , y ;
double Chassis_kp = 3 , Chassis_ki = 0 , Chassis_kd = 5 ;
unsigned long lastStreamTime = 0 ;
const int streamPeriod = 20 ;   
int acc_curr_time , acc_pre_time ;
int d_acc_curr_time , d_acc_pre_time ;
int stop_flag , curr_chassis_flag , pre_chassis_flag ;
int FM_spd , LA_spd ;
int hand_count , action , s_var ;
int s_a_val ;
int curr_time ;
int curr_pistone_time , pre_pistone_time ;

void setup(){
  xTaskCreatePinnedToCore(MainCode, "Task1", 10000, NULL, 1, &Task1, 0) ;
  xTaskCreatePinnedToCore(Orientation, "Task2", 10000, NULL, 1, &Task2, 1) ;
  // Serial.begin(115200) ;
  espSerial.begin(250000) ;
  ps5.begin("A0:FA:9C:0D:D5:BE");
  // ps5.begin("7C:66:EF:3C:EF:D5");
  // ps5.begin("D0:BC:C1:A3:40:83");
  pinMode( rly , OUTPUT ) ;
  digitalWrite( rly , LOW ) ;
  Wire.begin() ;
  mySensor.initSensor() ;
  mySensor.setOperationMode(OPERATION_MODE_NDOF) ;
  mySensor.setUpdateMode(MANUAL) ;
  delay(300) ;
}

void loop(){
}

void MainCode   ( void * parameters ){
  while(1){
    if(ps5.isConnected()){
      Serial.print("Connected") ;
      Serial.print("\t") ;

      if(ps5.PSButton() && ps5.Share() && ps5.Options()){
        allStop() ;
        abort() ;
      }
      else{
        
        curr_time = millis() ;

        curr_shr = ps5.Share() ;
        if(curr_shr != pre_shr && curr_shr > 0) ++start ;
        pre_shr = curr_shr ;

        if(start%2 == 1){
          espSerial.println("K") ;
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

            chassis_Curr_Spd = acc ( chassis_Curr_Spd , chassiSpd , 1 , 4  ) ;
            if(     ps5.L2Value()>100)  {clk(map(ps5.L2Value() , 100 , 255 , 0 , 50)) ; Chassis_setpoint = o_angle ; /*curr_chassis_flag = 1 ;*/}
            else if(ps5.R2Value()>100)  {aclk (map(ps5.R2Value() , 100 , 255 , 0 , 50)) ; Chassis_setpoint = o_angle ; /*curr_chassis_flag = 2 ;*/}
            else if( ps5.Right() )      {right(x,y) ; stop_flag = 1 ; curr_chassis_flag = 3 ; }
            else if( ps5.Left()  )      {left (x,y) ; stop_flag = 2 ; curr_chassis_flag = 4 ; }
            else if(   ps5.Up()  )      {front(x,y) ; stop_flag = 3 ; curr_chassis_flag = 5 ; }
            else if(  ps5.Down() )      {back (x,y) ; stop_flag = 4 ; curr_chassis_flag = 6 ; }
          }
          else { 
            // curr_chassis_flag = 0 ;
            Chassis_setpoint = o_angle ;
            chassis_Curr_Spd = d_acc ( chassis_Curr_Spd , 0 , 1 , 3  )  ;
            M_stop( chassis_Curr_Spd , stop_flag ) ;
          }

          if( pre_chassis_flag != curr_chassis_flag ) chassis_Curr_Spd = 0 ;

          pre_chassis_flag = curr_chassis_flag ;
          
          ps5.Triangle() ? espSerial.println("G") : espSerial.println("g") ;
          ps5.Circle  () ? espSerial.println("H") : espSerial.println("h") ;

          if      ( ps5.RStickY() > 50 )    {espSerial.println("d") ; FM_spd = map(ps5.RStickY() ,  50 ,  127 , 0 , 255) ; espSerial.println(2500 + FM_spd);} 
          else if ( ps5.RStickY() < -50 )   {espSerial.println("e") ; FM_spd = map(ps5.RStickY() , -50 , -128 , 0 , 255) ; espSerial.println(2500 - FM_spd);}
          else                              {espSerial.println("f") ; FM_spd = 0                                         ; espSerial.println(2500);}

          Serial.print(ps5.RStickY()) ;
          Serial.print("\t") ;

          curr_ps_opt = ( ps5.PSButton() && ps5.Options() ) ;
          if(curr_ps_opt == 1 && pre_ps_opt == 0 ) ++action ;
          pre_ps_opt = curr_ps_opt ;

          if(action % 2 == 0){
            // Offence
            curr_ps = ps5.PSButton() ;
            if(curr_ps == 1 && pre_ps == 0 ) ++s_a_val ;
            pre_ps = curr_ps ;

            if      ( ps5.LStickY() >  50 )             {espSerial.println("D") ; LA_spd = map(ps5.LStickY() ,  50 ,  127 , 150 , 255)  ; espSerial.println( 1100 + LA_spd ) ; s_a_val = 0 ;}
            else if ( ps5.LStickY() < -50 )             {espSerial.println("E") ; LA_spd = map(ps5.LStickY() , -50 , -128 , 150 , 255)  ; espSerial.println( 1100 - LA_spd ) ; s_a_val = 0 ;}
            else if ( s_a_val != 0 && s_a_val % 3 == 0) {espSerial.println("J") ;}
            else if ( s_a_val != 0 && s_a_val % 3 == 1) {espSerial.println("L") ;}
            else if ( s_a_val != 0 && s_a_val % 3 == 2) {espSerial.println("l") ;}
            else                                        {espSerial.println("F") ; LA_spd = 0                                            ; espSerial.println( 1100 )          ;}

            Serial.printf("\tcurr_ps = %d , pre_ps = %d , s_a_val = %d \t" , curr_ps , pre_ps , s_a_val ) ;

            curr_cro = ps5.Cross() ;
            if(curr_cro != pre_cro && curr_cro > pre_cro)     ++s_var ;
            pre_cro = curr_cro ;
            if      (s_var % 2 == 0) espSerial.println("i") ;
            else if (s_var % 2 == 1) espSerial.println("I") ;

            curr_opt = ps5.Options() ;
            if ( curr_opt != pre_opt && curr_opt > 0 )  ++mode ;
            pre_opt = curr_opt ;
            if      (mode % 2 == 0) drible() ;
            else if (mode % 2 == 1) ballHandling() ;
            Serial.print("Offence") ;
            Serial.print("\t") ;
            action = 0 ;
          }
          else{
            // Defence
            mode = 0 ;
            espSerial.println("D") ;
            espSerial.println(1355) ;
            espSerial.println("A") ;
            espSerial.println("b") ;
            espSerial.println("i") ;
            Serial.print("Defence") ;
            Serial.print("\t") ;
          }
          Serial.print(action) ;
          Serial.print("\t") ;
        }
        else{
          allStop() ;
        }
      }
    }
    else{
      allStop() ;
      Serial.print("Not_Connected") ;
      var_reset() ;
    }
    Serial.print("\t") ;
    espSerial.println(l_angle) ;
    Serial.print(l_angle) ;
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
      // l_angle = mySensor.readEulerRoll()    ;
      l_angle = mySensor.readEulerPitch() ;
      if      (o_angle == 360)                       o_angle = 0 ;
      else if (o_angle > 180)                        o_angle = o_angle - 360 ; 
      if      (o_angle - Chassis_setpoint < -180)    p_angle = o_angle - Chassis_setpoint + 360 ;
      else if (o_angle - Chassis_setpoint > 180 )    p_angle = o_angle - Chassis_setpoint - 360 ;
      else                                           p_angle = o_angle - Chassis_setpoint ;
    }
  }
}

int acc(int spd , int target , int step , int time ) {
  acc_curr_time = curr_time ;
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
  d_acc_curr_time = curr_time ;
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

void drible(){
  t_count = 0 ;
  espSerial.println("A") ;
  curr_sqr = ps5.Square() ;
  if(curr_sqr ==1 && pre_sqr == 0) ++hand_count ;
  pre_sqr = curr_sqr ;

  drible_currTime = curr_time ;
  if(driblePermission==1 || ps5.Touchpad()){
    // 
    if(drible_currTime - drible_preTime < 30){
      driblePermission = 1 ;
      espSerial.println("b") ;
      espSerial.println("c") ;
      // hand_count = 0 ;
    }
    else if(drible_currTime - drible_preTime >= 800){
      driblePermission = 0 ;
    }
    else if(drible_currTime - drible_preTime >= 600)  {espSerial.println("b") ;}
    else if(drible_currTime - drible_preTime >= 250)  {espSerial.println("c") ;}
    else if(drible_currTime - drible_preTime >= 70 )  {espSerial.println("C") ;}
    else if(drible_currTime - drible_preTime >= 30 )  {espSerial.println("B") ;}

  }
  else{
    hand_count % 2 ? espSerial.println("B") : espSerial.println("b") ;
    drible_preTime = drible_currTime ;
    driblePermission = 0 ;
    // espSerial.println("b") ;
    espSerial.println("c") ;
  }
  Serial.print("Drible") ;
  Serial.print("\t") ;
}

void ballHandling(){
  espSerial.println("a") ;
  curr_tou = ps5.Touchpad();
  if(curr_tou != pre_tou && curr_tou > pre_tou)     ++t_count ;
  pre_tou = curr_tou ;

  curr_pistone_time = curr_time ;

  if(t_count % 2 == 0 )    { espSerial.println("b") ; pre_pistone_time = curr_pistone_time ; }
  else                     {
    espSerial.println("B") ;
    if(curr_pistone_time - pre_pistone_time > 30 ) {espSerial.println("c") ; Serial.print("offfffffffffff") ;}
    else                                            {espSerial.println("C") ; Serial.print("onnnnnnnnnnnnn") ;}
  }

  Serial.print("ballHandling") ;
  Serial.print("\t") ;
}

void back(int a , int b){
  M1.setSpeed(b)      ;    M2.setSpeed(b)     ;
  M3.setSpeed(-a)       ;    M4.setSpeed(-a)      ;
  Serial.print("back") ;   Serial.print("\t")  ;
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

void clk(int x){
  M1.setSpeed(-x)       ;   M2.setSpeed(-x)     ;
  M3.setSpeed(-x)       ;   M4.setSpeed(-x)     ;
  Serial.print("aclk")  ;   Serial.print("\t")  ;
}

void aclk(int x){
  M1.setSpeed(x)        ;   M2.setSpeed(x)      ;
  M3.setSpeed(x)        ;   M4.setSpeed(x)      ;
  Serial.print("clk")   ;   Serial.print("\t")  ;
}

void M_stop(int x , int flag ){
  switch(flag){
    case 0 :  
      M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(-100)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(-100) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(-100)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(-100) ;
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
      // M1.setSpeed(-100)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(-100) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(-100)   ;    M4.setSpeed(0) ;
      // delay(500) ;
      // M1.setSpeed(0)   ;    M2.setSpeed(0) ;
      // M3.setSpeed(0)   ;    M4.setSpeed(-100) ;
      // delay(500) ;
      break ;
  }
  Serial.print("stop") ;    Serial.print("\t") ;
}

void var_reset(){
  start = 0 ;
  mode = 0 ;
  chassiSpd = 100 ;
  chassis_Curr_Spd = 0 ; 
  stop_flag = 0 ;
  curr_chassis_flag = 0 ; 
  pre_chassis_flag = 0 ;
  FM_spd = 150 ;
  hand_count = 0 ;
  action = 0 ;
  espSerial.println("j") ;
}

void allStop(){
  M_stop(chassis_Curr_Spd , stop_flag ) ;
  digitalWrite( rly , LOW ) ;
  espSerial.println("a") ;
  espSerial.println("b") ;
  espSerial.println("c") ;
  espSerial.println("f") ;
  espSerial.println("i") ;
  espSerial.println("F") ;
  espSerial.println("k") ;
  espSerial.println(2500) ;
  espSerial.println(1100) ;
}
