#include "HardwareSerial.h"
#include <CytronMotorDriver.h>
// #include <Adafruit_NeoPixel.h> 

// #define RGB_BUILTIN 38
#define dribllingPlateform 13
#define dribllingHand 27
#define dribllingPiston 14
// #define rly 19
#define FD 26
#define FP 2

String var , curr_tri , pre_tri , curr_cir , pre_cir  ;
int num ;
int S_motorSpd = 40 , l_angle , setpoint = 26 ;
int fm_spd = 0 , la_spd = 0;
int p ;
int perr ;

double pid(int) ;

HardwareSerial espSerial(0);
CytronMD M1(PWM_DIR, 16, 17) ;
CytronMD M2(PWM_DIR, 22, 21) ;
CytronMD FM(PWM_DIR, 25, 26) ;
CytronMD LA(PWM_DIR, 32, 33) ;

void setup() {
  espSerial.begin(250000) ;
  pinMode( dribllingPlateform   , OUTPUT ) ;
  pinMode( dribllingHand        , OUTPUT ) ;
  pinMode( dribllingPiston      , OUTPUT ) ;
  // pinMode( rly                  , OUTPUT ) ;
  pinMode( FD                   , OUTPUT ) ;
  digitalWrite(dribllingPlateform , HIGH ) ;
  digitalWrite(dribllingHand      , HIGH ) ;
  digitalWrite(dribllingPiston    , HIGH ) ;
  // digitalWrite(rly                , HIGH ) ;
  digitalWrite(FD                 , HIGH ) ;
}

void loop() {
  if(espSerial.available()){
    var = espSerial.readStringUntil('\n') ; 
    var.trim() ;
  }
  
  if(var == "A" || var == "B" || var == "C" || var == "D" || var == "E" || var == "F" || var == "G" || var == "H" || var == "I" || var == "J" || var == "K" || var == "L" || var == "M" || 
     var == "a" || var == "b" || var == "c" || var == "d" || var == "e" || var == "f" || var == "g" || var == "h" || var == "i" || var == "j" || var == "k" || var == "l" || var == "m" ){
    
    if      (var == "A")  digitalWrite(dribllingPlateform , LOW  ) ;
    else if (var == "a")  digitalWrite(dribllingPlateform , HIGH ) ;
    
    if      (var == "B")  {digitalWrite(dribllingHand      , LOW  ) ;}
    else if (var == "b")  {digitalWrite(dribllingHand      , HIGH ) ;}
    
    if      (var == "C")  {digitalWrite(dribllingPiston    , LOW  ) ;}
    else if (var == "c")  {digitalWrite(dribllingPiston    , HIGH ) ;}

    ;

    if      (var == "D")  {LA.setSpeed ( la_spd ) ;                 ; espSerial.printf(" %d , %d , %d \t" , l_angle , setpoint , p ) ;espSerial.printf("%s\n",var) ;}
    else if (var == "E")  {LA.setSpeed ( la_spd ) ;                 ; espSerial.printf(" %d , %d , %d \t" , l_angle , setpoint , p ) ;espSerial.printf("%s\n",var) ;}
    else if (var == "J")  {setpoint = 35          ; LA.setSpeed(-p) ; espSerial.printf(" %d , %d , %d \t" , l_angle , setpoint , p ) ;espSerial.printf("%s\n",var) ;}
    else if (var == "L")  {setpoint = 38          ; LA.setSpeed(-p) ; espSerial.printf(" %d , %d , %d \t" , l_angle , setpoint , p ) ;espSerial.printf("%s\n",var) ;}
    else if (var == "l")  {setpoint = 42          ; LA.setSpeed(-p) ; espSerial.printf(" %d , %d , %d \t" , l_angle , setpoint , p ) ;espSerial.printf("%s\n",var) ;}
    else if (var == "F")  {LA.setSpeed (0)        ;                 ; espSerial.printf(" %d , %d , %d \t" , l_angle , setpoint , p ) ;espSerial.printf("%s\n",var) ;}

    if( var == "G" || var == "g"){
      curr_tri = var ;
      if(curr_tri == "G" && pre_tri == "g" ) S_motorSpd += 5 ;
      pre_tri = curr_tri ;
    }

    if( var == "H" || var == "h" ){
      curr_cir = var ;
      if(curr_cir == "H" && pre_cir == "h" ) S_motorSpd -= 5 ;
      pre_cir = curr_cir ;
    }

    S_motorSpd = constrain( S_motorSpd , 0 , 255 ) ;
    
    if      (var == "d")   {digitalWrite( FD , LOW  ) ; analogWrite(FP , fm_spd ) ; FM.setSpeed(fm_spd) ;}
    else if (var == "e")   {digitalWrite( FD , HIGH ) ; analogWrite(FP , fm_spd ) ; FM.setSpeed(fm_spd) ;}
    else if (var == "f")   {                            analogWrite(FP ,    0   ) ; FM.setSpeed(0) ;}


    if      (var == "I")  {M1.setSpeed(S_motorSpd) ;  M2.setSpeed(S_motorSpd) ;}

    else if (var == "i")  {M1.setSpeed(0)          ;  M2.setSpeed(0)           ;}

    if      (var == "j")   S_motorSpd = 40 ;

    // if      ( var == "K") digitalWrite( rly , LOW ) ;
    // else if ( var == "k") digitalWrite( rly , HIGH) ;

    if(l_angle != 0){
      p = pid(50,0,20,-l_angle,setpoint) ;
      p = constrain(p,-255,255) ;
    }
  }
  else{  
    num = var.toInt() ;
  }

  if (num <  200 )              l_angle = num ;
  if (num == 2500 )             fm_spd = 0 ;
  if (num >  2200 && num < 2800)fm_spd = 2500 - num ;
  if (num == 1100 )             la_spd = 0 ;
  if (num >  800 && num < 1400) la_spd = num - 1100 ;

  // fm_spd = abs(fm_spd) ;
  
  // espSerial.print(num) ;
  // espSerial.print("\t") ;
  // espSerial.print(l_angle) ;
  // espSerial.print("\t") ;
  // espSerial.print(la_spd) ;
  // espSerial.print("\t") ;
  // espSerial.print(fm_spd) ;
  // espSerial.print("\n") ;
  // neopixelWrite(RGB_BUILTIN,abs(fm_spd/3),abs(la_spd),abs(fm_spd)); // Off / black
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
