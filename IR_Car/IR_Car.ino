#include "IR_Car.h"
#include <PID_v1.h>

#define MAX_ANGLE 60
#define KP        0.6
#define KI        0.05
#define KD        0.05

#define LOOP_TIME     50
#define TIME_TO_STOP  400
#define ACCELERATION  3

IRCar car;

// this variables for ANGLE CONTROL
double offset, setPoint = 0, angleOffset, oldOffset;
PID myPID(&offset, &angleOffset, &setPoint, KP, KI, KD, P_ON_M, DIRECT);
float oldAngle = 0;

// this variables for MODIFY SPEED
float speedRatio = 1;
int baseSpeed = 100;

// SWITCHING AND TURNING
enum DIRECTION
{
  LEFT = -1,
  NONE = 0,
  RIGHT = 1
};
bool isGoingToTurnOrSwitch = false;
bool isTurning = 0;
bool isSwitching = 0;
DIRECTION dir;

//LED
unsigned int *val;
unsigned int threshold[7] = {30, 30, 30, 30, 30, 30, 30};

void Show_Leds(void) {
  car.IRLed_GetAllAnalog();
  uint8_t filted = car.IRLed_GetAllFilted();
  car.Led_Display(filted);
}

float smoothDampAngle(double oldAngleOffset, double angleOffset, float t){
  double deltaAngel = angleOffset - oldAngleOffset;
  return oldAngleOffset + deltaAngel * t * speedRatio;
}

void switchToLeft();
void switchToRight();

void setup() 
{
  while(digitalRead(BT_4_PIN));
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MAX_ANGLE, MAX_ANGLE);
  car.Init();
  car.IRLed_SetAllThreshold(threshold);
  car.Stop();
  delay(500);

  /* Timer 2 use for show leds */
  TCCR2B = 0x07; // prescaler /1024
  TCNT2 = 255;
  TIMSK2 = 0x01; //interrupt enable
  interrupts();

  car.Run(baseSpeed, baseSpeed);
  
}

void loop() 
{

  int max_handle = 60;
  static long t1_cnt, onWhite_cnt, currentTime;

  currentTime = millis();
  if( currentTime - t1_cnt >= LOOP_TIME)
  {

    uint8_t sensor=car.IRLed_GetAllFilted();
    int pattern=0;
    //car.IRLed_SerialPrintFilted(sensor);
    switch(sensor)
    {
      //center
      case 0b0001000:
        offset = 0;
        break;
      //align right
      case 0b0011000:
        offset = 5;
        break;
      case 0b0010000:
        offset = 20;
        break;
      case 0b0110000:
        offset = 25;
        break;
      case 0b0100000:
        offset = 40;
        break;
      case 0b1100000:
        offset = 45;
        break;
      case 0b1000000:
        offset = 60;
        break;
      //align left
      case 0b0001100:
        offset = -5;
        break;
      case 0b0000100:
        offset = -20;
        break;
      case 0b0000110:
        offset = -25;
        break;
      case 0b0000010:
        offset = -40;
        break;
      case 0b0000011:
        offset = -45;
        break;
      case 0b0000001:
        offset = -60;
        break;
      case 0b1110000: //switch lane to left
      case 0b1111000:
      case 0b1111100:
      case 0b1111110:
        dir = LEFT;
        if (isGoingToTurnOrSwitch){
          car.Turn(-60);
          car.Stop();
          car.Run(-100, 100);
          delay(800);
          car.Run(baseSpeed, baseSpeed);
          car.Turn(0);
          dir = NONE;
          isGoingToTurnOrSwitch = 0;          
        }
        else{
          isGoingToTurnOrSwitch = 1;
        }
        break;
      case 0b0000111: //switch lane to right
      case 0b0001111:
      case 0b0011111:
      case 0b0111111:
        dir = RIGHT;
        if (isGoingToTurnOrSwitch){
          car.Turn(60);
          car.Stop();
          car.Run(100, -100);
          delay(800);
          car.Run(baseSpeed, baseSpeed);
          car.Turn(0);
          dir = NONE;
          isGoingToTurnOrSwitch = 0;
        }
        else{
          isGoingToTurnOrSwitch = 1;
        }
        break;
      case 0b1111111: //full white
        offset = 0;
         if (isGoingToTurnOrSwitch && dir != NONE){
          if(dir == RIGHT)
          {
            car.Turn(60);
            car.Stop();
            car.Run(100, -100);
          }
          if(dir == LEFT)
          {
            car.Turn(-60);
            car.Stop();
            car.Run(-100, 100);
          }
          delay(800);
          car.Run(baseSpeed, baseSpeed);
          car.Turn(0);
          dir = NONE;
          isGoingToTurnOrSwitch = 0;          
        }
        else{
          isGoingToTurnOrSwitch = 1;
          dir = NONE;
        }

        onWhite_cnt += currentTime - t1_cnt;
        break;
      case 0b0000000: //full black
        offset = 0;
        if(isGoingToTurnOrSwitch){
          car.Turn((int16_t)60 * dir);
          car.Run(baseSpeed + dir*20, baseSpeed - dir*20);
          delay(300);
          car.Turn(0);
          // run until middle sensor activated
//          while(1)
//          {
//            uint8_t x = car.IRLed_GetAllFilted();
//            if(x & 0b0001000)
//            {
//              car.Run(0, 0);
//              break; 
//            }
//          }

          delay(dir == LEFT ? 400 : 800); //temporary solution 
          car.Turn((int16_t)-60 * dir);
          car.Run(baseSpeed - dir*20, baseSpeed + dir*20);
          delay(400);
          car.Turn(0);
          car.Run(baseSpeed, baseSpeed);
          isGoingToTurnOrSwitch = false;
          dir = NONE;
        }
        else
        {
          
        }
        break;
      default:
        offset = oldOffset;
    }
    //Serial.println(offset, 1);
    myPID.Compute();
    float newAngle = smoothDampAngle(oldAngle, angleOffset, .7);
    oldAngle = newAngle;
//    Serial.println(angleOffset, 1);
    car.Turn((int16_t)newAngle);


    //check no full white 
    if (sensor != 0b1111111){
      onWhite_cnt = 0;
    }

    if( onWhite_cnt >= TIME_TO_STOP){
      car.Run(0, 0);
    }

    oldOffset = offset;
    t1_cnt = currentTime;
  }
  
//  if ( (sensor == 0b0001000 || sensor == 0b0011000 || sensor == 0b0001100||sensor == 0b0011100)) {            
//    pattern = 0;
//    c.Run(speed_car, speed_car);
//    c.Turn(0);
//  }
//  if (pattern == 0 && (sensor == 0b0010000 || sensor == 0b0110000 || sensor == 0b1100000 )) {
//    pattern = 1;
//    
//  }
//  //xac dinh lech trai
//  if (pattern == 0 && (sensor == 0b0000100 || sensor == 0b0000110 || sensor == 0b0000011 )) {
//    pattern = 2;
//  }
//  //lech phai
//  if (pattern == 1) {
//    switch (sensor) {
//      case 0b0010000:
//        c.Turn(-0.2*max_handle);
//        c.Run(0.9*speed_car,0.92*speed_car);
//
//        break;
//      case 0b0110000:
//        c.Turn(-0.4*max_handle);
//        c.Run(0.745*speed_car,0.82*speed_car);
//
//        break;
//      case 0b1100000:
//        c.Turn(-0.55*max_handle);
//        c.Run(0.65*speed_car,0.78*speed_car);
//
//        break;
//      case 0b1110000:
//        c.Turn(-0.55*max_handle);
//        c.Run(0.65*speed_car,0.78*speed_car);
//    }
//  }
//  
//  //lech phai
//  if (pattern == 2) {
//    switch (sensor) {
//      case 0b0000100:
//        c.Turn(0.2*max_handle);
//        c.Run(0.92*speed_car,0.9*speed_car); 
//
//        break;
//      case 0b0000110:
//        c.Turn(0.4*max_handle);
//        c.Run(0.82*speed_car,0.745*speed_car);
//
//        break;
//      case 0b0000011:
//        c.Turn(0.55*max_handle);
//        c.Run(0.78*speed_car,0.7*speed_car);
//       case 0b0000111:
//        c.Turn(0.55*max_handle);
//        c.Run(0.78*speed_car,0.7*speed_car);
//        break;
//    }
//  }
}

ISR(TIMER2_OVF_vect) //ISR for Timer1 Overflow
{
  Show_Leds();
}

void switchToLeft(){

  while(1){
    uint8_t sensor=car.IRLed_GetAllFilted();
    if (sensor == 0b0000000){
      while(1){
        
      }
    }
  }
}

void switchToRight()
{
  
}
