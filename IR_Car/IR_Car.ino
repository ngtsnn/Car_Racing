#include "IR_Car.h"
#include <PID_v1.h>

#define MAX_ANGLE 60
#define KP        0.5
#define KI        0.0
#define KD        0.0

#define LOOP_TIME     50
#define TIME_TO_STOP  400
#define ACCELERATION  20
#define SPEEDRATIO    0.1

IRCar car;

// variables for ANGLE CONTROL
uint8_t sensor;
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
bool isTurning = false;
bool isSwitching = false;
DIRECTION dir;

//
DIRECTION side = NONE;

//LED
unsigned int *val;
unsigned int threshold[7] = {140, 140, 140, 140, 140, 140, 140};

void Show_Leds(void) {
  car.IRLed_GetAllAnalog();
  uint8_t filted = car.IRLed_GetAllFilted();
  car.Led_Display(filted);
}

void smoothTurn(double angleOffset)
{
  static float lastAngle;
  if(angleOffset > 0)
  {
    lastAngle += ACCELERATION;
    lastAngle = lastAngle > angleOffset ? angleOffset : lastAngle;
  }
  if(angleOffset < 0)
  {
    lastAngle -= ACCELERATION;
    lastAngle = lastAngle < angleOffset ? angleOffset : lastAngle;
  }

  car.Turn(lastAngle);
}

void switchToLeft();
void switchToRight();

void setup() 
{
  analogReference(INTERNAL);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MAX_ANGLE, MAX_ANGLE);
  car.Init();
  car.Stop();
  car.IRLed_SetAllThreshold(threshold);
  
  /* Timer 2 use for show leds */
  TCCR2B = 0x07; // prescaler /1024
  TCNT2 = 255;
  TIMSK2 = 0x01; //interrupt enable
  interrupts();

//  while(digitalRead(BT_4_PIN))
// 
//    unsigned int sensorValsOnBlack[7] = {0};
//    unsigned int sensorValsOnWhite[7] = {0};
//    // calibrate on black surface
//    while(digitalRead(BT_1_PIN);
//    for(uint8_t iLED = 0; iLED < 7; iLED++)
//    {
//      for(uint8_t i = 0; i < 16; i++)
//      {
//        sensorValsOnBlack[iLED] += car.IRLed_GetEachAnalog(iLED, true);
//      }
//      sensorValsOnBlack[iLED] >>= 4;
//    } //for
//
//    // calibrate sensor on white surface
//    while(digitalRead(BT_2_PIN));
//    for(uint8_t iLED = 0; iLED < 7; iLED++)
//    {
//      for(uint8_t i = 0; i < 16; i++)
//      {
//        sensorValsOnWhite[iLED] += car.IRLed_GetEachAnalog(iLED, true);
//      }
//      sensorValsOnWhite[iLED] >>= 4;
//
//      threshold[iLED] = (sensorValsOnBlack[iLED] + sensorValsOnWhite[iLED]) / 2;
//    } //for
//
//    break;
//  }//while
//  delay(50);
  while(digitalRead(BT_4_PIN));
  delay(500);
  car.Run(baseSpeed, baseSpeed);
  
}//set up

void loop() 
{
  static uint32_t t1_cnt, t2_cnt, onWhite_cnt, currentTime, timeReceivedReadySignal;
  
  currentTime = millis();
  if( currentTime - t1_cnt >= LOOP_TIME)
  {
    car.IRLed_GetAllAnalog(true);
    sensor = car.IRLed_GetAllFilted();
//    unsigned int *sensorVals = car.IRLed_GetAllAnalog(1);
//    for (int i = 0; i < 7; i++){
//      Serial.print(sensorVals[i]);
//      Serial.print("\t");
//    }
//     Serial.println();
     t1_cnt = currentTime;
  }
  
  currentTime = millis();
  if( currentTime - t2_cnt >= LOOP_TIME)
  {
    switch(sensor)
    {
      //center
      case 0b0001000:
        offset = 0;
        side = NONE;
        break;
      //align right
      case 0b0011000:
        offset = 5;
        break;
      case 0b0010000:
        offset = 15;
        side = RIGHT;
        break;
      case 0b0110000:
        offset = 25;
        side = RIGHT;
        break;
      case 0b0100000:
        offset = 35;
        break;
      case 0b1100000:
        if (side == LEFT){
          offset = -55;
        }
        else{
          offset = 45;
        }
        break;
      case 0b1000000:
        if (side == LEFT){
          offset = -50;
        }
        else{
          offset = 50;
        }
        break;
      //align left
      case 0b0001100:
        offset = -5;
        break;
      case 0b0000100:
        offset = -15;
        side = LEFT;
        break;
      case 0b0000110:
        offset = -25;
        side = LEFT;
        break;
      case 0b0000010:
        offset = -35;
        break;
      case 0b0000011:
        if (side == RIGHT){
          offset = 55;
        }
        else{
          offset = -45;
        }
        break;
      case 0b0000001:
        if (side == RIGHT){
          offset = 50;
        }
        else{
          offset = -50;
        }
        break;
      case 0b1000001:
        if (side == RIGHT){
          offset = 50;
        }
        else {
          offset = -50;
        }
        break;
        
      case 0b1110000: //switch lane to left
      case 0b1111000:
      case 0b1111100:
      case 0b1111110:
        if (dir == RIGHT){
          dir == NONE; 
          break;
        }
        dir = LEFT;
        if(isGoingToTurnOrSwitch){
          isTurning = true;
          isSwitching = false;
        } 
        else{
          isGoingToTurnOrSwitch = true;
          isTurning = false;
          isSwitching = false;
        }
        break;
      case 0b0000111: //switch lane to right
      case 0b0001111:
      case 0b0011111:
      case 0b0111111:
        if (dir == LEFT){
          dir == NONE; 
          break;
        }
        dir = RIGHT;
        if(isGoingToTurnOrSwitch){
          isTurning = true;
          isSwitching = false;
        }
        else{
          isGoingToTurnOrSwitch = true;
          isTurning = false;
          isSwitching = false;
        }
        break;
      case 0b1111111: //full white
       onWhite_cnt += currentTime - t2_cnt;
       isSwitching = false;
        static uint8_t blankingTime;
        if(blankingTime == 0)
        {
          offset = 0;
          if(isGoingToTurnOrSwitch && dir != NONE){
            isTurning = true;
          }
          else 
          {
            isGoingToTurnOrSwitch = true;
            isTurning = false;
          }
          blankingTime = 2;
        }
        else
        {
          blankingTime--;
        }
 
        break;
      case 0b0000000: //full black
        offset = 0;
        isTurning = false;
        if(isGoingToTurnOrSwitch && dir != NONE){
          isSwitching = true;
        }
        else
        {
          isGoingToTurnOrSwitch = false;
        }
        break;
      default:
        offset = oldOffset;
    }
    
    //Serial.println(offset, 1);
    myPID.Compute();
    smoothTurn(angleOffset);

    car.Run(baseSpeed + (int16_t)(SPEEDRATIO * angleOffset), 
            baseSpeed - (int16_t)(SPEEDRATIO * angleOffset));

    //Get time when we got Ready Signal
    if(isGoingToTurnOrSwitch){
      timeReceivedReadySignal += currentTime - t2_cnt; 
    } else{
      timeReceivedReadySignal = 0;
    }

    //If the Ready Signal we got and Turning signal come too quickly
    //we skip this signal maybe this is the Jam signal.  
    if ((isSwitching || isTurning) && timeReceivedReadySignal <= 200)   
    {
      isGoingToTurnOrSwitch = isSwitching = isTurning = false;
      dir = NONE;
    }
    else if (isSwitching && dir != NONE){
      Serial.println("switch");
      car.Turn((int16_t)60 * dir);
      car.Run(baseSpeed + dir*20, baseSpeed - dir*20);
      delay(300);
      car.Turn(0);
      // run until middle sensor activated
      while(1)
      {
        car.IRLed_GetAllAnalog(true);
        uint8_t x = car.IRLed_GetAllFilted();
        
        if(x & 0b0001000)
        {
          break; 
        }
        delay(50);
      }
      car.Turn((int16_t)-60 * dir);
      car.Run(baseSpeed - dir*20, baseSpeed + dir*20);
      delay(100);
      car.Turn(0);
      car.Run(baseSpeed, baseSpeed);
      isGoingToTurnOrSwitch = isSwitching = isTurning = false;
      dir = NONE;
    }
    else if (isTurning && dir != NONE){
      Serial.println("turn");
      while(1)
      {
        car.IRLed_GetAllAnalog(true);
        if(car.IRLed_GetAllFilted() == 0)
        {
          break;
        }
      }

      car.Turn(60 * dir);
      car.Stop();
      car.Run(100 * dir, -100 * dir);

      while(1)
      {
        
        car.IRLed_GetAllAnalog(true);
        uint8_t x = car.IRLed_GetAllFilted();
        
        if(x & 0b0001000)
        {
          Serial.println("break");
          break; 
        }
        delay(50);
      }
      car.Run(baseSpeed, baseSpeed);
      car.Turn(0);
      dir = NONE;
      isGoingToTurnOrSwitch = isTurning = isSwitching = 0;
    }
    
//    if(sensor & 0b0001000)
//    {
//      Serial.println("ok");
//    }

    Serial.println(dir == -1 ? "left" : (dir == 0 ? "center" : "right"));
    
    //check no full white 
    if (sensor != 0b1111111){
      onWhite_cnt = 0;
    }

    if( onWhite_cnt >= TIME_TO_STOP){
      car.Run(0, 0);
    }

    oldOffset = offset;
    t2_cnt = currentTime;
  }
  
} //end loop

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
