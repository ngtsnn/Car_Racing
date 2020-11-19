 #include "IR_Car.h"
#include <PID_v1.h>

#define MAX_ANGLE 60
#define KP        0.55
#define KI        0.016
#define KD        0.02

#define LOOP_TIME       40
#define TIME_TO_STOP    400
#define ACCEL           10
#define STEERING_ACCEL  15
#define SPEEDRATIO      0.15
#define DEFINED_READY_TIME 150

IRCar car;

// variables for ANGLE CONTROL
uint8_t sensor;
double offset, setPoint = 0, angleOffset, oldOffset;
PID myPID(&offset, &angleOffset, &setPoint, KP, KI, KD, P_ON_M, DIRECT);
float oldAngle = 0;

// these variables are for MODIFY SPEED
float speedRatio = 1;
int baseSpeed = 120;

// SWITCHING AND TURNING
enum DIRECTION
{
  LEFT = -1,
  NONE = 0,
  RIGHT = 1
};
bool ready = false;
bool isTurning = false;
bool isSwitching = false;
bool passedFullWhite = false;
bool obstacle = false;
DIRECTION dir;

//
DIRECTION side = NONE;
bool isLosingLine = false;

bool checkStopSignal;

//LED
unsigned int *val;
unsigned int threshold[7] = {500, 500,  500,  500,  500,  500,  500};

uint8_t countToStop;
bool finishedRacing = false;

uint8_t readSensor(void) 
{
  car.IRLed_GetAllAnalog();
  uint8_t x = car.IRLed_GetAllFilted();
  car.Led_Display(x);
  return x;
}

void smoothRun(int16_t leftVal, int16_t rightVal)
{
  static int16_t lastLeftVal, lastRightVal;
  int16_t sign;

  sign = lastLeftVal < leftVal ? 1 : -1;
  if((sign*(leftVal - lastLeftVal)) > ACCEL)
  {
    lastLeftVal += sign*ACCEL;
  }
  else
  {
    lastLeftVal = leftVal;
  }
  
  sign = lastRightVal < rightVal ? 1 : -1;
  if((sign*(rightVal - lastRightVal)) > ACCEL)
  {
    lastRightVal += sign*ACCEL;
  }
  else
  {
    lastRightVal = rightVal;
  }
  
  car.run(lastLeftVal, lastRightVal);
}

void smoothTurn(double angleOffset)
{
  static float lastAngle;
  if(angleOffset > 0)
  {
    lastAngle += STEERING_ACCEL;
    lastAngle = lastAngle > angleOffset ? angleOffset : lastAngle;
  }
  if(angleOffset < 0)
  {
    lastAngle -= STEERING_ACCEL;
    lastAngle = lastAngle < angleOffset ? angleOffset : lastAngle;
  }
  car.turn(lastAngle);
}

void switchToLeft();
void switchToRight();

void setup() 
{
  analogReference(INTERNAL);
  myPID.SetSampleTime(LOOP_TIME);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MAX_ANGLE, MAX_ANGLE);
  
  car.Init();
  car.stop();
  car.IRLed_SetAllThreshold(threshold);

  uint8_t LED_status = 0b01000000;
  car.Led_Display(LED_status);
  while(digitalRead(BT_4_PIN))
  {
    if(!digitalRead(BT_1_PIN))
    {
      countToStop = (countToStop + 1)%5;
      car.Led_Display(LED_status | countToStop);
    }
    delay(200);
  }
  LED_status = 0b11000000;
  car.Led_Display(LED_status | countToStop);
  delay(500);
  /* Timer 2 use for show leds */
//  TCCR2B = 0x07; // prescaler /1024
//  TCNT2 = 255;
//  TIMSK2 = 0x01; //interrupt enable
//  interrupts();

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
  car.run(baseSpeed, baseSpeed);
  
}//set up

void loop() 
{
  static uint32_t t1_cnt, onWhite_cnt, currentTime, readyInterval, dt;
  
  currentTime = millis();
  dt = currentTime - t1_cnt;

  if( dt >= LOOP_TIME && !finishedRacing)
  {
    //find obstacles
    if(car.getDistance()){
      //slow down the car
//      car.run(80, 80);

      car.run(60, 100);
      //steer a little bit left
      for(int angle = 0; angle >= -60; angle -= 5){
        car.turn(angle);
        delay(15);
      }
      car.run(80, 80);
      
      //go straight
      car.turn(0);
      delay(800);
      
      car.run(100, 60);
      //steer right
      for(int angle = 0; angle <= 60; angle += 5){
        car.turn(angle);
        delay(40);        
      }
      car.run(80, 80);
      //go straight
      car.turn(0);

      //catch middle line
      while(1)
      {
        if((readSensor() & 0b1001001) == 0b0001000)
        {
          break; 
        }
        delay(10);
      }

      car.turn(-15);
      delay(300);
      car.turn(0);
      delay(100);

      car.run(baseSpeed, baseSpeed);
      ready = isTurning = isSwitching = false;
      dir = side = NONE;
    }
    
    sensor = readSensor();
    obstacle = car.getDistance();
    //Get time when we got Ready Signal
    if(ready)
    {
      readyInterval += dt; 
      if(readyInterval >= DEFINED_READY_TIME)
      {
        passedFullWhite = 0;
      }
    } 
    else
    {
      readyInterval = 0;
    }
    
    switch(sensor)
    {
      //center
      case 0b0001000:
        offset = 0;
        side = NONE;
        break;
      //align right
      case 0b0011000:
        offset = 6;
        side = RIGHT;
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
        if (side == LEFT)
        {
          offset = -60;
        }
//        else if(isLosingLine){
//          offset = -60;
//        }
        else
        {
          offset = 45;
        }
        break;
      case 0b1000000:
        if (side == LEFT)
        {
          offset = -70;
        }
//        else if(isLosingLine){
//          offset = -60;
//        }
        else
        {
          offset = 70;
        }
        break;
      //align left
      case 0b0001100:
        offset = -6;
        side = LEFT;
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
        if (side == RIGHT)
        {
          offset = 55;
        }
        else
        {
          offset = -45;
        }
        break;
      case 0b0000001:
        if (side == RIGHT)
        {
          offset = 70;
        }
//        else if(isLosingLine){
//          offset = 60;
//        }
        else{
          offset = -70;
        }
        break;
      case 0b1000001:
        if (side == RIGHT)
        {
          offset = 60;
        }
//        else if(isLosingLine){
//          offset = 60;
//        }
        else
        {
          offset = -60;
        }
        break;
        
      case 0b1110000: //switch lane to left
      case 0b1111000:
      case 0b1111100:
//      case 0b1111110:
      {
        if (dir == RIGHT)
        {
          dir = NONE; 
          break;
        }

        dir = LEFT;
        if(ready && readyInterval >= DEFINED_READY_TIME)
        {
          Serial.println("Turn left!");
            isTurning = true;
            isSwitching = false;
        } 
        else
        {
          Serial.println("Ready from left!");
          ready = true;
          isTurning = false;
          isSwitching = false;
        }

        break;
      }
      case 0b0000111: //switch lane to right
      case 0b0001111:
      case 0b0011111:
//      case 0b0111111:
      {
        if (dir == LEFT)
        {
          dir = NONE;
          break;
        }

        dir = RIGHT;
        if(ready && readyInterval >= DEFINED_READY_TIME)
        {
          Serial.println("turn right!");
          isTurning = true;
          isSwitching = false;
          
        }
        else 
        {
          Serial.println("ready from right!");
          ready = true;
          isTurning = false;
          isSwitching = false;
        }
//        static uint8_t blankingCount;
//        dir = RIGHT;
//        if(blankingCount == 0)
//        {
//          if(ready)
//          {
//            isTurning = true;
//            isSwitching = false;
//          } 
//          else
//          {
//            ready = true;
//            isTurning = false;
//            isSwitching = false;
//          }
//          blankingCount = 1;
//        }
//        else{
//          blankingCount--;
//        }

        break;
      }
      case 0b1111111: //full white
        onWhite_cnt += dt;
        isSwitching = false;
        offset = 0;

        if(ready && dir != NONE)
        {
          if(readyInterval >= DEFINED_READY_TIME)
          {
            Serial.println("turn from direction!");
            isTurning = true;
          }
          else  
          {
            // If ready flag was set but readyInterval is too small,
            //  reset readyInterval (still in the same white strip)
            readyInterval = 0;
          }
        }
        else // just received ready signal
        {
          Serial.println("ready from White!");
          ready = true;
          isTurning = false;
          passedFullWhite = true;
          dir = NONE;
        }
        break;
      case 0b0000000: //full black
        offset = 0;
        isTurning = false;

        if(ready && readyInterval >= DEFINED_READY_TIME)
        {
          if(dir != NONE)
          {
            Serial.println("switch from direction!");
            isSwitching = true;
          }
          else{
            isSwitching = ready = isTurning = false;
            dir = side = NONE;
            isLosingLine = true;
            Serial.println("lose line!");
          }
        }
        else
        {
          ready = false;
          isSwitching = false;
          isTurning = false;
        }
        break;

      
      default:
        offset = oldOffset;
    }
    
    //Serial.println(offset, 1);

    //If the Ready Signal we got and Turning signal come too quickly
    //we skip this signal maybe this is the Jam signal.  
    if (isSwitching || isTurning)   
    {
      if (isSwitching && dir != NONE)
      {
        car.turn((int16_t)60 * dir);
        car.run(baseSpeed + dir*20, baseSpeed - dir*20);
        delay(300);
        car.turn(0);
        // run until middle sensor activated
        while(1)
        {
          if(readSensor() & 0b0001000)
          {
            break; 
          }
          delay(10);
        }
        car.turn((int16_t)-20 * dir);
        car.run(baseSpeed - dir*30, baseSpeed + dir*30);
        delay(200);
        car.turn((int16_t)-40 * dir);
        car.run(baseSpeed - dir*30, baseSpeed + dir*30);
        delay(200);
        car.turn(0);
        car.run(baseSpeed, baseSpeed);
        
        ready = isSwitching = isTurning = false;
        dir = side = NONE;
        passedFullWhite = false;
      }




      //TURN
      else if(isTurning && dir != NONE)
      {
//        car.setLed(7, true);
        while(1)
        {
          if(readSensor() == 0)
          {      
            break;
          }
          delay(5);
        }
        car.run(-255, -255);
        delay(80);

        car.turn(60 * dir);
        if(dir == LEFT)
        {
          car.run(40, 130);
        }
        else
        {
          car.run(130, 40);
        }
        delay(300);
        
        while(1)
        {
          if((readSensor() & 0b1001001) == 0b0001000)
          {
            break; 
          }
          delay(10);
        }
        car.run(baseSpeed - 60*dir, baseSpeed + 60*dir);
        car.turn(-15*dir);
        delay(500);
        car.run(baseSpeed - 40*dir, baseSpeed + 40*dir);
        car.turn(0);
        delay(200);
        
        car.run(baseSpeed, baseSpeed);
        car.turn(0);
        
        dir = side = NONE;
        ready = isTurning = isSwitching = 0;
        passedFullWhite = false;
      }
    }




    
    else
    {
      myPID.SetSampleTime(dt);
      myPID.Compute();
      smoothTurn(angleOffset);
      if(ready){
//        baseSpeed -= 5;
//        baseSpeed = baseSpeed < 80 ? 80: baseSpeed;
        baseSpeed = 80;
      }
      else if(sensor & 0b1100011)
      {
        baseSpeed -= 5;
        baseSpeed = baseSpeed < 120 ? 120: baseSpeed;
      }
      else
      {
        baseSpeed += 3;
        baseSpeed = baseSpeed > 150 ? 150: baseSpeed;
      }
      car.run(baseSpeed + (int16_t)(SPEEDRATIO * angleOffset), 
              baseSpeed - (int16_t)(SPEEDRATIO * angleOffset));
    }
    
//    if(sensor & 0b0001000)
//    {
//      Serial.println("ok");
//    }


    //check no full black
    if (isLosingLine){
      if (readSensor() & 0b0011100)
        isLosingLine = false;
    }
    
    //check no full white 
    if (sensor != 0b1111111)
    {
      onWhite_cnt = 0;
      checkStopSignal = true;
    }

    if(onWhite_cnt >= TIME_TO_STOP && checkStopSignal)
    {
      countToStop--;
      if(countToStop == 0)
      {
        finishedRacing = true;
        car.stop();
      }
      
      onWhite_cnt = 0;
      checkStopSignal = false;
    }

    oldOffset = offset;
    t1_cnt = currentTime;
  }

} //end loop

ISR(TIMER2_OVF_vect) //ISR for Timer1 Overflow
{
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
