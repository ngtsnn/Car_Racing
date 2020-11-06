 #include "IR_Car.h"
#include <PID_v1.h>

#define MAX_ANGLE 60
#define KP        0.6
#define KI        0.06
#define KD        0.04

#define LOOP_TIME       50
#define TIME_TO_STOP    400
#define ACCEL           10
#define STEERING_ACCEL  15
#define SPEEDRATIO      0.1

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
DIRECTION dir;

//
DIRECTION side = NONE;
bool isLosingLine = false;

//LED
unsigned int *val;
unsigned int threshold[7] = {140, 140, 140, 140, 140, 140, 140};

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
  Serial.print("count = ");
  Serial.println(countToStop);
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
  static uint32_t t1_cnt, t2_cnt, onWhite_cnt, currentTime, readyInterval;
  
  currentTime = millis();
  if( currentTime - t1_cnt >= LOOP_TIME)
  {
    sensor = readSensor();
//    unsigned int *sensorVals = car.IRLed_GetAllAnalog(0);
//    for (int i = 0; i < 7; i++){
//      Serial.print(sensorVals[i]);
//      Serial.print("\t");
//    }
//     Serial.println();
     t1_cnt = currentTime;
  }
  
  currentTime = millis();
  if( currentTime - t2_cnt >= LOOP_TIME && !finishedRacing)
  {
    //Get time when we got Ready Signal
    if(ready)
    {
      readyInterval += currentTime - t2_cnt; 
      if(readyInterval > 300)
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
        if (side == LEFT)
        {
          offset = -55;
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
          offset = -50;
        }
//        else if(isLosingLine){
//          offset = -60;
//        }
        else
        {
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
          offset = 50;
        }
//        else if(isLosingLine){
//          offset = 60;
//        }
        else{
          offset = -50;
        }
        break;
      case 0b1000001:
        if (side == RIGHT)
        {
          offset = 50;
        }
//        else if(isLosingLine){
//          offset = 60;
//        }
        else
        {
          offset = -50;
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
        if(ready && readyInterval > 300)
        {
            isTurning = true;
            isSwitching = false;
        } 
        else
        {
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
        if(ready && readyInterval > 300)
        {
          isTurning = true;
          isSwitching = false;
        }
        else 
        {
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
        onWhite_cnt += currentTime - t2_cnt;
        isSwitching = false;
        offset = 0;

        if(ready && dir != NONE)
        {
          if(readyInterval > 300)
          {
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
          ready = true;
          isTurning = false;
          passedFullWhite = true;
          dir = NONE;
        }
        break;
      case 0b0000000: //full black
        offset = 0;
        isTurning = false;
        isLosingLine = true;
        if(ready && readyInterval > 300)
        {
          if(dir != NONE)
            isSwitching = true;
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
        Serial.println("switch");
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
      else if(isTurning && dir != NONE)
      {
        Serial.println("turn");
        while(1)
        {
          if(readSensor() == 0)
          {
            break;
          }
          delay(5);
        }
  
        car.turn(60 * dir);
        car.run(120 * dir, -120 * dir);
        uint8_t sensorMask, sensorVal;
        
        if(dir == RIGHT)
        {
          sensorMask = 0b1001001;
          sensorVal = 0b0001000;
        }
        else
        {
          sensorMask = 0b1001001;
          sensorVal = 0b0001000;        
        }
        while(1)
        {
          if((readSensor() & sensorMask) == sensorVal)
          {
            Serial.println("break");
            break; 
          }
          delay(10);
        }
        car.run(baseSpeed - 40*dir, baseSpeed + 40*dir);
        car.turn(-30*dir);
        delay(100);
        car.run(baseSpeed - 20*dir, baseSpeed + 20*dir);
        car.turn(-30*dir);
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
      myPID.Compute();
      smoothTurn(angleOffset);
      car.run(baseSpeed + (int16_t)(SPEEDRATIO * angleOffset), 
              baseSpeed - (int16_t)(SPEEDRATIO * angleOffset));
    }
    
//    if(sensor & 0b0001000)
//    {
//      Serial.println("ok");
//    }

    Serial.println(dir == -1 ? "left" : (dir == 0 ? "center" : "right"));

    //check no full black
    if (isLosingLine){
      if (readSensor() & 0b0011100)
        isLosingLine = false;
    }
    
    //check no full white 
    if (sensor != 0b1111111)
    {
      onWhite_cnt = 0;
    }
    if( onWhite_cnt >= TIME_TO_STOP)
    {
      countToStop--;
      Serial.print("count = ");
      Serial.println(countToStop);
      if(countToStop == 0)
      {
        finishedRacing = true;
        car.stop();
      }
      onWhite_cnt = 0;
    }

    oldOffset = offset;
    t2_cnt = currentTime;
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
