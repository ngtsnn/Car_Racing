#include "IR_Car.h"
#include <PID_v1.h>

#define MAX_ANGLE 60
#define KP        0.55
#define KI        0.005
#define KD        0.005

#define LOOP_TIME       40
#define TIME_TO_STOP    400
#define ACCEL           10
#define STEERING_ACCEL  15
#define SPEEDRATIO      0.15
#define DEFINED_READY_TIME 250

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
bool preReady = false;
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
unsigned int threshold[7] = {600, 600,  600,  600,  600,  600,  600};

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
      Serial.println("find obstacle!");
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
      
      car.run(120, 60);
      //steer right
      for(int angle = 0; angle <= 60; angle += 5){
        car.turn(angle);
        delay(30);        
      }
      delay(100);
      car.run(80, 80);
      //go straight
      car.turn(0);

      //catch middle line
      while(1)
      {
        if(readSensor() & 0b0011000)
        {
          break; 
        }
        delay(10);
      }

      baseSpeed = 120;
      car.run(baseSpeed, baseSpeed);
      preReady = ready = isSwitching = isTurning = passedFullWhite = false;
      dir = side = NONE;
      Serial.println("done");
    }
    
    sensor = readSensor();
    obstacle = car.getDistance();
    //Get time when we got Ready Signal
    if(preReady)
    {
      if(readyInterval >= DEFINED_READY_TIME)
      {
        preReady = false;
        ready = true;
        readyInterval = 0;
      }
      else
      {
        readyInterval += dt; 
      }
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
       if(ready)
        {
          Serial.println("turn left!");
          isTurning = true;
          isSwitching = false;
          
        }
        else 
        {
          Serial.println("Pready from right!");
          preReady = true;
          if(passedFullWhite)
          {
            ready = true;
            dir = NONE;
          }
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
        if(ready)
        {
          Serial.println("turn right!");
          isTurning = true;
          isSwitching = false;
          
        }
        else 
        {
          Serial.println("Pready from right!");
          preReady = true;
          if(passedFullWhite)
          {
            ready = true;
            dir = NONE;
          }
          isTurning = false;
          isSwitching = false;
        }

        break;
      }
      case 0b1111111: //full white
        onWhite_cnt += dt;
        isSwitching = false;
        offset = 0;

        if(ready)
        {
          if(dir != NONE)
          {
            Serial.println("turn from direction!");
            isTurning = true;
          }
        }
        else // just received ready signal
        {
          Serial.println("Pready from White!");
          isTurning = false;
          preReady = true;
          passedFullWhite = true;
          dir = NONE;
        }
        break;
      case 0b0000000: //full black
        offset = 0;
        isTurning = false;

        if(ready)
        {
          if(dir != NONE)
          {
            Serial.println("switch from direction!");
            isSwitching = true;
          }
          else
          {
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
    if (isSwitching || isTurning || isLosingLine)   
    {
      if (isSwitching && dir != NONE)
      {
        baseSpeed = 80;
        car.turn((int16_t)40 * dir);        
        car.run(baseSpeed + dir*50, baseSpeed - dir*50);
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
        
        ready = isSwitching = isTurning = passedFullWhite = false;
        dir = side = NONE;
      }


      //LOSE LINE
      else if(isLosingLine){
        for (int angle = 0; angle >= -30; angle -= 5){
          car.turn(angle);
          delay(30);
        }
        car.turn(0);
        while(!(readSensor() & 0b1100000))
        {
          delay(10);
        }
        car.run(180, 100);
        for (int angle = 0; angle <= 30; angle += 5){
          car.turn(angle);
          delay(30);
        }
        delay(160);
        car.run(120, 120);
        car.turn(0);
        while(!(readSensor() & 0b0011100)){
          delay(10);
        }
        car.turn(-15);
        delay(100);
        car.turn(0);
        ready = isSwitching = isTurning = passedFullWhite = false;
        dir = side = NONE;
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

        car.turn(40 * dir);
        if(dir == LEFT)
        {
          car.run(-170, 210);
          delay(500);
        }
        else
        {
          car.run(200, -180);
          delay(400);
        }
        //delay(600);
        
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
        delay(350);
        car.run(baseSpeed - 40*dir, baseSpeed + 40*dir);
        car.turn(0);
        delay(200);
        
        car.run(baseSpeed, baseSpeed);
        car.turn(0);
        
        ready = isSwitching = isTurning = passedFullWhite = false;
        dir = side = NONE;
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
        baseSpeed = 50;
      }
      else if(sensor & 0b1100011)
      {
        baseSpeed -= 5;
        baseSpeed = baseSpeed < 120 ? 120: baseSpeed;
      }
      else
      {
        baseSpeed += 3;
        baseSpeed = baseSpeed > 140 ? 140: baseSpeed;
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
