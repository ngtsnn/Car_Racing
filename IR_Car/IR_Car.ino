#include "IR_Car.h"
#include <PID_v1.h>

#define MAX_ANGLE 60
#define KP        0.6
#define KI        0.05
#define KD        0

#define LOOP_TIME    50
#define TIME_TO_STOP 500

IRCar car;

double offset, setPoint = 0, angleOffset, oldOffset;
PID myPID(&offset, &angleOffset, &setPoint, KP, KI, KD, P_ON_M, DIRECT);
float oldAngle = 0;


int speed = 100;
  
unsigned int *val;
unsigned int threshold[7] = {30, 30, 30, 30, 30, 30, 30};

void Show_Leds(void) {
  car.IRLed_GetAllAnalog();
  uint8_t filted = car.IRLed_GetAllFilted();
  car.Led_Display(filted);
}

float smoothDampAngle(double oldAngleOffset, double angleOffset, float t){
  double deltaAngel = angleOffset - oldAngleOffset;
  return oldAngleOffset + deltaAngel * t;
}

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

  car.Run(speed, speed);
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
      case 0b1111111: //full white
        offset = 0;
        onWhite_cnt += currentTime - t1_cnt;

        if(millis() - onWhite_cnt >= TIME_TO_STOP){
          car.Run(0, 0);
        }
        //car.Run(0,0);
        break;
      case 0b0000000: //full black
        offset = 0;
        car.Run(0,0);
        break;
      default:
        offset = oldOffset;
    }
    //Serial.println(offset, 1);
    myPID.Compute();
    float newAngle = smoothDampAngle(oldAngle, angleOffset, .7);
    oldAngle = newAngle;
    Serial.println(angleOffset, 1);
    car.Turn((int16_t)newAngle);


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
