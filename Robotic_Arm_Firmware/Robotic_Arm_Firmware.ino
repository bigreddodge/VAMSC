/*
 * Firmware:      Robotic Arm Exhibit v1.1.1
 * Description:   Cylindrical-coordinate control system
 * Location:      Virginia Beach Marine Science Center
 *                "Reaching Out for Clues" Exhibit
 * Author:        Imran A. Uddin
 *                Old Dominion University
 * Version:       1.1.1
 * Major:         20AUG2014
 * Minor:         20AUG2015
 * Revision:      29JAN2016 (Header updated 9FEB2016)
 * Includes:      Robotic_Arm_Firmware.ino (this file)
 *                joint.h
 *                joint.cpp
 *                coor.h
 *                coor.cpp
 * Verified IDE:  Arduino v1.6.3, v1.6.5
 */

#include <math.h>
#include <Servo.h>
#include "joint.h"
#include "coor.h"
#include <string.h>

//Global Constants
  static const int Umin = 50;
  static const int Umax = 90;
  static const int Rmin = 110;
  static const int Rmax = 150;
  static const int Dmin = 240;
  static const int Dmax = 280;
  static const int Lmin = 490;
  static const int Lmax = 530;

  static const double toRadians = M_PI / 180;           //Conversion factors
  static const double toDegrees = 180 / M_PI;

/**** Adjustment parameters ****/
  
  static const double LBOUND = -16;
  static const double RBOUND = 16;
  static const double FBOUND = 40;
  static const double FLOORBOUND = 12;
  static const int thetamin = 0 * toRadians;
  static const int thetamax = 180 * toRadians;

/*******************************/
 
  static const double rinc = 0.25;
  static const double thetainc = 0.0061086523; //0.35;
  static const double zinc = 0.2;

  static const int rmin = 12; //0
  static const int zmin = 0;     //MINIMUM MUST BE NO LESS THAN 0
  
  static const int hSmax = 180;
  static const int vSmax = 90;
  static const int emax = 90;
  static const int loopDelay = 1;

  static const double Leg = 18;                         //Symmetrical length between servos
  static const double SOffset = 4;                      //Linear offset from hShoulder to vShoulder rotational axes
  static const double RSECTOR = atan2(FBOUND, RBOUND);  //Expressed in Radians
  static const double LSECTOR = atan2(FBOUND, LBOUND);

//Initial coordinates
  static const double rinit = 14;
  static const double tinit = M_PI / 2;
  static const double zinit = 3;

//Global Variables
  //double r, theta, z;
  int in, reset = 0;
  unsigned long timer;
  bool ClawState;
  
//Input assignments
  static const int Btn1 = 15;
  static const int Btn2 = 14;
  static const int ClawBtn = 16;
  static const int LightBtn = 3;

//Control assignments
  static const int hShoulderPin = 9;
  static const int vShoulderMPin = 8;
  static const int vShoulderSPin = 7;
  static const int ElbowMPin = 6;
  static const int ElbowSPin = 5;
  static const int ClawPin = 4;
  static const int LightPin = 2;

//Servo declarations        umin, umax, amin, amax , SS, pinconst
  joint hShoulder  =  joint(1380, 1620,    0, hSmax,  0, hShoulderPin);    //Need
  joint vShoulderM =  joint(1860, 1420,    0, vSmax,  0, vShoulderMPin);   //to
  joint vShoulderS =  joint(1860, 1420,    0, vSmax,  1, vShoulderSPin);   //fill
  joint ElbowM     =  joint(1490, 1615,    0,  emax,  0, ElbowMPin);       //these
  joint ElbowS     =  joint(1490, 1615,    0,  emax,  1, ElbowSPin);       //in
  joint Claw       =  joint(2250,  700,    0,     1,  0, ClawPin);         // v1: 2100/0 = Closed; 725/1 = Open,
                                                                           // v2: 2300/0 = Closed; 600/1 = Open,
                                                                           // v3: 2250/0 = Closed; 700/1 = Open,

//Coordinate class implementation
coor* Coor = new coor(rinit, tinit, zinit);


void setup()
{
  //Data direction assignments
    pinMode(Btn1, INPUT);     //vertical down.
    pinMode(Btn2, INPUT);     //vertical up.
    pinMode(ClawBtn, INPUT);  //claw control
    pinMode(LightBtn, INPUT); //button input
    pinMode(LightPin, OUTPUT);//Light Signal
  Serial.begin(9600);
  PowerUp();
}

void loop()
{
  delay(loopDelay);  //Controls program execution speed
  
  //Watchdog timer
    timer = 0;
    do {in = GetInput();
      if (++timer >= 15000) //15000 = 5 Minutes
      {
        Serial.println("Power Down...");
        PowerDown();
        do {in = GetInput();} while (!in);
        PowerUp();
      }
    //Serial.println(timer);
    delay(10);
    } while (!in);    
  //End timer code

  //Process Input
    if (in == 1)
      Coor->setR(Coor->R() + rinc);
      //r += rinc;     //increment radius
    if (in == 2)
      Coor->setR(Coor->R() - rinc);
      //r -= rinc;     //decrement radius
    if (in == 3)
      Coor->setT(Coor->T() + thetainc);
      //theta += thetainc; //increment theta
    if (in == 4)
      Coor->setT(Coor->T() - thetainc);
      //theta -= thetainc; //decrement theta
    if (in == 5)
      Coor->setZ(Coor->Z() - zinc);
      //z -= zinc;     //decrement z
    if (in == 6)
      Coor->setZ(Coor->Z() + zinc);
      //z += zinc;     //increment z

  //Check Boundaries
    //Floor Bound
    if (Coor->Z() > FLOORBOUND)
      Coor->setZ(FLOORBOUND);

//Check Boundaries
if ((Coor->T() < RSECTOR) && (Coor->X() >= RBOUND))  //Right Boundary
  Coor->setX(RBOUND);
else if ((Coor->T() > LSECTOR) && (Coor->X() <= LBOUND))  //Left Boundary
  Coor->setX(LBOUND);
else if (Coor->Y() >= FBOUND)  //Front Boundary
  Coor->setY(FBOUND);

    //Mechanical Limitations
    if (Coor->T() < thetamin)
      Coor->setT(thetamin);
    if (Coor->T() > thetamax)
      Coor->setT(thetamax);
    if (Coor->R() >= (2 * Leg))
      Coor->setR((2 * Leg) - 1);
    if (Coor->R() < rmin)
      Coor->setR(rmin);
    if (Coor->Z() < zmin)
      Coor->setZ(zmin);

  UpdatePositions();
}

void UpdatePositions()
{
  //Calculate Angles 
    double elbow = 2 * acos(sqrt(pow(Coor->R(), 2) + pow(Coor->Z(), 2)) / (2 * Leg));
    if (isnan(elbow))
    {
      elbow = 0;
      Serial.println("NAN DETECTED");
    }
    double shoulder = (elbow / 2) + atan(Coor->R() / Coor->Z());
    if (isnan(shoulder))
    {
      shoulder = 179 * toRadians;
      Serial.println("NAN DETECTED");
    }
    elbow *= toDegrees;
    shoulder *= toDegrees;
// Angle Limitations (Added 8/20/2015)

   if (elbow > 125)
     elbow = 125;
   
    Serial.println("Elbow: " + String(elbow) + "    Shoulder:" + String(shoulder));
  //Move Servos
    hShoulder.Move(Coor->T() * toDegrees);
    vShoulderM.Move(shoulder);
    vShoulderS.Move(shoulder);
    ElbowM.Move(elbow);
    ElbowS.Move(elbow);
    Claw.Move(ClawState);
}

void PowerUp()
{
  //Power Up
  hShoulder.Start();
  vShoulderM.Start();
  vShoulderS.Start();
  ElbowM.Start();
  ElbowS.Start();
  Claw.Start();
  //Move to Starting Position

  Coor->setR(rinit);
  Coor->setT(tinit);
  Coor->setZ(zinit);  

    ClawState = false;
  reset = 0;
  UpdatePositions();
  delay(2000);
  blinkLED(false);
}

void PowerDown()
{
  //Move to Rest Position
  Coor->setR(rinit);
  Coor->setT(tinit);
  Coor->setZ(FLOORBOUND);  
  UpdatePositions();  
  delay(2000);
  //Power Down
  hShoulder.Stop();
  vShoulderM.Stop();
  vShoulderS.Stop();
  ElbowM.Stop();
  ElbowS.Stop();
  Claw.Stop();
  blinkLED(false);
}


//Reads control board and returns:
//1 = forward, 2 = back, 3 = left, 4 = right, 5 = btn1, 6 = btn2
//Prevents more than one control from operating at a time.
//Toggles headlight.  Handles Claw movements
int GetInput()
{
  //Claw
  if (digitalRead(ClawBtn) != ClawState)
  {
    ClawState = !ClawState;
    return 9;
  }
  int input = analogRead(A0);
  delay(10);
  if ((abs(analogRead(A0) - input)/input) < 0.1)
  {
  if (input >= Umin && input <= Umax)
    return 1;
  if (input >= Dmin && input <= Dmax)
    return 2;
  if (input >= Lmin && input <= Lmax)
    return 3;
  if (input >= Rmin && input <= Rmax)
    return 4;
  if (digitalRead(Btn1))
    return 5;
  if (digitalRead(Btn2))
    return 6;
  }
  if (digitalRead(LightBtn))
  {
    delay(10);
    if (digitalRead(LightBtn))
    {
      digitalWrite(LightPin, !digitalRead(LightPin));
      while(digitalRead(LightBtn))
      {
        ++reset;
        Serial.println(reset);
        if (reset >= 12000)
        {
          PowerDown();
          delay(2000);
          PowerUp();
          break;
        }
      }
      reset = 0;
      return 7;  
    }
  }
  return 0;
}

void blinkLED(bool state)
{
  for (int l = 0; l < 6; l++)
  {
    digitalWrite(LightPin, l % 2); 
    delay(100);
  }
  digitalWrite(LightPin, state);
}

