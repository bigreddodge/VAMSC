/** ------------------------------------------------------------------------------------------------------------------------------------------- */
/*
 * Firmware:      Robotic Arm Exhibit v1.1.3
 * Description:   Cylindrical-coordinate control system
 * Location:      Virginia Beach Marine Science Center
 *                "Reaching Out for Clues" Exhibit
 * Author:        Imran A. Uddin
 *                Old Dominion University
 * Version:       1.1.3
 * Major:         20AUG2014
 * Minor:         20AUG2015
 * Revision:      02APR2017
 *                Parameter and operational modifications
 * Includes:      Robotic_Arm_Firmware.ino (this file)
 *                joint.h
 *                joint.cpp
 *                coor.h
 *                coor.cpp
 * Verified IDEs: v1.6.5
 */
/** ------------------------------------------------------------------------------------------------------------------------------------------- */

#include <math.h>
#include <Servo.h>
#include "joint.h"
#include "coor.h"
#include <string.h>

/** Global Constant Declarations */
    // Unit conversion multiplier constants
      static const double toRadians = M_PI / 180;   // DEG -> RAD
      static const double toDegrees = 180 / M_PI;   // RAD -> DEG
  

  /** --------------------------------------------------------------------------------------------------------------------------------------- */
  /** NOTE: All spatial references (left, right, etc.) are indicated observing the arm from the base toward the opposing wall of the exhibit. */
  /** vvvvvvvvvv     BEGIN ADJUSTABLE PARAMETERS     vvvvvvvvvvv */
  
    // Exhibit boundaries - Dimensions 45(F)x38(L+R)x16(FLOOR)
      static const double LBOUND = -22;             // Sets the leftmost claw range in inches from center
      static const double RBOUND = 16;              // Sets the rightmost claw range in inches from center
      static const double FBOUND = 45;              // Sets the forwardmost claw range in inches from center
      static const double FLOORBOUND = 16;          // Sets the lowest claw range in inches from center
      static const double PADDING = 1;              // Spatial buffer for all boundaries
    
    // Initial coordinates
      static const double rinit = 18;               // Initial claw radius
      static const double tinit = M_PI / 2;         // Initial rotation (pi/2 = 90* = centered)
      static const double zinit = 3;                // Initial height

    /** The remaining parameters below are modifiable, but modification is NOT RECOMMENDED without full calculations. */
    // Mechanical boundaries & parameters
      static const double AxesOffset = 3;           // Linear offset from hShoulder to vShoulder rotational axes
      static const double MountOffset = 3.75;       // Linear offset from mount wall to horizontal rotation axis
      static const double Leg1 = 18;                // ShoulderChannels are no longer symmetric
      static const double Leg2 = 14;
      static const int thetamin = 0 * toRadians;    // Sets the rightmost mechanical rotation limit
      static const int thetamax = 180 * toRadians;  // Sets the leftmost mechanical rotation limit
      static const double rmin = AxesOffset + 1;       // Sets the minimum distance from claw to origin
      static const double zmin = 0.0;               /* Sets the vertical claw limit.
                                                     * MUST BE NON-NEGATIVE (Causes trig error -> erratic behavior). */

      static const double ElbowMinAngle = 45.0;     // Elbow folding mechanical limit.
      
    /** Linear Servo Parameters */
      static const double AHVO = 3;                 // Actuator Head Vertical Offset
      static const double AHRO = 1;                 // Actuator Head Radial Offset
      static const double ARVO = 2;                 // Actuator Rod Vertical Offset
      static const double ARRO = 4.5;               // Actuator Rod Radial Offset

      static const double ActMinPos = 12;           // Actuator length at minimum extension (within mechanical limits)
      static const double ActMaxPos = 16;           // Actuator length at maximum extension (within mechanical limits) 
      static const double ActMinCmd = 145;          // Actuator command corresponding to minimum extension (determined empirically)
      static const double ActMaxCmd = 20;           // Actuator command corresponding to maximum extension (determined empirically)

    // Paremetric velocity constants
      static const double rinc = 0.25;              // Radial increment size
      static const double thetainc = M_PI / 128;    // Rotational increment size
      static const double zinc = 0.1;               // Vertical increment size

    // Firmware execution constants
      static const unsigned long watchdog = 15000;  // Inactivity period for power-down. 15000 = 5 minutes
      static const int loopDelay = 1;               // Firmware execution delay constant
  
  /** ^^^^^^^^^^      END ADJUSTABLE PARAMETERS      ^^^^^^^^^^ */
  /** --------------------------------------------------------------------------------------------------------------------------------------- */

    // Operating parameters determined by hardware limitations
      static const double AMHG = sqrt(pow(AHVO,2) + pow(AHRO,2));   // Actuator Mount (Head-Side) Geometry (Hypotenuse = sqrt(10))
      static const double AMRG = sqrt(pow(ARVO,2) + pow((Leg1 - ARRO),2));    // Actuator Mount (Rod-Side) Geometry (Hypotenuse = sqrt(186.25))
      static const double MountComponentAngle = atan(AHVO/AHRO) * toDegrees;  //
      static const double LegComponentAngle = atan(ARVO/(Leg1 - ARRO)) * toDegrees;

      static const double ActPosRange = abs(ActMaxPos - ActMinPos);       // 4.0" in current configuration
      static const double ActCmdRange = abs(ActMaxCmd - ActMinCmd);       // 125.0 in current configuration
    
  /** These constants define the mininum and maximum acceptable analogRead() values produced
   *  by operating the joystick on the control board.
   *  They were determined by repeated trials and modified to provide reasonable thresholds.
   */
    // Joystick: UP -- analog 50-90
      static const int Umin = 50;
      static const int Umax = 90;
    // Joystick: RIGHT -- analog 110-150
      static const int Rmin = 110;
      static const int Rmax = 150;
    // Joystick: DOWN -- analog 240-280
      static const int Dmin = 240;
      static const int Dmax = 280;
    // Joystick: LEFT -- analog 490-530
      static const int Lmin = 490;
      static const int Lmax = 530;

      static const int hSmax = 180;
      static const int vSmax = 90;
      static const int emax = 90;

    /** Sector boundaries provide rotational thresholds which help
     *  determine proper arm behavior when commanding a coordinate variable
     *  causes a boundary violation in a different coordinate variable.
     *  These thresholds are expressed in Radians.
     */
      //static const double RSECTOR = atan2(FBOUND - MountOffset, RBOUND);
      //static const double LSECTOR = atan2(FBOUND - MountOffset, LBOUND);
  
  /** Hardware digital I/O pin assignments */
    // Control Inputs
      static const int Btn1 = 15;
      static const int Btn2 = 14;
      static const int ClawBtn = 16;
      static const int LightBtn = 3;
    // Arm Outputs
      static const int hShoulderPin = 9;
      static const int vShoulderMPin = 8;
      static const int vShoulderSPin = 7;
      static const int ElbowMPin = 6;
      static const int ElbowSPin = 5;
      static const int ClawPin = 4;
      static const int LightPin = 2;

  /** Objects */
    /** Servo objects: See joint.h for description.
     *      identifier          umin, umax, amin, amax , SS, Hardware Pin Constant  */
      joint hShoulder  =  joint(1380, 1620,    0, hSmax,  0, hShoulderPin);         // Each declaration constructs an instance of the joint object.
      joint vShoulderM =  joint(1860, 1420,    0, vSmax,  0, vShoulderMPin);        // Each joint object contains properties and functions for a servo motor
      joint vShoulderS =  joint(1860, 1420,    0, vSmax,  1, vShoulderSPin);        // SS: Slave Select -- setting this property to '1' indicates the servo
      joint ElbowM     =  joint(1490, 1615,    0,  emax,  0, ElbowMPin);            //     operates reverse tandem from its adjoining master servo.
      joint ElbowS     =  joint(1490, 1615,    0,  emax,  1, ElbowSPin);            // Physical disuse of a slave servo has no effect on firmware operation.
      joint Claw       =  joint(2250,  700,    0,     1,  0, ClawPin);              // v1: 2100/0 = Closed; 725/1 = Open,
                                                                                    // v2: 2300/0 = Closed; 600/1 = Open,
                                                                                    // v3: 2250/0 = Closed; 700/1 = Open,
    /** Coordinate object: See coor.h for description. */
      coor* Coor = new coor(rinit, tinit, zinit);

/** Global Variable Declarations */
      int in, reset = 0;
      unsigned long timer;
      bool ClawState;
//      double lastElbow, lastShoulder, lastR, lastT, lastZ = 0;
/** ------------------------------------------------------------------------------------------------------------------------------------------- */


/** setup() Function
 * Arduino Initialization
 */
void setup() {
  // Data direction (I/O) assignments
    pinMode(Btn1, INPUT);     //vertical down.
    pinMode(Btn2, INPUT);     //vertical up.
    pinMode(ClawBtn, INPUT);  //claw control
    pinMode(LightBtn, INPUT); //button input
    pinMode(LightPin, OUTPUT);//Light Signal

  // Enable serial debugging messages
    Serial.begin(9600);
  
  // Enable Arm
    PowerUp();
}

/** loop() Function
 * Arduino Process
 */
void loop() {
  delay(loopDelay);                                   // Controls program execution speed
  
  /** Watchdog Timer
   *  When controls are inactive for time set by "watchdog" constant, the arm
   *  powers down and waits in an internal loop until an input is received.
  */
      timer = 0;                                      // Reset timer
      do {                                            // Begin input loop
        in = GetInput();                              // Check for input (also records input for processing)
        if (++timer >= watchdog) {                    // Increment timer and check for timeout
          Serial.println("Power Down...");
          PowerUp();                                  // Power up arm (reset to home position)
          do {in = GetInput();} while (!in);          // Wait for input
        }
        delay(10);                                    // Process delay (also aides in debouncing inputs)
      } while (!in);                                  // End of input loop
      
  /** Process Input
   *  Modifies claw coordinates according to control input.
   *  These values are subject to modification by boundary checks (below) before the arm is m
   *  See Also: GetInput() function comments (below)
  */
      //lastR = Coor->R();
      //lastT = Coor->T();
      //lastZ = Coor->Z();
      if (in == 1)
        Coor->setR(Coor->R() + rinc);                 // increment radius (Extend Out)
      if (in == 2)
        Coor->setR(Coor->R() - rinc);                 // decrement radius (Retract In)
      if (in == 3)
        Coor->setT(Coor->T() + thetainc);             // increment theta (Rotate Left)
      if (in == 4)
        Coor->setT(Coor->T() - thetainc);             // decrement theta (Rotate Right)
      if (in == 5)
        Coor->setZ(Coor->Z() - zinc);                 // decrement z (Retract Up)
      if (in == 6)
        Coor->setZ(Coor->Z() + zinc);                 // increment z (Extend Down)

  /** Check Claw Position Boundaries
   * Compares commanded claw position to exhibit boundaries and physical limitations of the arm.
   * If a boundary conflict is found, the parameter which violates the boundary is modified to equal the boundary,
   * which may cause other parameters to change such that all boundary limits are satisfied.
   */

    /** Exhibit boundary check.
     *  If the arm is commanded to exceed the boundary, the arm will stop.
     */
      if (Coor->Z() > FLOORBOUND)                                 // Check floor boundary
        Coor->setZ(FLOORBOUND);
      if (Coor->Z() < zmin)                                       // Check vertical height boundary
        Coor->setZ(zmin);
      if (Coor->T() < thetamin)                                   // Check rightmost mechanical rotation limit
        Coor->setT(thetamin);
      if (Coor->T() > thetamax)                                   // Check leftmost mechanical rotation limit
        Coor->setT(thetamax);
      if (Coor->R() < rmin)                                       // Check for overfolding of elbow
        Coor->setR(rmin);

    /** Left/Right boundary checks. Modifies Cartesian X to allow continued arm movement.
     *  If the arm is commanded to exceed a boundary, the X Cartesian component is held at the boundary,
     *  allowing the Y Cartesian component to continue movement.
     */
      if ((Coor->T() > (90 * toRadians)) && (Coor->X() <= (LBOUND + PADDING)))
        Coor->setX(LBOUND + PADDING);
      if ((Coor->T() < (90 * toRadians)) && (Coor->X() >= (RBOUND - PADDING)))
        Coor->setX(RBOUND - PADDING);
        
    /** Front boundary check. (Never exceeded in current configuration).
     *  Modifies Cartesian Y to the boundary, allowing continued arm movement.
     */
      if (Coor->Y() > (FBOUND - (MountOffset + PADDING)))
        Coor->setY(FBOUND - (MountOffset + PADDING));
    
    /** Check full-extension mechanical limitation.
     *  Prevents the arm from being fully extended (linear).
     */
      double ClawVector = PythagH(Coor->R() - AxesOffset, Coor->Z());
      if (ClawVector > (Leg1 + Leg2 - 1))         // Check for radial overextension
        Coor->setR(PythagL((Leg1 + Leg2 - 1), Coor->Z()) + AxesOffset);

    /** Check full-folding mechanical limitation. 
     *  Limit the radius by calculating the minimum distance from the
     *  vertical rotation axis to the claw according to the minimum elbow angle.
     */
      double minClawVector = LoCOppSide(ElbowMinAngle, Leg1, Leg2);
      if (ClawVector < minClawVector)
        Coor->setR(PythagL(minClawVector, Coor->Z()) + AxesOffset);
  
  /** Move arm to modified commanded position */
      UpdatePositions();
}

/** ------------------------------------------------------------------------------------------------------------------------------------------- */

/** UpdatePositions() Function
 * Calculates required servo commands.
 * Peforms final mathematical/boundary checks to ensure valid positioning
 * Moves servos to 
 * modified commanded position.
 * Also provides basic serial debugging output.
 */
void UpdatePositions() {
  /* Redundant. Remove.
  // Calculate joint angles (updated 02APR2017 using Law of Cosines) 
    double RZHyp = sqrt(pow(Coor->R() - AxesOffset, 2) + pow(Coor->Z(), 2));                         // Find imaginary hypotenuse of claw coordinate
    if (RZHyp > (Leg1 + Leg2 - 1)){
      RZHyp = Leg1 + Leg2 - 1;
      Coor->setR(lastR);
      Coor->setZ(lastZ);
    }
    */
    
    // Serial.println("RZHyp: " + String(RZHyp) + "   R: " + String(Coor->R()) + "   Z: " + String(Coor->Z()));
    
    
    // Calculate vertical rotation axis to claw distance.
      double ClawVector = PythagH(Coor->R() - AxesOffset, Coor->Z());
    
    /** Calculate elbow angle using Law of Cosines.
     *  
     */
      // double elbow = acos((pow(Leg1, 2) + pow(Leg2, 2) - pow(RZHyp,2)) / (2 * Leg1 * Leg2));
      double elbow = LoCAngleDEG(Leg1, Leg2, ClawVector);

    /** Shoulder angle is the sum of two component angles formed by the arm.
     *  
     */
      //double shoulder = asin((Coor->R() - AxesOffset) / RZHyp) + acos((pow(RZHyp, 2) + pow(Leg1, 2) - pow(Leg2, 2))/(2 * RZHyp * Leg1));
      double shoulder = (asin((Coor->R() - AxesOffset) / ClawVector) * toDegrees) + LoCAngleDEG(ClawVector, Leg1, Leg2);
    
    /*
    elbow *= toDegrees;                                                                 // Convert elbow angle to degrees
    shoulder *= toDegrees;                                                              // Convert shoulder angle to degrees
    if (isnan(elbow)){
      elbow = lastElbow;
    }
    if (isnan(shoulder)){
      shoulder = lastShoulder;
    }
    */
    
    /*if (isnan(elbow) || isnan(shoulder)) {                                           // Check for undefined result
      Serial.print("Undefined angle commanded. Shoulder: ");
      Serial.print(shoulder);
      Serial.print("   Elbow: ");
      Serial.println(elbow);
      return;
    }*/
    
   /*if (elbow < ElbowMinAngle){                                                       // Elbow mechanical hard-limit - Folding. (Added 8/20/2015)
     elbow = ElbowMinAngle;
   }*/

   Serial.println("(r,t,z): (" + String(Coor->R()) + "," + String(Coor->T()) + "," + String(Coor->Z()) + ")   " + "Elbow: " + String(elbow) + "   Shoulder: " + String(shoulder));
  
  // Linear actuator positioning
      double ActCmd = actuatorCommand(shoulder);
  //lastShoulder = shoulder;
  //lastElbow = elbow;
  
  // Move servos
    hShoulder.Move(Coor->T() * toDegrees);
    if (ActCmd != -1) {vShoulderM.Move(ActCmd);}
    // vShoulderM.Move(shoulder);
    // vShoulderS.Move(shoulder);
    ElbowM.Move(180 - elbow);
    ElbowS.Move(180 - elbow);
    Claw.Move(ClawState);
}


/** actuatorCommand() Function
 *  Converts the specified angleDEG shoulder angle to a joint() object compatible linear actuator command
 *  based on the hardware configuration geometry defined/calculated in the global constants section.
 */
double actuatorCommand(double angleDEG){
  double ActuatorComponentAngle = 360 - (MountComponentAngle + LegComponentAngle + 90 + angleDEG);
  
  //double ActuatorLength = sqrt(pow(AMHG,2) + pow(AMRG,2) - (2 * AMHG * AMRG * cos(ActuatorComponentAngle * toRadians)));
  double ActuatorLength = LoCOppSide(ActuatorComponentAngle, AMHG, AMRG);
  
  double ActuatorCommand = ((ActMaxPos - ActuatorLength) * (ActCmdRange / ActPosRange)) + ActMaxCmd;
  
  if (ActuatorLength < ActMinPos) {
    Serial.print("\nActuator min-extension boundary violation!");
    return ActMinCmd;
  }
  else if (ActuatorLength > ActMaxPos) {
    Serial.print("\nActuator max-extension boundary violation!");
    return ActMaxPos;
  }
  else {
    return ActuatorCommand;
  }
}


/** --------------------------------------------------------------------------------------------------------------------------------------- */
/** Trig Functions */

double LoCAngleDEG(double Adj1, double Adj2, double Opp){
  return (acos((pow(Adj1, 2) + pow(Adj2, 2) - pow(Opp, 2)) / (2 * Adj1 * Adj2)) * toDegrees);
}

double LoCOppSide(double AngleDEG, double Adj1, double Adj2){
  return sqrt(pow(Adj1, 2) + pow(Adj2, 2) - (2 * cos(AngleDEG * toRadians) * Adj1 * Adj2));
}

double PythagH(double L1, double L2){
  return sqrt(pow(L1, 2) + pow(L2, 2));
}

double PythagL(double H, double L){
  return sqrt(pow(H, 2) - pow(L, 2));
}

/** --------------------------------------------------------------------------------------------------------------------------------------- */
/** Control Functions */


/** GetInput() Function
 * Reads control board and returns:
 * 1 = forward, 2 = back, 3 = left, 4 = right, 5 = btn1, 6 = btn2
 * Prevents more than one control from operating at a time.
 * Toggles headlight.  Handles Claw movements.
 */
int GetInput() {
  /** Only modify claw state when claw button status changes */
  if (digitalRead(ClawBtn) != ClawState) {
    ClawState = !ClawState;
    return 9;
  }

  /** Read value on Analog Pin 0 and determine control command.
   *  Debounces input by checking input again for <10% error after 10ms */ 
  int input = analogRead(A0);
  delay(10);
  if ((abs(analogRead(A0) - input)/input) < 0.1) {
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
  /** Holding headlight control button for 3 seconds initiates a reset sequence. */
  if (digitalRead(LightBtn)) {
    delay(10);
    if (digitalRead(LightBtn)) {
      digitalWrite(LightPin, !digitalRead(LightPin));
      while(digitalRead(LightBtn)) {
        ++reset;
        Serial.println(reset);
        if (reset >= 12000) {
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

/** PowerUp() Function
 * Enables control of arm servos, energizing them to starting positions.
 */
void PowerUp() {
  // Power-up
    hShoulder.Start();
    vShoulderM.Start();
    vShoulderS.Start();
    ElbowM.Start();
    ElbowS.Start();
    Claw.Start();

  // Move to starting position
    Coor->setR(rinit);
    Coor->setT(tinit);
    Coor->setZ(zinit);  
    ClawState = false;
    reset = 0;
    UpdatePositions();
    //delay(2000);
    blinkLED(false);
}

/** PowerDown() Function
 * Disables control of arm servos, deenergizing them after positioning the arm near the exhibit floor.
 */
void PowerDown()
{
  // Move to rest position
    Coor->setR(rinit);
    Coor->setT(tinit);
    Coor->setZ(FLOORBOUND);  
    UpdatePositions();  
    delay(2000);
  // Power-down
    hShoulder.Stop();
    vShoulderM.Stop();
    vShoulderS.Stop();
    ElbowM.Stop();
    ElbowS.Stop();
    Claw.Stop();
    blinkLED(false);
}

/** blinkLED() Function
 * Rapidly blinks the headlamp LED 3 times.
 * "state" determines final lamp state.
 */
void blinkLED(bool state) {
  for (int l = 0; l < 6; l++) {
    digitalWrite(LightPin, l % 2); 
    delay(100);
  }
  digitalWrite(LightPin, state);
}

