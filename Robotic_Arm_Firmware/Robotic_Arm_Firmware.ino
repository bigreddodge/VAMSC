/** ------------------------------------------------------------------------------------------------------------------------------------------- */
/*
 * Firmware:      Robotic Arm Exhibit v1.1.2
 * Description:   Cylindrical-coordinate control system
 * Location:      Virginia Beach Marine Science Center
 *                "Reaching Out for Clues" Exhibit
 * Author:        Imran A. Uddin
 *                Old Dominion University
 * Version:       1.1.2
 * Major:         20AUG2014
 * Minor:         20AUG2015
 * Revision:      22FEB2017
 *                Added comments, adjusted structure and format.
 *                No operational changes in this revision.
 * Includes:      Robotic_Arm_Firmware.ino (this file)
 *                joint.h
 *                joint.cpp
 *                coor.h
 *                coor.cpp
 * Verified IDEs: Arduino v1.6.3, v1.6.5
 */
/** ------------------------------------------------------------------------------------------------------------------------------------------- */

#include <math.h>
#include <Servo.h>
#include "joint.h"
#include "coor.h"
#include <string.h>

/** Constant Declarations */
    // Unit conversion multiplier constants
      static const double toRadians = M_PI / 180;   // DEG -> RAD
      static const double toDegrees = 180 / M_PI;   // RAD -> DEG
  

  /** --------------------------------------------------------------------------------------------------------------------------------------- */
  /** NOTE: All spatial references (left, right, etc.) are indicated observing the arm from the base toward the opposing wall of the exhibit. */
  /** vvvvvvvvvv     BEGIN ADJUSTABLE PARAMETERS     vvvvvvvvvvv */
  
    // Exhibit boundaries
      static const double LBOUND = -16;             // Sets the leftmost claw range in inches from center
      static const double RBOUND = 16;              // Sets the rightmost claw range in inches from center
      static const double FBOUND = 40;              // Sets the forwardmost claw range in inches from center
      static const double FLOORBOUND = 12;          // Sets the lowest claw range in incheas from center
    
    // Initial coordinates
      static const double rinit = 14;               // Initial claw radius
      static const double tinit = M_PI / 2;         // Initial rotation (pi/2 = 90* = centered)
      static const double zinit = 3;                // Initial height

    /** The remaining parameters below are modifiable, but modification is NOT RECOMMENDED without full calculations. */
    // Mechanical boundaries & parameters
      static const int thetamin = 0 * toRadians;    // Sets the rightmost mechanical rotation limit
      static const int thetamax = 180 * toRadians;  // Sets the leftmost mechanical rotation limit
      static const int rmin = 12;                   // Sets the minimum distance from claw to origin
      static const int zmin = 0;                    /* Sets the vertical claw limit.
                                                     * MUST BE NON-NEGATIVE (Causes trig error -> erratic behavior). */
      static const double SOffset = 4;              // Linear offset from hShoulder to vShoulder rotational axes
      static const double Leg = 18;                 /* Symmetric length between servos - single channel length
                                                     * NOTE: Shoulder->Elbow & Elbow->Claw channels must be identical length. */

    // Paremetric velocity constants
      static const double rinc = 0.25;              // Radial increment size
      static const double thetainc = 0.0061086523;  // Rotational increment size
      static const double zinc = 0.2;               // Vertical increment size

    // Firmware execution constants
      static const unsigned long watchdog = 15000;  // Inactivity period for power-down. 15000 = 5 minutes
      static const int loopDelay = 1;               // Firmware execution delay constant
  
  /** ^^^^^^^^^^      END ADJUSTABLE PARAMETERS      ^^^^^^^^^^ */
  /** --------------------------------------------------------------------------------------------------------------------------------------- */


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
      static const double RSECTOR = atan2(FBOUND, RBOUND);
      static const double LSECTOR = atan2(FBOUND, LBOUND);
  
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
          PowerDown();                                // Power down arm
          do {in = GetInput();} while (!in);          // Wait for input
          PowerUp();                                  // Power up arm
        }
        delay(10);                                    // Process delay (also aides in debouncing inputs)
      } while (!in);                                  // End of input loop

  /** Process Input
   *  Modifies claw coordinates according to control input.
   *  These values are subject to modification by boundary checks (below) before the arm is moved.
   *  See Also: GetInput() function comments (below)
  */
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

  /** Check Boundaries
   * Compares commanded arm position to exhibit boundaries and physical limitations of the arm.
   * If a boundary conflict is found, the parameter which violates the boundary is modified to equal the boundary.
   */
   
    // Check Exhibit Boundaries
      if (Coor->Z() > FLOORBOUND)                                 // Floor Bound
        Coor->setZ(FLOORBOUND);
      if ((Coor->T() < RSECTOR) && (Coor->X() >= RBOUND))         // Right Boundary
        Coor->setX(RBOUND);
      else if ((Coor->T() > LSECTOR) && (Coor->X() <= LBOUND))    // Left Boundary
        Coor->setX(LBOUND);
      else if (Coor->Y() >= FBOUND)                               // Front Boundary
        Coor->setY(FBOUND);

    // Check Arm Mechanical Limitations
      if (Coor->T() < thetamin)                                   // Check rightmost mechanical rotation limit
        Coor->setT(thetamin);
      if (Coor->T() > thetamax)                                   // Check leftmost mechanical rotation limit
        Coor->setT(thetamax);
      if (Coor->R() >= (2 * Leg))                                 // Check for radial overextension
        Coor->setR((2 * Leg) - 1);
      if (Coor->R() < rmin)                                       // Check for overfolding of elbow
        Coor->setR(rmin);
      if (Coor->Z() < zmin)                                       // Check for vertical overextension
        Coor->setZ(zmin);

  /** Move arm to modified commanded position */
      UpdatePositions();
}

/** ------------------------------------------------------------------------------------------------------------------------------------------- */

/** UpdatePositions() Function
 * Calculates required servo commands.
 * Peforms final mathematical/boundary checks to ensure valid positioning
 * Moves servos to modified commanded position.
 * Also provides basic serial debugging output.
 */
void UpdatePositions() {
  // Calculate joint angles
    double elbow = 2 * acos(sqrt(pow(Coor->R(), 2) + pow(Coor->Z(), 2)) / (2 * Leg));   // elbow = arccos(root(r^2+z^2)/(2*Leg))
    if (isnan(elbow)) {                                                                 // Check for undefined result
      elbow = 0;                                                                        // Reset to real number
      Serial.println("NAN DETECTED");
    }
    
    double shoulder = (elbow / 2) + atan(Coor->R() / Coor->Z());                        // shoulder = (elbow/2)+arctan(r/z)
    if (isnan(shoulder)) {                                                              // Check for undefined result
      shoulder = 179 * toRadians;                                                       // Reset to real number
      Serial.println("NAN DETECTED");
    }
    
    elbow *= toDegrees;                                                                 // Convert elbow angle to degrees
    shoulder *= toDegrees;                                                              // Convert shoulder angle to degrees
    
   if (elbow > 125)                                                                     // Elbow mechanical hard-limit - Folding. (Added 8/20/2015)
     elbow = 125;
   Serial.println("Elbow: " + String(elbow) + "    Shoulder:" + String(shoulder));

  // Move servos
    hShoulder.Move(Coor->T() * toDegrees);
    vShoulderM.Move(shoulder);
    vShoulderS.Move(shoulder);
    ElbowM.Move(elbow);
    ElbowS.Move(elbow);
    Claw.Move(ClawState);
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
    delay(2000);
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

/** blinkLED() Function
 * Rapidly blinks the headlamp LED 3 times.
 */
void blinkLED(bool state) {
  for (int l = 0; l < 6; l++) {
    digitalWrite(LightPin, l % 2); 
    delay(100);
  }
  digitalWrite(LightPin, state);
}

