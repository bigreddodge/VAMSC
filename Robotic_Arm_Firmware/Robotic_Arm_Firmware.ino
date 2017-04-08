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

/** Global Function Prototypes */
double LoCAngleDEG(double Adj1, double Adj2, double Opp);
double LoCOppSide(double AngleDEG, double Adj1, double Adj2);
double PythagH(double L1, double L2);
double PythagL(double H, double L);

/** Global Constant Declarations */
    // Unit conversion multiplier constants
      static const double toRadians = M_PI / 180;   // DEG -> RAD
      static const double toDegrees = 180 / M_PI;   // RAD -> DEG
  
  /** --------------------------------------------------------------------------------------------------------------------------------------- */
  /** NOTE: All spatial references (left, right, etc.) are indicated observing the arm from the base toward the opposing wall of the exhibit. */
  /** vvvvvvvvvv     BEGIN ADJUSTABLE PARAMETERS     vvvvvvvvvvv */
  
    // Exhibit boundaries - Dimensions 45(F)x38(L+R)x16(FLOOR)
      static const double LBOUND = -22;             // Sets the leftmost (negative) claw range in inches from center
      static const double RBOUND = 16;              // Sets the rightmost (positive) claw range in inches from center
      static const double FBOUND = 45;              // Sets the forwardmost claw range in inches from center
      static const double FLOORBOUND = 16;          // Sets the lowest claw range in inches from center
      static const double PADDING = 1;              // Spatial buffer for all boundaries
    
    // Initial coordinates
      static const double rinit = 21;               // Initial claw radius
      static const double tinit = M_PI / 2;         // Initial rotation (pi/2 = 90* = centered)
      static const double zinit = 1;                // Initial height

    /** The remaining parameters below are modifiable, but modification is NOT RECOMMENDED without full calculations. */
    // Mechanical boundaries & parameters
      static const double AxesOffset = 3;           // Linear offset from hShoulder to vShoulder rotational axes
      static const double MountOffset = 3.75;       // Linear offset from mount wall to horizontal rotation axis
      static const double Leg1 = 18;                // Shoulder vertical rotation axis to elbow rotation axis
      static const double Leg2 = 14;                // Elbow rotation axis to end of closed claw
      static const int thetamin = 0 * toRadians;    // Sets the rightmost mechanical rotation limit
      static const int thetamax = 180 * toRadians;  // Sets the leftmost mechanical rotation limit
      static const double rmin = AxesOffset + 1;    // Sets the minimum distance from claw to origin. Overrides geometric minimum.
      static const double zmin = 0.0;               // Sets the vertical claw limit. MUST BE NON-NEGATIVE (Causes trig error -> erratic behavior).
      static const double ElbowWidth = 4.0;         // Width of the elbow joint assembly
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
      static const int LoopDelay = 1;               // Firmware execution delay constant
      static const int ResetDelay = 12000;          // 
  
  /** ^^^^^^^^^^      END ADJUSTABLE PARAMETERS      ^^^^^^^^^^ */
  /** --------------------------------------------------------------------------------------------------------------------------------------- */

    // Operating parameters determined by hardware limitations
      static const double AMHG = PythagH(AHVO, AHRO);                               // Actuator Mount (Head-Side) Geometry (Hypotenuse = sqrt(10))
      static const double AMRG = PythagH(ARVO, (Leg1 - ARRO));                      // Actuator Mount (Rod-Side) Geometry (Hypotenuse = sqrt(186.25))
      static const double MountComponentAngle = atan(AHVO/AHRO) * toDegrees;        // Component angles are used to calculate linear servo extension
      static const double LegComponentAngle = atan(ARVO/(Leg1 - ARRO)) * toDegrees; // 

      static const double ActPosRange = abs(ActMaxPos - ActMinPos);                 // 4.0" in current configuration
      static const double ActCmdRange = abs(ActMaxCmd - ActMinCmd);                 // 125.0 in current configuration
    
  /** These constants define the mininum and maximum acceptable analogRead() value
   *  produced by operating the control board joystick. Thresholds were determined
   *  empirically and provide proper control of four movements with one analog input.
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

  /** Hardware digital I/O pin assignments */
    // Control Inputs
      static const int Btn1 = 15;               // Up button (panel)
      static const int Btn2 = 14;               // Down button (panel)
      static const int ClawBtn = 16;            // Claw button (joystick center button)
      static const int LightBtn = 3;            // Light button (panel)
    // Arm Outputs
      static const int hShoulderPin = 9;        // Horizontal rotation (Theta) servo signal
      static const int vShoulderMPin = 8;       // Shoulder vertical rotation servo (master) signal
      static const int vShoulderSPin = 7;       // Shoulder vertical rotation servo (slave) signal
      static const int ElbowMPin = 6;           // Elbow vertical rotation servo (master) signal
      static const int ElbowSPin = 5;           // Elbow vertical rotation servo (slave) signal
      static const int ClawPin = 4;             // Claw servo signal
      static const int LightPin = 2;            // Headlight digital output signal

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
    pinMode(Btn1, INPUT);                             // Down button
    pinMode(Btn2, INPUT);                             // Up button
    pinMode(ClawBtn, INPUT);                          // Claw control input
    pinMode(LightBtn, INPUT);                         // Light button input
    pinMode(LightPin, OUTPUT);                        // Light output signal

  // Enable serial debugging messages
    Serial.begin(9600);
  
  // Enable Arm
    PowerUp();
}

/** loop() Function
 * Arduino Process
 */
void loop() {
  delay(LoopDelay);                                   // Controls program execution speed
  
  /** Watchdog Timer
   *  When controls are inactive for time set by "watchdog" constant, the arm
   *  powers down and waits in an internal loop until an input is received.
  */
      timer = 0;                                      // Reset timer
      do {                                            // Begin input loop
        in = GetInput();                              // Check for input (also records input for processing)
        if (++timer >= watchdog) {                    // Increment timer and check for timeout
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
      if (in == 1) {
        Coor->setR(Coor->R() + rinc);                 // increment radius (Extend Out)
      }
      else if (in == 2) {
        Coor->setR(Coor->R() - rinc);                 // decrement radius (Retract In)
      }
      else if (in == 3) {
        Coor->setT(Coor->T() + thetainc);             // increment theta (Rotate Left)
      }
      else if (in == 4) {
        Coor->setT(Coor->T() - thetainc);             // decrement theta (Rotate Right)
      }
      else if (in == 5) {
        Coor->setZ(Coor->Z() - zinc);                 // decrement z (Retract Up)
      }
      else if (in == 6) {
        Coor->setZ(Coor->Z() + zinc);                 // increment z (Extend Down)
      }

  /** Perform exhibit boundary and mechanical limit checks */
      checkBoundaries();
  
  /** Move arm to modified commanded position */
      updatePositions();
}

/** ------------------------------------------------------------------------------------------------------------------------------------------- */

/** checkBoundaries() Function
 * 
 */
void checkBoundaries() {
  /** Check Claw Position Boundaries
   * Compares commanded claw position to exhibit boundaries and physical limitations of the arm.
   * If a boundary conflict is found, the parameter which violates the boundary is modified to equal the boundary,
   * which may cause other parameters to change such that all boundary limits are satisfied.
   */
    
  /** Print commanded position debug info */
    Serial.print("\nCommanded: ");
    printCoor();

    /** Exhibit boundary check.
     *  If the arm is commanded to exceed the boundary, the arm will stop.
     */
      if (Coor->Z() > FLOORBOUND) {                               // Check floor boundary
        Coor->setZ(FLOORBOUND);
      }
      if (Coor->Z() < zmin) {                                     // Check vertical height boundary
        Coor->setZ(zmin);
      }
      if (Coor->T() < thetamin) {                                 // Check rightmost mechanical rotation limit
        Coor->setT(thetamin);
      }
      if (Coor->T() > thetamax) {                                 // Check leftmost mechanical rotation limit
        Coor->setT(thetamax);
      }
      if (Coor->R() < rmin) {                                     // Check for overfolding of elbow
        Coor->setR(rmin);
      }

    /** Left/Right boundary checks. Modifies Cartesian X to allow continued arm movement.
     *  If the arm is commanded to exceed a boundary, the X Cartesian component is held at the boundary,
     *  allowing the Y Cartesian component to continue movement. */
      if ((Coor->T() > (90 * toRadians)) && (Coor->X() <= (LBOUND + PADDING))) {
        Coor->setX(LBOUND + PADDING);
      }
      if ((Coor->T() < (90 * toRadians)) && (Coor->X() >= (RBOUND - PADDING))) {
        Coor->setX(RBOUND - PADDING);
      }
        
    /** Front boundary check. (Never exceeded in current configuration).
     *  Modifies Cartesian Y to the boundary, allowing continued arm movement. */
      if (Coor->Y() > (FBOUND - (MountOffset + PADDING))) {
        Coor->setY(FBOUND - (MountOffset + PADDING));
      }
    
    /** Check full-extension mechanical limitation.
     *  Prevents the arm from being fully extended (linear). */
      double ClawVector = PythagH(Coor->R() - AxesOffset, Coor->Z());
      if (ClawVector > (Leg1 + Leg2 - 1)) {
        Coor->setR(PythagL((Leg1 + Leg2 - 1), Coor->Z()) + AxesOffset);
      }

    /** Check full-folding mechanical limitation. 
     *  Limit the radius by calculating the minimum distance from the vertical
     *  rotation axis to the claw according to the minimum elbow angle. */
      double minClawVector = LoCOppSide(ElbowMinAngle, Leg1, Leg2);
      if (ClawVector < minClawVector) {
        Coor->setR(PythagL(minClawVector, Coor->Z()) + AxesOffset);
      }
    
  /** Print adjusted position debug info */
    Serial.print("     Adjusted: ");
    printCoor();
}

/** updatePositions() Function
 * Calculates required servo commands.
 * Peforms final mathematical/boundary checks to ensure valid positioning
 * Moves servos to 
 * modified commanded position.
 * Also provides basic serial debugging output.
 */
void updatePositions() {
  /** Calculate vertical rotation axis to claw distance */
    double ClawVector = PythagH(Coor->R() - AxesOffset, Coor->Z());
  /** Calculate elbow angle using Law of Cosines */
    double elbow = LoCAngleDEG(Leg1, Leg2, ClawVector);
  /** Shoulder angle is the sum of two component angles formed by the arm */
    double shoulder = (asin((Coor->R() - AxesOffset) / ClawVector) * toDegrees) + LoCAngleDEG(ClawVector, Leg1, Leg2);
      
  /** Convert shoulder angle to linear actuator position */
    double ActCmd = actuatorCommand(shoulder);

  /** Print debugging parameters */
    Serial.print("   Elbow: " + String(elbow) + "   Shoulder: " + String(shoulder) + "   ClawPos: " + String(ClawVector));

  /** Move servos */
    hShoulder.Move(Coor->T() * toDegrees);
    vShoulderM.Move(ActCmd);
    ElbowM.Move(180 - elbow);
    ElbowS.Move(180 - elbow);
    Claw.Move(ClawState);
}


/** actuatorCommand() Function
 *  Converts the specified angleDEG shoulder angle to a joint() object compatible linear actuator command
 *  based on the hardware configuration geometry defined/calculated in the global constants section.
 */
double actuatorCommand(double angleDEG) {
  double ActuatorComponentAngle = 360 - (MountComponentAngle + LegComponentAngle + 90 + angleDEG);
  double ActuatorLength = LoCOppSide(ActuatorComponentAngle, AMHG, AMRG);
  double ActuatorCommand = ((ActMaxPos - ActuatorLength) * (ActCmdRange / ActPosRange)) + ActMaxCmd;
  if (ActuatorLength < ActMinPos) {
    Serial.print("\nActuator min-extension boundary violation!");
    return ActMinCmd;
  }
  else if (ActuatorLength > ActMaxPos) {
    Serial.print("\nActuator max-extension boundary violation!");
    return ActMaxCmd;
  }
  else {
    return ActuatorCommand;
  }
}


/** --------------------------------------------------------------------------------------------------------------------------------------- */
/** Trig Functions (Global by prototype) */

double LoCAngleDEG(double Adj1, double Adj2, double Opp) {
  return (acos((pow(Adj1, 2) + pow(Adj2, 2) - pow(Opp, 2)) / (2 * Adj1 * Adj2)) * toDegrees);
}

double LoCOppSide(double AngleDEG, double Adj1, double Adj2) {
  return sqrt(pow(Adj1, 2) + pow(Adj2, 2) - (2 * cos(AngleDEG * toRadians) * Adj1 * Adj2));
}

double PythagH(double L1, double L2) {
  return sqrt(pow(L1, 2) + pow(L2, 2));
}

double PythagL(double H, double L) {
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
      Serial.print("\nReset sequence");
      while(digitalRead(LightBtn)) {
        ++reset;
        Serial.print('.');
        if (!(reset % 1000)) {
          Serial.print("\n");
        }
        if (reset >= ResetDelay) {
          Serial.print("\nResetting.");
          PowerUp();
          break;
        }
      }
      Serial.print("\n");
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
    Serial.print("\nPowering up...");
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
    updatePositions();
    blinkLED(false);
    Serial.print("READY.");
}

/** PowerDown() Function
 * Disables control of arm servos, deenergizing them after positioning the arm near the exhibit floor.
 */
void PowerDown() {
    Serial.print("\nGoing to stanby...");
  // Move to rest position
    Coor->setR(rinit);
    Coor->setT(tinit);
    Coor->setZ(FLOORBOUND);  
    updatePositions();  
    delay(2000);
  // Power-down
    hShoulder.Stop();
    vShoulderM.Stop();
    vShoulderS.Stop();
    ElbowM.Stop();
    ElbowS.Stop();
    Claw.Stop();
    blinkLED(false);
    Serial.print("DONE.");
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

void printCoor() {
  Serial.print("(r,t,z): (" + String(Coor->R()) + ", " + String(Coor->T() * toDegrees) + ", " + String(Coor->Z()) + ")");
}

