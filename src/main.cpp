#include <Arduino.h>
#include <NBHX711.h>
#include <Servo.h>
#include <limits.h>

constexpr int HX711_DOUT_PIN = 2;
constexpr int HX711_SCK_PIN = 3;
constexpr int SERVO_PIN = 4;
constexpr int BUTTON_PIN = 5;
constexpr int POT_PIN = A7;

constexpr bool THRESHOLD_PUSH = false; // Threshold pushes back against you
constexpr bool THRESHOLD_PULL = true; // Threshold pulls away from you (true = "do direction reversal")
constexpr float REVERSE = -1.0f;

// Strain gauge wiring:
// Reference: https://www.transducertechniques.com/wheatstone-bridge.aspx#:~:text=A%20half%2Dbridge%20type%20II,the%20opposite%20side%20(bottom)
// Red wire represents "measurement" wire, white/black are excitation wires
// Resistance of each strain-gauage element is 1k (white-black resistance is 2k)

// #define TESTING

// TODO List:
// 1. X Update code: Add reading angle from pot
//    May want to set as some kind of toy, e.g., rotation of the pot sets the set-point of the servo
// 2. X Set up hardware:
//    1. X Breadboard: Connect servo, HX711 + load cell, pot
//    2. X Mount: Should a mount of some kind be printed for this?
// 3. Test
//    1. X Test the limToSetPoint flag
//    2. Test the joint design (components feel secure, have full range of motion, little to no backlash)
// 4. Update code: Add capability of measuring all 3 load cells and pots
// 5. Print out and assemble full joint
//      May need a mount of some kind - should be pretty solid so there's no backlash!

// Set servo parameters
constexpr float SERVO_MIN_ANGLE = 0.0f;
constexpr float SERVO_MAX_ANGLE = 180.0f;
constexpr float SERVO_MIN_US = 500.0f;
constexpr float SERVO_MAX_US = 2500.0f;
constexpr unsigned long SERVO_UPDATE_PERIOD_US = 0;
static float CurJointAngle;
static float ThresholdPoint;
static Servo servo;

// Set combined force and rotation measurement parameters
constexpr unsigned long MEASUREMENT_PERIOD_US = 40000; // Assume regular 25Hz measurements

// Set Scale parameters
constexpr bool USE_SCALE_LOWPASS_FILTER = true; // Default - use sliding window filter
constexpr int SCALE_IIR_LEN = 3;
constexpr float SCALE_IIR_A0 = 4.831f;
constexpr float SCALE_IIR_A1 = 1.789;
constexpr float SCALE_IIR_A2 = -0.948f;
// constexpr float SCALE_IIR_B0 = 1.0f; // Excluded for efficiency
constexpr float SCALE_IIR_B1 = 2.0f;
// constexpr float SCALE_IIR_B2 = 1.0f; // Excluded for efficiency
constexpr byte SCALE_SLIDING_WINDOW_LEN = 10;
constexpr float SCALE_VAL_TO_GRAMS = 353.8f;
constexpr byte NUM_TARE_READS = SCALE_SLIDING_WINDOW_LEN;
constexpr unsigned long TARE_DEBOUNCE_US = 500000;

// Set potentiometer parameters
constexpr float POT_ANGLE_MIN = 0.0f; // Minimum angle of pot
constexpr float POT_ANGLE_MAX = 180.0f; // Maximum angle of pot
// (Following are useful if pot does not wipe full range)
constexpr float POT_VOLT_MIN = 0.0f; // Voltage at minimum angle of pot
constexpr float POT_VOLT_MAX = 5.0f; // Voltage at maximum angle of pot
constexpr float POT_EXCITE_MIN = 0.0f; // Voltage on negative terminal of pot
constexpr float POT_EXCITE_MAX = 5.0f; // Voltage on positive terminal of pot

// Scale variables, functions, and derivative parameters
constexpr byte NUM_AVERAGED_SCALE_READS = USE_SCALE_LOWPASS_FILTER ?
  1 : SCALE_SLIDING_WINDOW_LEN;
constexpr int IIR_IND_1 = 0; // y0 is new output value, x0 is new input val - neither in array
constexpr int IIR_IND_2 = 1;
constexpr byte ONE_SAMPLE = 1;
static float IIRInputs[SCALE_IIR_LEN - 1] = {0.0f};
static float IIROutputs[SCALE_IIR_LEN - 1] = {0.0f};
static NBHX711 scale( HX711_DOUT_PIN, HX711_SCK_PIN, NUM_AVERAGED_SCALE_READS );
static unsigned long LastTareTime = 0;
static float RawInputForce = 0.0f;
static float AdjustedInputForce = 0.0f;
inline float forceSpeedCurve( float forceInput );
inline float fMap( float x, float in_min, float in_max, float out_min, float out_max );
inline int angleToMicroSeconds( float commandAngle );
inline float PerformScaleLPIIRFilter( float input );
void TareScale( void );
void ResetScaleIIRFilter( void );

// Combined force and rotation measurement variables
unsigned long LastMeasureTime = 0;

// Potentiometer variables
static float PotAngle = 0.0f; // Last read potentiometer angle

// Force to speed conversion parameters
constexpr int FORCE_DIRECTION = 1;     // 1: positive force = positive angle velocity
constexpr float FORCE_SPEED_SCALE = 0.25;   // g/(degree/s)
constexpr float FORCE_DEAD_ZONE = 20.0;  // g

// Threshold testing parameters
// constexpr int SERVO_SET_POINT = 90;                 // degrees
// constexpr float SERVO_SET_POINT_F = 90.0f;          // degrees
constexpr float THRESHOLD_FORCE = 100.0f;              // g
constexpr bool THRESHOLD_ONLY_RESISTIVE = false;       // true = Servo cannot move on its own
constexpr bool THRESHOLD_DIRECTION = THRESHOLD_PUSH;

// General conversions
constexpr float ANALOG_READ_MIN_F = 0.0f;
constexpr float ANALOG_READ_MAX_F = 1023.0f;
constexpr float MICROSECONDS_TO_SECONDS =  1000000.0f; // Divide

// Timing variables
static unsigned long LastTime = micros( );

#ifdef TESTING
constexpr unsigned long UPDATE_CHECK_PERIOD_US = 100000;
static unsigned int NumUpdates = 0;
static unsigned int LastNumUpdates = 0;
static unsigned long LastUpdateCheck = 0;
#endif

void setup( )
{
#ifdef TESTING
  Serial.begin( 9600 );
#endif

  // Initialize the HX711
  scale.begin( );
  TareScale( );
  scale.setScale( SCALE_VAL_TO_GRAMS );

  // Initialize servo
  servo = Servo( );
  servo.attach( SERVO_PIN );
  pinMode( BUTTON_PIN, INPUT_PULLUP );
}

void loop( )
{
  // Calculate the time difference for this cycle
  unsigned long thisTime = micros( );
  float dt; // Time difference in microseconds
  if ( thisTime >= LastTime )
  {
    dt = ( (float) ( thisTime - LastTime ) ) / MICROSECONDS_TO_SECONDS;
  }
  else
  {
    dt = ( ( (float) thisTime ) + ( (float) ULONG_MAX ) - ( (float) LastTime ) ) /
      MICROSECONDS_TO_SECONDS ;
  }
  LastTime = thisTime;

  // Tare scale, if requested
  int newTareButtonVal = digitalRead( BUTTON_PIN );
  if ( newTareButtonVal == LOW )
  {
    if ( ( thisTime - LastTareTime ) >= TARE_DEBOUNCE_US )
    {
      TareScale( );
      LastTareTime = thisTime;
    }
  }

  // Update measurements
  if ( ( thisTime - LastMeasureTime ) >= MEASUREMENT_PERIOD_US )
  {
    // Measurement from potentiometer
    float rawVal = (float) analogRead( POT_PIN ); // Note: "Left" is 0, "Right" is  180
    PotAngle = fMap(
      rawVal, ANALOG_READ_MIN_F, ANALOG_READ_MAX_F, POT_ANGLE_MIN, POT_ANGLE_MAX );
    ThresholdPoint = PotAngle;

    // Measurement from scale
    scale.update( ); // TODO: If still using IIR filter, standard HX711 library is more efficient
    if ( USE_SCALE_LOWPASS_FILTER )
    {
      RawInputForce = PerformScaleLPIIRFilter( scale.getUnits( ONE_SAMPLE ) ); // TODO: Start here - check that IIR filter works!  Maybe check in MATLAB too?
    }
    else
    {
      RawInputForce = scale.getUnits( NUM_AVERAGED_SCALE_READS );
    }

    // Use deadzone for scale
    if ( abs( RawInputForce ) < FORCE_DEAD_ZONE )
    {
      AdjustedInputForce = 0; // Threshold the measurement from the load cell
    }
    else
    {
      AdjustedInputForce = ( RawInputForce > 0 ) ?
        ( RawInputForce - FORCE_DEAD_ZONE ) :
        ( RawInputForce + FORCE_DEAD_ZONE );
    }

  #ifdef TESTING
    ++NumUpdates;
  #endif
    LastMeasureTime = thisTime;
  }

  float controlSignal = AdjustedInputForce;

  // Use a threshold on the force measurement
  // Encode threshold direction by using positive or negative of joint angle
  // and control signal
  if ( THRESHOLD_DIRECTION == THRESHOLD_PULL )
  {
    CurJointAngle = CurJointAngle * REVERSE;
    controlSignal = controlSignal * REVERSE;
  }

  bool limToSetPoint = false;
  if ( CurJointAngle >= ThresholdPoint )
  {
    // If the servo angle is GTE the set point, apply the threshold rules
    if ( !THRESHOLD_ONLY_RESISTIVE || controlSignal > 0 )
    {
      // If the actuator is allowed to push back on its own OR the input force is pushing farther up, then apply the threshold
      limToSetPoint = controlSignal >= 0; // Don't allow threshold force to push actuator BELOW threshold on its own input force is positive
      controlSignal -= THRESHOLD_FORCE; // Threshold force is pushing DOWN
      if ( THRESHOLD_ONLY_RESISTIVE )
      {
        controlSignal = max( controlSignal, 0.0f ); // Don't let the servo move on its own (min force of +0)
      }
    }
  }

  if ( THRESHOLD_DIRECTION == THRESHOLD_PULL )
  {
    controlSignal = controlSignal * REVERSE;
  }

  // Convert the force measurement into a velocity
  float dJointAngleDt = forceSpeedCurve( controlSignal );

  // Integrate the velocity to get the change in position
  float dJointAngle = dJointAngleDt * dt;

  // Find the new joint angle
  float minAngle = limToSetPoint ? ThresholdPoint : SERVO_MIN_ANGLE; // TODO START HERE (limToSetPoint fixed)
  CurJointAngle += dJointAngle;
  CurJointAngle = constrain( CurJointAngle, minAngle, SERVO_MAX_ANGLE );

#ifdef TESTING
  if ( ( thisTime - LastUpdateCheck ) > UPDATE_CHECK_PERIOD_US )
  {
    LastUpdateCheck = thisTime;
    LastNumUpdates = NumUpdates;
    NumUpdates = 0;
    // Serial.println( LastNumUpdates );
    Serial.println( );
    Serial.println( controlSignal );
    Serial.println( minAngle );
    Serial.println( limToSetPoint );
    Serial.println( CurJointAngle );
}
  // Write the value to the servo
  servo.writeMicroseconds( angleToMicroSeconds( CurJointAngle ) );
#else
  // Write the new angle
  servo.writeMicroseconds( angleToMicroSeconds( CurJointAngle ) );
#endif
}

inline float forceSpeedCurve( float forceInput )
{
  // Convert the force (above a given threshold) to a speed
  // For now, try a linear relationship
  return forceInput * FORCE_SPEED_SCALE;
}

inline int angleToMicroSeconds( float commandAngle )
{
  return static_cast<int>( fMap(
    commandAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
    SERVO_MIN_US, SERVO_MAX_US ) );
}

inline float PerformScaleLPIIRFilter( float newInput )
{
  float newOutput =
    newInput              + // * SCALE_IIR_B0 (B0 is 1.0)
    IIRInputs [IIR_IND_1]      * SCALE_IIR_B1 +
    IIRInputs [IIR_IND_2] + // * SCALE_IIR_B2 (B1 is 1.0)
    IIROutputs[IIR_IND_1]      * SCALE_IIR_A1 +
    IIROutputs[IIR_IND_2]      * SCALE_IIR_A2;

  newOutput /= SCALE_IIR_A0;

  // Slide input / output queues
  IIRInputs [IIR_IND_2] = IIRInputs [IIR_IND_1];
  IIRInputs [IIR_IND_1] = newInput;
  IIROutputs[IIR_IND_2] = IIROutputs[IIR_IND_1];
  IIROutputs[IIR_IND_1] = newOutput;

  // Return new value
  return newOutput;
}

void TareScale( void )
{
  #ifdef TESTING
  Serial.println( "Do tare!" );
  #endif

  scale.tare( NUM_TARE_READS );
  ResetScaleIIRFilter( );

  #ifdef TESTING
  Serial.println( "Done with tare!" );
  #endif
}

void ResetScaleIIRFilter( void )
{
  for ( unsigned char i = 0; i < ( SCALE_IIR_LEN - 1 ); i++ )
  {
    IIRInputs[i] = 0.0f;
    IIROutputs[i] = 0.0f;
  }
}

inline float fMap( float x, float in_min, float in_max, float out_min, float out_max )
{
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}