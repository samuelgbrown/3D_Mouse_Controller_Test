#include <Arduino.h>
// #include <NBHX711.h>
#include <HX711.h>
#include <Servo.h>
#include <limits.h>

enum FILTERING_TYPE {
  NO_FILTERING = 0,
  SLIDING_WINDOW,
  IIR_FILTERING
};

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
// 2. Test the joint design (components feel secure, have full range of motion, little to no backlash)
//    2.a. TODO: Need to update hardware design - replace flimsy potentiometer with more solid one
//    2.b. Update connections - crimp dupont connectors to sensor wires
// 3. X Check scale filter designs - sliding window adequate, or is LP IIR better?
// 4. Update code: Add capability of measuring all 3 load cells and pots
//    4.a Check if Arduino is fast enough to run this.  May need to try this on the RasPi Zero
// 5. Print out and assemble full joint
//      May need a mount of some kind - should be pretty solid so there's no backlash!
  
// Set servo parameters
constexpr float SERVO_MIN_ANGLE = 0.0f;
constexpr float SERVO_MAX_ANGLE = 180.0f;
constexpr float SERVO_MIN_US = 500.0f;
constexpr float SERVO_MAX_US = 2500.0f;
constexpr float SERVO_START_POINT = 90.0f;
constexpr float MAX_SPEED = 50.0f; // degree/s
constexpr unsigned long SERVO_UPDATE_PERIOD_US = 25000; // 40Hz
static int CurJointAngle_us = 0; // TESTING ONLY
static float CurJointAngle = SERVO_START_POINT;
static unsigned long LastServoUpdateTime = 0;
static float ThresholdPoint;
static Servo servo;

// Set combined force and rotation measurement parameters
constexpr unsigned long START_DELAY = 1000; // Start delay, in ms
constexpr unsigned long SENSOR_UPDATE_PERIOD_US = 25000; // 40Hz

// Set Scale parameters
constexpr float FORCE_SPEED_SCALE = 0.2;   // g/(degree/s)filter
constexpr FILTERING_TYPE FILTERING_TO_USE = NO_FILTERING; // Default - use sliding window 
constexpr byte SCALE_SLIDING_WINDOW_LEN = 10;
constexpr float SCALE_VAL_TO_GRAMS = 353.8f;
constexpr byte NUM_TARE_READS = 10;
constexpr unsigned long TARE_BUTTON_DEBOUNCE_US = 500000;
constexpr float SCALE_ASYM_GAIN = 1.75f; // Increased relative gain for positive force compared to negative

// Scale input filtering (designed using https://www.meme.net.au/butterworth.html)
// V1: fs=25Hz, fc=5Hz
// constexpr int SCALE_IIR_LEN = 3;
// constexpr float SCALE_IIR_A0 = 4.831f;
// constexpr float SCALE_IIR_A1 = 1.789f;
// constexpr float SCALE_IIR_A2 = -0.948f;
// // constexpr float SCALE_IIR_B0 = 1.0f; // Excluded for efficiency
// constexpr float SCALE_IIR_B1 = 2.0f;
// // constexpr float SCALE_IIR_B2 = 1.0f; // Excluded for efficiency

// V2: fs=10Hz, fc=5Hz
constexpr int SCALE_IIR_LEN = 2;
// constexpr float SCALE_IIR_A0 = 1.0f;
// constexpr float SCALE_IIR_A1 = -1.0f;
// constexpr float SCALE_IIR_B0 = 1.0f; // Excluded for efficiency
// constexpr float SCALE_IIR_B1 = 1.0f;

// Set potentiometer parameters
constexpr float POT_ANGLE_MIN = 0.0f; // Minimum angle of pot
constexpr float POT_ANGLE_MAX = 180.0f; // Maximum angle of pot
// (Following are useful if pot does not wipe full range)
constexpr float POT_VOLT_MIN = 0.0f; // Voltage at minimum angle of pot
constexpr float POT_VOLT_MAX = 5.0f; // Voltage at maximum angle of pot
constexpr float POT_EXCITE_MIN = 0.0f; // Voltage on negative terminal of pot
constexpr float POT_EXCITE_MAX = 5.0f; // Voltage on positive terminal of pot

// Scale variables, functions, and derivative parameters
constexpr byte NUM_AVERAGED_SCALE_READS = SCALE_SLIDING_WINDOW_LEN;
constexpr byte SCALE_SIZE_BUFFER = max( NUM_AVERAGED_SCALE_READS, NUM_TARE_READS );
constexpr int IIR_IND_1 = 0; // y0 is new output value, x0 is new input val - neither in array
// constexpr int IIR_IND_2 = 1;
constexpr byte ONE_SAMPLE = 1;
static float IIRInputs[SCALE_IIR_LEN - 1] = {0.0f};
static float IIROutputs[SCALE_IIR_LEN - 1] = {0.0f};
// static NBHX711 scale( HX711_DOUT_PIN, HX711_SCK_PIN, SCALE_SIZE_BUFFER );
static HX711 scale;
static unsigned long LastTareTime = 0;
static float FiltedRawInputForce = 0.0f;
static float AdjustedInputForce = 0.0f;
inline float forceSpeedCurve( float forceInput );
inline float fMap( float x, float in_min, float in_max, float out_min, float out_max );
inline int angleToMicroSeconds( float commandAngle );
inline float PerformScaleLPIIRFilter( float input );
void TareScale( void );
void ResetScaleIIRFilter( void );
void UpdateTareCheck( unsigned long thisTime );
void UpdateSensors( unsigned long thisTime );
void UpdateServo( unsigned long thisTime );

// Combined force and rotation measurement variables
static unsigned long LastMeasureTime = 0;

// Potentiometer variables
static float PotAngle = 0.0f; // Last read potentiometer angle

// Force to speed conversion parameters
constexpr int FORCE_DIRECTION = 1;     // 1: positive force = positive angle velocity
constexpr float MAX_SPEED_NEG = -1.0f * MAX_SPEED; // degree/s
constexpr float FORCE_DEAD_ZONE = 20.0;  // g

// Threshold testing parameters
// constexpr int SERVO_SET_POINT = 90;                 // degrees
// constexpr float SERVO_SET_POINT_F = 90.0f;          // degrees
constexpr float THRESHOLD_FORCE = 200.0f;              // g
constexpr bool THRESHOLD_ONLY_RESISTIVE = false;       // true = Servo cannot move on its own
constexpr bool THRESHOLD_DIRECTION = THRESHOLD_PUSH;

// General conversions
constexpr float ANALOG_READ_MIN_F = 0.0f;
constexpr float ANALOG_READ_MAX_F = 1023.0f;
constexpr float MICROSECONDS_TO_SECONDS =  1000000.0f; // Divide

#ifdef TESTING
constexpr unsigned long UPDATE_CHECK_PERIOD_US = 300000;
static unsigned int NumUpdates = 0;
static unsigned int LastNumUpdates = 0;
static unsigned long LastUpdateCheck = 0;
#endif

void setup( )
{
#ifdef TESTING
  Serial.begin( 9600 );
  Serial.print( "Starting...");
  #endif

  // Initialize the HX711
  scale.begin( HX711_DOUT_PIN, HX711_SCK_PIN ); // TODO: If still using IIR filter, standard HX711 library is more efficient
  scale.set_scale( SCALE_VAL_TO_GRAMS );
  scale.wait_ready( );
  #ifdef TESTING
    Serial.println( "Scale ready!");
  #endif
  TareScale( );

  // Initialize servo
  servo = Servo( );
  servo.attach( SERVO_PIN );
  servo.write( CurJointAngle );
  pinMode( BUTTON_PIN, INPUT_PULLUP );

  // Wait for servo to settle before starting
  delay( START_DELAY );
  #ifdef TESTING
    Serial.print( "Finished start-up!");
  #endif
}

void loop( )
{
  // Get the timestamp for this cycle
  unsigned long thisTime = micros( );

  UpdateTareCheck( thisTime );

  UpdateSensors( thisTime );

  UpdateServo( thisTime );

  #ifdef TESTING
  if ( ( thisTime - LastUpdateCheck ) > UPDATE_CHECK_PERIOD_US )
  {
    LastUpdateCheck = thisTime;
    LastNumUpdates = NumUpdates;
    NumUpdates = 0;
    Serial.println( );
    // Serial.println( LastNumUpdates );
    Serial.println( AdjustedInputForce );
    // Serial.println( CurJointAngle_us );
    Serial.println( CurJointAngle );
  }
  #endif
}

void TareScale( void )
{
  // Note: This is a blocking function!
  // Will block for as long as it takes to get all measurerments!
  #ifdef TESTING
  Serial.println( "Do tare!" );
  Serial.print( "Old offset: " );
  Serial.println( scale.get_offset( ) );
  #endif
  digitalWrite( LED_BUILTIN, HIGH ); // Light LED to indicate TARE is active

  // // Perform a series of fresh updates
  // byte tareMeasurementsPerformed = 0;
  // while ( tareMeasurementsPerformed < NUM_TARE_READS )
  // {
  //   if ( scale.update( ) )
  //   {
  //     tareMeasurementsPerformed++;

  //     #ifdef TESTING
  //     Serial.print( tareMeasurementsPerformed );
  //     Serial.print( "-" );
  //     #endif
  //   }
  // }

  // Perform the tare, and reset the filter
  scale.tare( NUM_TARE_READS );
  ResetScaleIIRFilter( );

  #ifdef TESTING
  Serial.println( );
  Serial.println( "Done with tare!" );
  Serial.print( "New offset: " );
  Serial.println( scale.get_offset( ) );
  #endif
  digitalWrite( LED_BUILTIN, LOW );
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

void UpdateTareCheck( unsigned long thisTime )
{
  // Tare scale, if requested
  int newTareButtonVal = digitalRead( BUTTON_PIN );
  if ( newTareButtonVal == LOW )
  {
    if ( ( thisTime - LastTareTime ) >= TARE_BUTTON_DEBOUNCE_US )
    {
      TareScale( );
      LastTareTime = thisTime;
    }
  }
}

void UpdateSensors( unsigned long thisTime )
{
  // Read from potentiometer and scale
  if ( ( ( thisTime - LastMeasureTime ) >= SENSOR_UPDATE_PERIOD_US ) &&
          scale.is_ready( ) )
  {
    // When new data is available from the scale (~10Hz), perform the measurement step
    // Measurement from potentiometer
    float rawVal = (float) analogRead( POT_PIN ); // Note: "Left" is 0, "Right" is  180
    PotAngle = fMap(
      rawVal, ANALOG_READ_MIN_F, ANALOG_READ_MAX_F, POT_ANGLE_MIN, POT_ANGLE_MAX );
    ThresholdPoint = PotAngle;

    // Measurement from scale
    float rawInputForce;
    float scaledRawInputForce;
    switch ( FILTERING_TO_USE )
    {
      case SLIDING_WINDOW:
        rawInputForce = scale.get_units( NUM_AVERAGED_SCALE_READS );
        FiltedRawInputForce = ( rawInputForce > 0.0f ) ?
          ( rawInputForce * SCALE_ASYM_GAIN ) : rawInputForce; // Perform asymmetric gain
        break;
      case IIR_FILTERING:
        rawInputForce = scale.get_units( ONE_SAMPLE );
        scaledRawInputForce = ( rawInputForce > 0.0f ) ?
          ( rawInputForce * SCALE_ASYM_GAIN ) : rawInputForce; // Perform asymmetric gain
        FiltedRawInputForce = PerformScaleLPIIRFilter( scaledRawInputForce );
        break;
      default:
        // No filtering
        rawInputForce = scale.get_units( ONE_SAMPLE );
        FiltedRawInputForce = ( rawInputForce > 0.0f ) ?
          ( rawInputForce * SCALE_ASYM_GAIN ) : rawInputForce; // Perform asymmetric gain
        break;
    }

    // Use deadzone for scale
    if ( abs( FiltedRawInputForce ) < FORCE_DEAD_ZONE )
    {
      AdjustedInputForce = 0; // Threshold the measurement from the load cell
    }
    else
    {
      AdjustedInputForce = ( FiltedRawInputForce > 0 ) ?
        ( FiltedRawInputForce - FORCE_DEAD_ZONE ) :
        ( FiltedRawInputForce + FORCE_DEAD_ZONE );
    }
    
    #ifdef TESTING
    ++NumUpdates;
    #endif
    LastMeasureTime = thisTime;
  }
}

void UpdateServo( unsigned long thisTime )
{
  if ( ( thisTime - LastServoUpdateTime ) >= SERVO_UPDATE_PERIOD_US )
  {
    // Calculate dt for this servo update
    float dt; // Time difference in microseconds
    if ( thisTime >= LastServoUpdateTime )
    {
      dt = ( (float) ( thisTime - LastServoUpdateTime ) ) / MICROSECONDS_TO_SECONDS;
    }
    else
    {
      dt = ( ( (float) thisTime ) + ( (float) ULONG_MAX ) - ( (float) LastServoUpdateTime ) ) /
        MICROSECONDS_TO_SECONDS ;
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

    // #ifndef TESTING
    // Write the new angle
    CurJointAngle_us = angleToMicroSeconds( CurJointAngle );
    servo.writeMicroseconds( CurJointAngle_us );
    // #endif

    LastServoUpdateTime = thisTime;
  }
}

inline float forceSpeedCurve( float forceInput )
{
  // Convert the force (above a given threshold) to a speed
  // For now, try a linear relationship
  return constrain( forceInput * FORCE_SPEED_SCALE, MAX_SPEED_NEG, MAX_SPEED );
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
    newInput                 + // * SCALE_IIR_B0   (B0 is 1.0)
    IIRInputs [IIR_IND_1]    - // * SCALE_IIR_B1 + (B1 is 1.0)
    // IIRInputs [IIR_IND_2] + // * SCALE_IIR_B2   (B2 is not defined)
    IIROutputs[IIR_IND_1];     // * SCALE_IIR_A1 + (A1 is -1.0)
    // IIROutputs[IIR_IND_2]      * SCALE_IIR_A2;+ (A2 is not defined)

  // newOutput /= SCALE_IIR_A0; // (A0 is 1.0)

  // Slide input / output queues
  // IIRInputs [IIR_IND_2] = IIRInputs [IIR_IND_1];
  IIRInputs [IIR_IND_1] = newInput;
  // IIROutputs[IIR_IND_2] = IIROutputs[IIR_IND_1];
  IIROutputs[IIR_IND_1] = newOutput;

  // Return new value
  return newOutput;
}
