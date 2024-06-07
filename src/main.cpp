#include <Arduino.h>
#include <HX711.h>
#include <Servo.h>

#define HX711_DOUT_PIN ( 2 )
#define HX711_SCK_PIN ( 4 )
#define SERVO_PIN ( 3 )

HX711 scale;
Servo servo;
float forceSpeedCurve(float rawInputForce);
float fMap(float x, float in_min, float in_max, float out_min, float out_max);

// #define TESTING

// TODO List:
// 1. Update code: Add reading angle from pot
//    May want to set as some kind of toy, e.g., rotation of the pot sets the set-point of the servo
// 2. Set up hardware:
//    1. Breadboard: Connect servo, HX711 + load cell, pot
//    2. Mount: Should a mount of some kind be printed for this?
// 3. Test
//    1. Test the limToSetPoint flag
//    2. Test the joint design (components feel secure, have full range of motion, little to no backlash)
// 4. Update code: Add capability of measuring all 3 load cells and pots
// 5. Print out and assemble full joint
//      May need a mount of some kind - should be pretty solid so there's no backlash!

// Set servo parameters
// int scaleMin = 0;
// int scaleMax = 200;
#define SERVO_MIN ( 0 )
#define SERVO_MAX ( 180 )
float curJointAngle;

// Force to speed conversion parameters
#define FORCE_DIRECTION ( 1 )     // 1: positive force = positive angle velocity
#define FORCE_SPEED_SCALE ( 1 ) // g/(degree/s)
#define FORCE_NOISE_FLOOR ( 10 )  // g

// Threshold testing parameters
#define SERVO_SET_POINT ( 90 )                 // degrees
#define THRESHOLD_FORCE ( 30 )                // g
const boolean THRESHOLD_ONLY_RESISTIVE = false; // true = Servo cannot move on its own

// Timing parameters
unsigned long lastTime = millis();
float dt; // Time difference in milliseconds

void setup()
{
#ifdef TESTING
  Serial.begin(9600);
#endif

  // Initialize the HX711
  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  scale.tare(); // Tare the board
  scale.set_scale(353.8);

  // Initialize servo
  servo = Servo();
  servo.attach(SERVO_PIN);
  servo.write(SERVO_SET_POINT);
  curJointAngle = SERVO_SET_POINT;
}

void loop()
{
  // Calculate the time difference for this cycle
  dt = float((millis() - lastTime)) / 1000;
  lastTime = millis();

  // Get a measurement from the load cell
  long rawInputForce = scale.get_units(1); // TODO: Get the non-blocking version of this
  long adjustedInputForce = rawInputForce;
  if (abs(adjustedInputForce) < FORCE_NOISE_FLOOR)
  {
    adjustedInputForce = 0; // Threshold the measurement from the load cell
  }
  else
  {
    adjustedInputForce = ( adjustedInputForce > 0 ) ? 
      ( adjustedInputForce - FORCE_NOISE_FLOOR ) :
      ( adjustedInputForce + FORCE_NOISE_FLOOR );
  }

  long controlSignal = adjustedInputForce;

  // Use a threshold on the force measurement
  bool limToSetPoint = false;
  if (curJointAngle <= SERVO_SET_POINT)
  {
    // If the servo angle is LTE the set point, apply the threshold rules
    if (!THRESHOLD_ONLY_RESISTIVE || controlSignal < 0)
    {
      // If the actuator is allowed to push back on its own OR the force is pushing farther down, then apply the threshold
      limToSetPoint = controlSignal <= 0;
      controlSignal += THRESHOLD_FORCE;
      if (THRESHOLD_ONLY_RESISTIVE)
      {
        controlSignal = min(controlSignal, 0); // Don't let the servo move on its own
      }
    }
  }

  // Convert the force measurement into an angular velocity
  float dJointAngleDt = forceSpeedCurve(controlSignal);

  // Integrate the angular velocity to get the change in position
  float dJointAngle = dJointAngleDt * ((float)dt); // Integrate the joint angle over the time period

  // Find the new joint angle
  curJointAngle = constrain(curJointAngle + dJointAngle, SERVO_MIN, limToSetPoint ? SERVO_SET_POINT : SERVO_MAX); // TODO START HERE (limToSetPoint made, but not tested)

#ifdef TESTING
  Serial.println();
  Serial.println(curJointAngle);
  Serial.println(dJointAngle);
  Serial.println(rawInputForce);

  // if (Serial.available())
  // {
  //   int microSeconds =  Serial.parseInt();
  //   while (Serial.available())
  //   {
  //     Serial.read();
  //   }

  // microSeconds = constrain(microSeconds, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  //   servo.writeMicroseconds(microSeconds);
  //   Serial.println();
  //   Serial.println(microSeconds);
  // }

  // // Convert it to a servo angle
  // int jointAngle = map(newVal, scaleMin, scaleMax, servoMin, servoMax);

  // // Write the value to the servo
  // servo.write(jointAngle);
#else
  // Write the new angle
  servo.write(round(curJointAngle)); // TODO: Convert to microsecond pulse width for higher precision?
#endif
}

float forceSpeedCurve(float rawInputForce)
{
  // Convert the force (above a given threshold) to a speed
  // For now, try a linear relationship
  return rawInputForce * FORCE_SPEED_SCALE;
}

float fMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}