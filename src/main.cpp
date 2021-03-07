#include <Arduino.h>
#include <HX711.h>
#include <Servo.h>

const int HX711_DOUT_PIN = 2;
const int HX711_SCK_PIN = 4;
const int SERVO_PIN = 3;

HX711 scale;
Servo servo;
float forceSpeedCurve(float rawInputForce);
float fMap(float x, float in_min, float in_max, float out_min, float out_max);

// #define TESTING

// Set servo parameters
// int scaleMin = 0;
// int scaleMax = 200;
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
float curJointAngle;

// Force to speed conversion parameters
const long FORCE_DIRECTION = 1;     // 1: positive force = positive angle velocity
const float FORCE_SPEED_SCALE = 1; // g/(degree/s)
const long FORCE_NOISE_FLOOR = 10;  // g

// Threshold testing parameters
const int SERVO_SET_POINT = 90;                 // degrees
const long THRESHOLD_FORCE = 30;                // g
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
  if (abs(rawInputForce) < FORCE_NOISE_FLOOR)
  {
    adjustedInputForce = 0; // Threshold the measurement from the load cell
  }
  else
  {
    adjustedInputForce = adjustedInputForce > 0 ? adjustedInputForce - FORCE_NOISE_FLOOR : adjustedInputForce + FORCE_NOISE_FLOOR; // TODO TESTING: Is this a good call?
  }

  long controlSignal = adjustedInputForce;

  // Use a threshold on the force measurement
  bool limToSetPoint = false;
  if (curJointAngle <= SERVO_SET_POINT)
  {
    // If the servo angle is LTE the set point, apply the threshold rules
    if (!THRESHOLD_ONLY_RESISTIVE || controlSignal < 0)
    {
      // If the force is pushing farther down, then apply the threshold
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