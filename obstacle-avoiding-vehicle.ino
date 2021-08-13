
// ARDUINO OBSTACLE-AVOIDING CAR
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

//
// PINS
//
#define TRIG_PIN       A0
#define ECHO_PIN      A1
#define SERVO_PIN       10

//
// CONSTANTS
//
enum Motor
{
  RIGHT = 0,
  LEFT,   // = 1
  NUM_MOTORS  // = 2
};

#define MAX_DISTANCE    200
#define MAX_SPEED       250
#define MAX_SPEED_OFFSET  20

// Servo rotation angle to look to the left
// or right. Change these accordingly
#define LOOK_ANGLE_FRONT  115
#define LOOK_ANGLE_LEFT   170
#define LOOK_ANGLE_RIGHT  50

//
// STATES
//

// These are defined in a clock-wise manner.
//
// STOP = 0
//
//            FORWARD (STOP + 1)
//
// LEFT (BACKWARD + 1)              RIGHT (FORWARD + 1)
//
//            BACKWARD (RIGHT + 1)
//
//

// Same as #define, but grouped together. Enumerators
// don't allocate any memory
enum MoveDirection
{
  MOVE_STOP  = 0,
  MOVE_FORWARD,  // = 1
  MOVE_RIGHT,    // = 2
  MOVE_BACKWARD, // = 3
  MOVE_LEFT      // = 4
};

// Same as #define, but grouped together. Enumerators
// don't allocate any memory
enum LookDirection
{
  LOOK_FRONT = 0,
  LOOK_LEFT,   // = 1
  LOOK_RIGHT   // = 2
};

// angleFromLookDirection takes as input a LookDirection,
// and returns the angle for a motor
//
int angleFromLookDirection(LookDirection direction)
{
  int angle = LOOK_ANGLE_FRONT;
  switch (direction)
  {
  case LookDirection::LOOK_LEFT:
    angle = LOOK_ANGLE_LEFT;
    break;
  case LookDirection::LOOK_RIGHT:
    angle = LOOK_ANGLE_RIGHT;
    break;
  }
  return angle;
}

//
// VARIABLES
//
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

AF_DCMotor motors[NUM_MOTORS] =
{
  AF_DCMotor(1, MOTOR12_1KHZ),  // Motor::LEFT
  AF_DCMotor(2, MOTOR12_1KHZ),  // Motor::RIGHT
};

Servo myservo;

// The direction in which the vehicle is moving currently.
MoveDirection cmove_direction = MoveDirection::MOVE_STOP;


int distance = 100;
int speed = 0;

//
// FUNCTION PROTOTYPES
//
int readPing();
int lookTo(LookDirection, int);
void move(MoveDirection, int);

//
// SETUP
//
void setup()
{
  myservo.attach(SERVO_PIN);
  myservo.write(LOOK_ANGLE_FRONT);

  delay(2000);
  for (int i = 0; i < 4; i++)
  {
    distance = readPing();
    delay(100);
  }
}

//
// MAIN LOOP
//
void loop()
{
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(40);

  // No obstacle is observed in the close forward proximity,
  // keep moving
  if (distance > 15)
  {
    move(MoveDirection::MOVE_FORWARD, 0);
  }

  // If an object is within close proximity, do the look-around sequence; 
  // move a bit backwards and then look left and right to observe obstacles
  // around the vehicle
  else
  {
    // Stop
    move(MoveDirection::MOVE_STOP, 100);

    // Move a bit backwards, so the obstacles to the sides can be observed
    // more accurately
    move(MoveDirection::MOVE_BACKWARD, 300);

    // Stop again
    move(MoveDirection::MOVE_STOP, 200);

    // Observe the obstacles to the right
    distanceRight = lookTo(LookDirection::LOOK_RIGHT, 200);

    // Observe the obstacles to the left
    distanceLeft = lookTo(LookDirection::LOOK_LEFT, 200);

    // The right has a non-obstructed path
    if (distanceRight >= distanceLeft)
    {
      move(MoveDirection::MOVE_RIGHT, 500);
      move(MoveDirection::MOVE_STOP, 100);
    }

    // The left has a non-obstructed path
    else
    {
      move(MoveDirection::MOVE_LEFT, 500);
      move(MoveDirection::MOVE_STOP, 100);
    }
  }

  distance = readPing();
}

int readPing()
{
  delay(70);
  int cm = sonar.ping_cm();
  return (cm == 0 ? 250 : cm);
}

//
// Look takes as input the direction in which to look;
// LOOK_DIR_LEFT or LOOK_DIR_RIGHT. The duration is measured
// in milliseconds.
//
// Returns the distance on the side that was looked at.
//
int lookTo(LookDirection direction, int duration)
{
  int angle = angleFromLookDirection(direction);

  // Turn the servo to look either right or left
  myservo.write(angle);
  delay(500);

  // Read the distance
  int distance = readPing();
  delay(duration);

  // Turn the sensors back to the front
  myservo.write(LOOK_ANGLE_FRONT);
  delay(100);

  return distance;
}

// Moves to a certain direction for a certain amount of time.
// If you want the robot to move to a direction indefinitely,
// set duration to 0:
//
//    move(MoveDirection::MOVE_FORWARD, 0);
//
// will cause the robot to keep moving forward until move() is
// called again somewhere else.
//
void move(MoveDirection direction, int duration)
{
  // If the current movement direction is the same as the direction in which
  // you are trying to move, then don't bother with this function
  if (cmove_direction == direction)
  {
    return;
  }

  // Non-zero durations not allowed
  if (duration < 0)
  {
    duration = 5;
  }

  int run_direction = (direction == MoveDirection::MOVE_FORWARD ? FORWARD : BACKWARD);
  switch (direction)
  {
  case MoveDirection::MOVE_STOP:
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].run(RELEASE);
    }
    delay(duration);
    break;
  case MoveDirection::MOVE_FORWARD:
  case MoveDirection::MOVE_BACKWARD:
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].run(run_direction);
    }

    // Accelerate all motors gently
    for (int speed = 0; speed < MAX_SPEED; speed += 2)
    {
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        motors[i].setSpeed(speed);
      }
    }
    delay(duration);
    break;
  case MoveDirection::MOVE_RIGHT:
    // To move the vehicle to the right, the leftside wheels must
    // turn forward, and the rightside wheels backwards.
    motors[Motor::LEFT].run(FORWARD);
    motors[Motor::RIGHT].run(BACKWARD);

    delay(duration);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].run(FORWARD);
    }
    break;
  case MoveDirection::MOVE_LEFT:
    // To move the vehicle to the left, the leftside wheels must
    // turn forward, and the rightside wheels backwards.
    motors[Motor::LEFT].run(BACKWARD);
    motors[Motor::RIGHT].run(FORWARD);

    delay(duration);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].run(FORWARD);
    }
    break;
  }

  // The vehicle is currently moving to the input direction
  cmove_direction = direction;
}
