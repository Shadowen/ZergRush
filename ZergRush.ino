#include <limits.h>
#include <Heartbeat.h>
#include "Node.h"
#include "PriorityQueue.h"
#include "Hopper.h"

// Pin assignments
// Photoresistor pins
const int iPin1 = A0;
const int leftPin = A1;
const int rightPin = A2;
const int iPin2 = A3;
// Motor pins
const int leftDirection = 4;
const int leftMotor = 5;
const int rightDirection = 7;
const int rightMotor = 6;
// Encoders - interrupts are pins 2 and 3 (INT0, INT1);
const int leftEncoderInterruptPin = 0;
const int rightEncoderInterruptPin = 1;
const int leftEncoderPin = 2;
const int rightEncoderPin = 3;
// Onboard LED
const int onboardLED = 13;

// Photoresistor calibrations
/** LINE_THRESHOLD should be a number between 0 and 1, representing the "blackness" of line expected **/
const float LINE_THRESHOLD = 0.3;
short leftMin;
short leftMax;
short leftThreshold;
short rightMin;
short rightMax;
short rightThreshold;
short iMin1;
short iMax1;
short iThreshold1;
short iMin2;
short iMax2;
short iThreshold2;

// Motor calibrations
const short leftMotorSpeed = 200;
const short rightMotorSpeed = leftMotorSpeed;
const float TURNING_RATIO = 0.8;
const short turnStartDelay = 500;

// Line following state machine
const char STATE_LINE_FOLLOWING = 0;
const char STATE_STRAIGHT = 1;
const char STATE_LEFT_TURN = 2;
const char STATE_RIGHT_TURN = 3;
const char STATE_ABOUT_TURN = 4;
char robotState = STATE_LINE_FOLLOWING;
// Time at which we last saw an intersection
unsigned long lastIntersection = 0;

// Encoder counts
volatile int leftCount = 0;
volatile int rightCount = 0;

// Navigation
// Which cardinal direction the robot is facing
const char NORTH = 0;
const char EAST = 1;
const char SOUTH = 2;
const char WEST = 3;
char facing;
// What grid coordinates am I at?
byte x;
byte y;
// Counter that keeps track of how many intersections we've passed
int numIntersections = 0;

// Pathfinding
// The nodes themselves
Node gridNodes[7][7];
// Data structure that keeps track of the nodes left to check
PriorityQueue openSet;

// Hopper tracking
// Conversion factors from hopper to actual coordinates
const char xScale = 20;
const char xOffset = 42;
const char yScale = 20;
const char yOffset = 58;
// Hopper variables
Hopper hoppers[2];
char countHoppers = 0;

void setup()
{
  // Init pin setups
  pinMode(leftDirection, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(onboardLED, OUTPUT);
  attachInterrupt(leftEncoderInterruptPin, leftEncoderISR, RISING);
  attachInterrupt(rightEncoderInterruptPin, rightEncoderISR, RISING);
  // Start Heartbeat
  Heartbeat.begin(callback);

  for (byte x = 0; x < 7; x++)
  {
    for (byte y = 0; y < 7; y++)
    {
      gridNodes[x][y].x = x;
      gridNodes[x][y].y = y;
      gridNodes[x][y].distanceFromStart = 0;
      gridNodes[x][y].heuristicDistance = 0;
    }
  }

  Heartbeat.sendMonitor("Starting calibration...");
  leftMin = SHRT_MAX;
  leftMax = SHRT_MIN;
  rightMin = SHRT_MAX;
  rightMax = SHRT_MIN;
  iMin1 = SHRT_MAX;
  iMax1 = SHRT_MIN;
  iMin2 = SHRT_MAX;
  iMax2 = SHRT_MIN;
  unsigned long startTime = millis();
  while (true) {
    short left = analogRead(leftPin);
    short right = analogRead(rightPin);
    short i1 = analogRead(iPin1);
    short i2 = analogRead(iPin2);
    // Send to computer
    Heartbeat.write(24);
    Heartbeat.write(8);
    Heartbeat.writeShort(left);
    Heartbeat.writeShort(right);
    Heartbeat.writeShort(i1);
    Heartbeat.writeShort(i2);

    leftMin = min(left, leftMin);
    leftMax = max(left, leftMax);
    rightMin = min(right, rightMin);
    rightMax = max(right, rightMax);
    iMin1 = min(i1, iMin1);
    iMax1 = max(i1, iMax1);
    iMin2 = min(i2, iMin2);
    iMax2 = max(i2, iMax2);

    // Time since we have started the calibration
    unsigned long calTime = millis() - startTime;
    if (calTime < 500) {
      // Forward
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, leftMotorSpeed);
      analogWrite(rightMotor, rightMotorSpeed);
    }
    else if (calTime < 1000)
    {
      // Backward
      digitalWrite(leftDirection, LOW);
      digitalWrite(rightDirection, LOW);
      analogWrite(leftMotor, leftMotorSpeed);
      analogWrite(rightMotor, rightMotorSpeed);
    }
    else
    {
      break;
    }
    delay(10);
  }
  // Wait
  digitalWrite(leftDirection, HIGH);
  digitalWrite(rightDirection, HIGH);
  analogWrite(leftMotor, 0);
  analogWrite(rightMotor, 0);
  Heartbeat.sendMonitor("Done calibrating!");
  // Calculate thresholds
  leftThreshold = (leftMax - leftMin) * LINE_THRESHOLD + leftMin;
  rightThreshold = (rightMax - rightMin) * LINE_THRESHOLD + rightMin;
  iThreshold1 = (iMax1 - iMin1) * LINE_THRESHOLD + iMin1;
  iThreshold2 = (iMax2 - iMin2) * LINE_THRESHOLD + iMin2;
  // Send thresholds to Heartbeat
  Heartbeat.write(21);
  Heartbeat.write(8);
  Heartbeat.writeShort(leftThreshold);
  Heartbeat.writeShort(rightThreshold);
  Heartbeat.writeShort(iThreshold1);
  Heartbeat.writeShort(iThreshold2);

  // TODO use pathfinding
  findPath(0, 0, 5, 1);
  sendGrid(gridNodes);

  delay(2000);
}

void findPath(const byte& startX, const byte& startY, const byte& destX, const byte& destY)
{
  gridNodes[destX][destY].distanceFromStart = 0;
  gridNodes[destX][destY].heuristicDistance = abs(destX - startX) + abs(destY - startY);
  // Open set
  openSet.clear();
  openSet.insert(&gridNodes[destX][destY]);
  Heartbeat.sendByte(5, destX);
  Heartbeat.sendByte(6, destY);
  // Closed set
  for (byte x = 0; x < 7; x++)
  {
    for (byte y = 0; y < 7; y++)
    {
      gridNodes[x][y].isOpen = false;
      gridNodes[x][y].isClosed = false;
    }
  }
  gridNodes[1][0].isClosed = true;
  gridNodes[1][1].isClosed = true;
  gridNodes[1][2].isClosed = true;

  Node* currentNode;
  while ((currentNode = openSet.pop()) != NULL)
  {
    currentNode->isClosed = true;
    const byte& x = currentNode->x;
    const byte& y = currentNode->y;

    // Allow Heartbeat to see the progress of the pathfinder
    // WARNING: Significantly slows down pathfinder
    //sendGrid(gridNodes);
    //Heartbeat.write(byte(13));
    //Heartbeat.write(byte(2));
    //Heartbeat.write(x);
    //Heartbeat.write(y);

    if (x == startX && y == startY)
    {
      Heartbeat.sendMonitor("Path found!");
      // Destination reached
      return;
    }

    checkNode(x + 1, y, startX, startY, currentNode);
    checkNode(x - 1, y, startX, startY, currentNode);
    checkNode(x, y + 1, startX, startY, currentNode);
    checkNode(x, y - 1, startX, startY, currentNode);
  }
  // No path found
  Heartbeat.sendMonitor("No path found :(");
}

void checkNode(const byte& x, const byte& y, const byte& startX, const byte& startY, Node* parent)
{
  // Invalid node
  if (x < 0 || x > 6 || y < 0 || y > 6 || gridNodes[x][y].isClosed)
  {
    return;
  }
  const byte distanceFromStart = parent->distanceFromStart + 1;
  const boolean inOpenSet = openSet.contains(&gridNodes[x][y]);
  if (!inOpenSet || distanceFromStart < gridNodes[x][y].distanceFromStart)
  {
    gridNodes[x][y].parent = parent;
    gridNodes[x][y].distanceFromStart = distanceFromStart;
    gridNodes[x][y].heuristicDistance = distanceFromStart + abs(x - startX) + abs(y - startY);
    gridNodes[x][y].isOpen = true;
    if (!inOpenSet)
    {
      openSet.insert(&gridNodes[x][y]);
    }
  }
}

void loop()
{
  // Read line follower sensors
  const short left = analogRead(leftPin);
  const short right = analogRead(rightPin);
  const short int1 = analogRead(iPin1);
  const short int2 = analogRead(iPin2);
  // Send sensor readings to Heartbeat
  Heartbeat.write(byte(24));
  Heartbeat.write(byte(8));
  Heartbeat.writeShort(left);
  Heartbeat.writeShort(right);
  Heartbeat.writeShort(int1);
  Heartbeat.writeShort(int2);

  switch (robotState) {
    case STATE_LINE_FOLLOWING:
      // Following the line
      if (left < leftThreshold && right < rightThreshold) {
        // Both white, lost
        digitalWrite(leftDirection, LOW);
        digitalWrite(rightDirection, LOW);
        analogWrite(leftMotor, 0);
        analogWrite(rightMotor, 0);
        Heartbeat.sendMonitor("Lost");
      }
      else if (left < leftThreshold && right >= rightThreshold) {
        // Go right
        digitalWrite(leftDirection, HIGH);
        digitalWrite(rightDirection, LOW);
        analogWrite(leftMotor, leftMotorSpeed);
        analogWrite(rightMotor, 2 / 3 * rightMotorSpeed);
        Heartbeat.sendMonitor("Right");
      }
      else if (left >= leftThreshold && right < rightThreshold) {
        // Go left
        digitalWrite(leftDirection, LOW);
        digitalWrite(rightDirection, HIGH);
        analogWrite(leftMotor, 2 / 3 * leftMotorSpeed);
        analogWrite(rightMotor, rightMotorSpeed);
        Heartbeat.sendMonitor("Left");
      }
      else {
        // Intersection
        if (int1 >= iThreshold1 && int2 >= iThreshold2) {
          Heartbeat.sendMonitor("Intersection detected");
          // Update current location
          switch (facing) {
            case NORTH:
              y++;
              break;
            case EAST:
              x ++;
              break;
            case SOUTH:
              y--;
              break;
            case WEST:
              x--;
              break;
          }
          lastIntersection = millis();
          numIntersections++;
          // Heartbeat(current position)
          Heartbeat.write(byte(13));
          Heartbeat.write(byte(2));
          Heartbeat.write(x);
          Heartbeat.write(y);

          // Make a decision on which way to turn by referring to Pathfinder
          const Node& currentNode = gridNodes[x][y];
          const Node& destNode = *(currentNode.parent);
          const char dx = destNode.x - currentNode.x;
          const char dy = destNode.y - currentNode.y;
          char reqFacing;
          if (dy == 1)
          {
            reqFacing = NORTH;
          }
          else if (dx == 1) {
            reqFacing = EAST;
          }
          else if (dy == -1) {
            reqFacing = SOUTH;
          }
          else if (dx == -1) {
            reqFacing = WEST;
          }
          else {
            Heartbeat.sendMonitor("Pathfinder attempted impossible turn");
          }
          char turnReq = reqFacing - facing;
          if (turnReq == 1 || turnReq == -3) {
            robotState = STATE_RIGHT_TURN;
            Heartbeat.sendMonitor("Turning right...");
          }
          else if (turnReq == -1 || turnReq == 3) {
            robotState = STATE_LEFT_TURN;
            Heartbeat.sendMonitor("Turning left...");
          }
          else if (turnReq == 2 || turnReq == -2) {
            robotState = STATE_ABOUT_TURN;
            Heartbeat.sendMonitor("Turning around...");
          }
          else {
            robotState = STATE_STRAIGHT;
            Heartbeat.sendMonitor("Going straight...");
          }
        }
        else
        {
          // Go straight
          digitalWrite(leftDirection, HIGH);
          digitalWrite(rightDirection, HIGH);
          analogWrite(leftMotor, leftMotorSpeed);
          analogWrite(rightMotor, rightMotorSpeed);
          Heartbeat.sendMonitor("Straight");
        }
        break;
      case STATE_STRAIGHT:
        // Go straight across an intersection
        digitalWrite(leftDirection, HIGH);
        digitalWrite(rightDirection, HIGH);
        analogWrite(leftMotor, leftMotorSpeed);
        analogWrite(rightMotor, rightMotorSpeed);
        Heartbeat.sendMonitor("Straight Turn");
        if (millis() - lastIntersection > turnStartDelay && int1 < iThreshold1 && int2 < iThreshold2) {
          robotState = STATE_LINE_FOLLOWING;
        }
        break;
      case STATE_RIGHT_TURN:
        // Making a turn
        digitalWrite(leftDirection, HIGH);
        digitalWrite(rightDirection, LOW);
        analogWrite(leftMotor, leftMotorSpeed);
        analogWrite(rightMotor, TURNING_RATIO * rightMotorSpeed);
        Heartbeat.sendMonitor("Right Turn");

        if (millis() - lastIntersection > turnStartDelay && left >= leftThreshold) {
          robotState = STATE_LINE_FOLLOWING;
          // Update facing
          facing ++;
          if (facing > 3) {
            facing = 0;
          }
          // Heartbeat(facing)
          Heartbeat.sendByte(14, facing);
        }
        break;
      case STATE_LEFT_TURN:
        // Making a turn
        digitalWrite(leftDirection, LOW);
        digitalWrite(rightDirection, HIGH);
        analogWrite(leftMotor, TURNING_RATIO * leftMotorSpeed);
        analogWrite(rightMotor, rightMotorSpeed);
        Heartbeat.sendMonitor("Left Turn");

        if (millis() - lastIntersection > turnStartDelay && right >= rightThreshold) {
          robotState = STATE_LINE_FOLLOWING;
          // Update facing
          facing --;
          if (facing < 0) {
            facing = 3;
          }
          // Heartbeat(facing)
          Heartbeat.sendByte(14, facing);
        }
        break;
      case STATE_ABOUT_TURN:
        // Making a turn
        digitalWrite(leftDirection, HIGH);
        digitalWrite(rightDirection, LOW);
        analogWrite(leftMotor, leftMotorSpeed);
        analogWrite(rightMotor, TURNING_RATIO * rightMotorSpeed);
        Heartbeat.sendMonitor("About Turn");

        if (millis() - lastIntersection > turnStartDelay && left >= leftThreshold) {
          robotState = STATE_RIGHT_TURN;
          lastIntersection = millis();
          // Update facing
          facing ++;
          if (facing > 3) {
            facing = 0;
          }
          // Heartbeat(facing)
          Heartbeat.sendByte(14, facing);
        }
        break;
      default:
        Heartbeat.sendMonitor("Invalid robot state!");
        // Invalid state
      }
  }

  Heartbeat.sendHeartbeat();
  delay(10);
}


void sendGrid(Node gridNodes[7][7])
{
  Heartbeat.write(byte(10));
  Heartbeat.write(byte(7 * 7 * 2));
  for (byte x = 0; x < 7; x++)
  {
    for (byte y = 0; y < 7; y++)
    {
      const Node& node = gridNodes[x][y];
      Heartbeat.write(x);
      Heartbeat.write(y);
    }
  }

  Heartbeat.write(byte(11));
  Heartbeat.write(byte(7 * 7 * 2));
  for (byte x = 0; x < 7; x++)
  {
    for (byte y = 0; y < 7; y++)
    {
      const Node& node = gridNodes[x][y];
      Heartbeat.write(node.distanceFromStart);
      Heartbeat.write(node.heuristicDistance);
    }
  }

  Heartbeat.write(byte(12));
  Heartbeat.write(byte(7 * 7 * 3));
  for (byte x = 0; x < 7; x++)
  {
    for (byte y = 0; y < 7; y++)
    {
      Node node = gridNodes[x][y];
      Heartbeat.write(node.parent->x);
      Heartbeat.write(node.parent->y);
      Heartbeat.write(byte(node.isClosed));
    }
  }
}

void callback(byte id, byte length, void* data)
{
  Heartbeat.sendMonitor("Serial received!");

  char x;
  char y;
  char orientation;
  switch (id) {
    case 6:
      x = ((char*)data)[0];
      y = ((char*)data)[1];
      orientation = *((bool *)(((char*)data) + 2));
      Heartbeat.sendMonitor("Obstacle added: (" + String(x + 0) + ", " + String(y + 0) + "[" + String(orientation) + "]" + ")");
      hoppers[countHoppers].x = x * xScale + xOffset;
      hoppers[countHoppers].y = y * yScale + yOffset;
      hoppers[countHoppers].isPointingUp = orientation;
      countHoppers++;
      break;
  }
}

void leftEncoderISR() {
  leftCount++;
}

void rightEncoderISR() {
  rightCount++;
}



