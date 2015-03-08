#include <limits.h>
#include <VirtualWire.h>
#include <Heartbeat.h>
#include "Node.h"
#include "PriorityQueue.h"

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
// RF Link kit
const int rfrxPin = 11;
const int rftxPin = 12;
byte count = 1;
// Onboard LED
const int onboardLED = 13;

// Photoresistor calibrations
/** LINE_THRESHOLD should be a number between 0 and 1, representing the "blackness" of line expected **/
const float LINE_THRESHOLD = 0.2;
short leftMin = SHRT_MAX;
short leftMax = SHRT_MIN;
short leftThreshold;
short rightMin = SHRT_MAX;
short rightMax = SHRT_MIN;
short rightThreshold;
short iMin1 = SHRT_MAX;
short iMax1 = SHRT_MIN;
short iThreshold1;
short iMin2 = SHRT_MAX;
short iMax2 = SHRT_MIN;
short iThreshold2;

// Motor calibrations
const int leftMotorSpeed = 255;
const int rightMotorSpeed = leftMotorSpeed - 15;
const float TURNING_RATIO = 0.80;
const int turnStartDelay = 500;

// Line following state machine
boolean straightTurn = false;
boolean rightTurn = false;
boolean leftTurn = false;
boolean aboutTurn = false;
unsigned long lastIntersection = 0;

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

void setup()
{
  // Init pin setups
  pinMode(leftDirection, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(onboardLED, OUTPUT);
  vw_set_rx_pin(rfrxPin);
  vw_set_tx_pin(rftxPin);
  vw_setup(2000);
  vw_rx_start();
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
  unsigned long startTime = millis();
  unsigned long calTime = 0; // Time that we have been calibrating
  while(true){
    short left = analogRead(leftPin);
    short right = analogRead(rightPin);
    short i1 = analogRead(iPin1);
    short i2 = analogRead(iPin2);

    leftMin = min(left, leftMin);
    leftMax = max(left, leftMax);
    rightMin = min(right, rightMin);
    rightMax = max(right, rightMax);
    iMin1 = min(i1, iMin1);
    iMax1 = max(i1, iMax1);
    iMin2 = min(i2, iMin2);
    iMax2 = max(i2, iMax2);

    calTime = millis() - startTime;
    if (calTime < 1000){
      // Straight
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, leftMotorSpeed);
      analogWrite(rightMotor, rightMotorSpeed);
    }
    else if (calTime < 2000){
      // Left turn
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, 0);
      analogWrite(rightMotor, rightMotorSpeed);
    }
    else{
      break;
    }
    delay(100);
  }
  Heartbeat.sendMonitor("Done calibrating!");
  // Calculate thresholds
  leftThreshold = (leftMax - leftMin) * LINE_THRESHOLD;
  rightThreshold = (rightMax - rightMin) * LINE_THRESHOLD;
  iThreshold1 = (iMax1 - iMin1) * LINE_THRESHOLD;
  iThreshold2 = (iMax2 - iMin2) * LINE_THRESHOLD;

  // Send calibrations
  Heartbeat.write(byte(21));
  Heartbeat.write(byte(2*8));
  Heartbeat.write(leftMin);
  Heartbeat.write(leftMax);
  Heartbeat.write(rightMin);
  Heartbeat.write(rightMax);
  Heartbeat.write(iMin1);
  Heartbeat.write(iMax1);
  Heartbeat.write(iMin2);
  Heartbeat.write(iMax2);

  findPath(0, 0, 5, 1);

  sendGrid(gridNodes);

  // Wait
  digitalWrite(leftDirection, HIGH);
  digitalWrite(rightDirection, HIGH);
  analogWrite(leftMotor, 0);
  analogWrite(rightMotor, 0);

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
  const byte left = analogRead(leftPin);
  const byte right = analogRead(rightPin);
  const byte int1 = analogRead(iPin1);
  const byte int2 = analogRead(iPin2);
  // Send to computer
  Heartbeat.write(byte(24));
  Heartbeat.write(byte(4));
  Heartbeat.write(left);
  Heartbeat.write(right);
  Heartbeat.write(int1);
  Heartbeat.write(int2);

  if (straightTurn){
    // Go straight across an intersection
    digitalWrite(leftDirection, HIGH);
    digitalWrite(rightDirection, HIGH);
    analogWrite(leftMotor, leftMotorSpeed);
    analogWrite(rightMotor, rightMotorSpeed);
    Heartbeat.sendMonitor("Straight Turn");
    if (millis() - lastIntersection > 500 && right >= rightThreshold){
      straightTurn = false;
    }
  }
  else if (rightTurn){
    // Making a turn
    digitalWrite(leftDirection, HIGH);
    digitalWrite(rightDirection, LOW);
    analogWrite(leftMotor, leftMotorSpeed);
    analogWrite(rightMotor, TURNING_RATIO * rightMotorSpeed);
    Heartbeat.sendMonitor("Right Turn");

    if (millis() - lastIntersection > turnStartDelay && left >= leftThreshold){
      rightTurn = false;
      // Update facing
      facing ++;
      if (facing > 3){
        facing = 0;
      }
      // Heartbeat(facing)
      Heartbeat.sendByte(14, facing);
    }
  }
  else if (leftTurn){
    // Making a turn
    digitalWrite(leftDirection, LOW);
    digitalWrite(rightDirection, HIGH);
    analogWrite(leftMotor, TURNING_RATIO * leftMotorSpeed);
    analogWrite(rightMotor, rightMotorSpeed);
    Heartbeat.sendMonitor("Left Turn");

    if (millis() - lastIntersection > turnStartDelay && right >= rightThreshold){
      leftTurn = false;
      // Update facing
      facing --;
      if (facing < 0){
        facing = 3;
      }
      // Heartbeat(facing)
      Heartbeat.sendByte(14, facing);
    }
  }
  else if (aboutTurn){
    // Making a turn
    digitalWrite(leftDirection, HIGH);
    digitalWrite(rightDirection, LOW);
    analogWrite(leftMotor, leftMotorSpeed);
    analogWrite(rightMotor, TURNING_RATIO * rightMotorSpeed);
    Heartbeat.sendMonitor("About Turn");

    if (millis() - lastIntersection > turnStartDelay && left >= leftThreshold){
      aboutTurn = false;
      rightTurn = true;
      lastIntersection = millis();
      // Update facing
      facing ++;
      if (facing > 3){
        facing = 0;
      }
      // Heartbeat(facing)
      Heartbeat.sendByte(14, facing);
    }
  }
  else
  {
    // Following the line
    if (left < leftThreshold && right < rightThreshold){
      // Both white, go right
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, 0);
      analogWrite(rightMotor, 0);
      Heartbeat.sendMonitor("Lost");
    }
    else if (left < leftThreshold && right >= rightThreshold){
      // Go left
      digitalWrite(leftDirection, LOW);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, 2/3 * leftMotorSpeed);
      analogWrite(rightMotor, rightMotorSpeed);
      Heartbeat.sendMonitor("Left");
    }
    else if (left >= leftThreshold && right < rightThreshold){
      // Go right
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, LOW);
      analogWrite(leftMotor, leftMotorSpeed);
      analogWrite(rightMotor, 2/3 * rightMotorSpeed);
      Heartbeat.sendMonitor("Right");
    }    
    else{
      // Intersection
      if (int1 >= iThreshold1 && int2 >= iThreshold2){
        Heartbeat.sendMonitor("Intersection detected");
        // Update current location
        switch(facing){
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
        else if (dx == 1){
          reqFacing = EAST;
        }
        else if (dy == -1){
          reqFacing = SOUTH;
        }        
        else if (dx == -1){
          reqFacing = WEST;
        }
        else{
          Heartbeat.sendMonitor("Pathfinder attempted impossible turn");
        }
        char turnReq = reqFacing - facing;
        if (turnReq == 1 || turnReq == -3){
          rightTurn = true;
          Heartbeat.sendMonitor("Turning right...");
        }
        else if (turnReq == -1 || turnReq == 3){
          leftTurn = true;
          Heartbeat.sendMonitor("Turning left...");
        }
        else if (turnReq == 2 || turnReq == -2){
          aboutTurn = true;
          Heartbeat.sendMonitor("Turning around...");
        }
        else{
          straightTurn = true;
          Heartbeat.sendMonitor("Going straight...");
        }
        // TODO debug
        leftTurn = true;
        rightTurn = false;
        straightTurn = false;
        aboutTurn = false;
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
    }
  } 

  Heartbeat.sendHeartbeat();
  // RF Send
  byte msg[1] = {
    0  };
  msg[0] = count++;
  vw_send((uint8_t *)msg, 1);

  // RF Receive
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    // Message with a good checksum received, dump it.
    Heartbeat.sendMonitor("Got: ");
    Heartbeat.sendMonitor((char*)buf);
    digitalWrite(onboardLED, HIGH);
  }
  delay(1000);
  digitalWrite(onboardLED, LOW);
}


void sendGrid(Node gridNodes[7][7])
{
  Heartbeat.write(byte(10));
  Heartbeat.write(byte(7*7*2));
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
  Heartbeat.write(byte(7*7*2));
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
  Heartbeat.write(byte(7*7*3));
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
  switch(id){
  case 6:
    x = ((char*)data)[0];
    y = ((char*)data)[1];
    Heartbeat.sendMonitor("Obstacle added: (" + String(x + 0) + ", " + String(y + 0) + ")");
    break;
  }
}





