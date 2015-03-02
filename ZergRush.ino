#include <limits.h>
#include <Heartbeat.h>
#include "Node.h"
#include "PriorityQueue.h"

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

// DEBUG: Onboard LED
const int onboardLED = 13;

// Photoresistor calibrations
const int LINE_THRESHOLD = 50;
short leftMin = SHRT_MAX;
short leftMax = SHRT_MIN;
short rightMin = SHRT_MAX;
short rightMax = SHRT_MIN;
short iMin1 = SHRT_MAX;
short iMax1 = SHRT_MIN;
short iMin2 = SHRT_MAX;
short iMax2 = SHRT_MIN;

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

int numIntersections = 0;

// Pathfinding
PriorityQueue openSet;
Node gridNodes[7][7];

void setup()
{
  pinMode(leftDirection, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(onboardLED, OUTPUT);

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
  digitalWrite(leftDirection, HIGH);
  digitalWrite(rightDirection, HIGH);
  analogWrite(leftMotor, 100);
  analogWrite(rightMotor, 100);
  unsigned long startTime = millis();
  unsigned long calTime = 0; // Time that we have been calibrating
  while((calTime = millis() - startTime) < 5000){
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
    delay(100);
  }
  Heartbeat.sendMonitor("Done calibrating!");
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
  const byte left = map(analogRead(leftPin), leftMin, leftMax, 0, 100);
  const byte right = map(analogRead(rightPin), rightMin, rightMax, 0, 100);
  const byte int1 = map(analogRead(iPin1), iMin1, iMax1, 0, 100);
  const byte int2 =  map(analogRead(iPin2), iMin2, iMax2, 0, 100);
  const byte intersection = min(int1, int2);
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
    analogWrite(leftMotor, 255);
    analogWrite(rightMotor, 255);
    Heartbeat.sendMonitor("Straight Turn");
    if (millis() - lastIntersection > 500 && right >= LINE_THRESHOLD){
      straightTurn = false;
    }
  }
  else if (rightTurn){
    // Making a turn
    digitalWrite(leftDirection, HIGH);
    digitalWrite(rightDirection, LOW);
    analogWrite(leftMotor, 255);
    analogWrite(rightMotor, 25);
    Heartbeat.sendMonitor("Right Turn");

    if (millis() - lastIntersection > 500 && right >= LINE_THRESHOLD){
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
    analogWrite(leftMotor, 25);
    analogWrite(rightMotor, 255);
    Heartbeat.sendMonitor("Left Turn");

    if (millis() - lastIntersection > 500 && left >= LINE_THRESHOLD){
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
    analogWrite(leftMotor, 255);
    analogWrite(rightMotor, 25);
    Heartbeat.sendMonitor("About Turn");

    if (millis() - lastIntersection > 500 && left >= LINE_THRESHOLD){
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
    if (left < LINE_THRESHOLD && right < LINE_THRESHOLD){
      // Both white, go right
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, 255);
      analogWrite(rightMotor, 0);
      Heartbeat.sendMonitor("Lost");
    }
    else if (left < LINE_THRESHOLD && right >= LINE_THRESHOLD){
      // Go left
      digitalWrite(leftDirection, LOW);
      digitalWrite(rightDirection, HIGH);
      analogWrite(leftMotor, 255);
      analogWrite(rightMotor, 255);
      Heartbeat.sendMonitor("Left");
    }
    else if (left >= LINE_THRESHOLD && right < LINE_THRESHOLD){
      // Go right
      digitalWrite(leftDirection, HIGH);
      digitalWrite(rightDirection, LOW);
      analogWrite(leftMotor, 255);
      analogWrite(rightMotor, 255);
      Heartbeat.sendMonitor("Right");
    }    
    else{
      // Intersection
      if (intersection >= LINE_THRESHOLD){
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
      }
      else
      {
        // Go straight
        digitalWrite(leftDirection, HIGH);
        digitalWrite(rightDirection, HIGH);
        analogWrite(leftMotor, 255);
        analogWrite(rightMotor, 255);
        Heartbeat.sendMonitor("Straight");
      }
    }
  } 

  Heartbeat.sendHeartbeat();
  delay(500);
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











