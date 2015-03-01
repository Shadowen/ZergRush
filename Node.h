#ifndef NODE
#define NODE

#include <Arduino.h>

class Node
{
public:
	unsigned char x;
	unsigned char y;
	unsigned char distanceFromStart;
	unsigned char heuristicDistance;
	
	// For A*
	Node* parent;
	bool isOpen;
	bool isClosed;
	// For priority queue
	Node* next;
	
	Node(){
		parent = NULL;
		next = NULL;
	}
	
	inline String toString()
	{
		return "(" + String(x) + ", " + String(y) + ")";
	}
};

#endif
