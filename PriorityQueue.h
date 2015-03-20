#pragma once
#include <Arduino.h>
#include "Node.h"

class PriorityQueue
{
  private:
    Node* head;
  public:
    PriorityQueue();
    ~PriorityQueue();
    bool contains(Node*) const;
    void insert(Node*);
    Node* pop();
    void clear();
    String toString() const;
};
