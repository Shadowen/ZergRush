#include "PriorityQueue.h"

PriorityQueue::PriorityQueue()
{
  head = new Node();
  head->heuristicDistance = -1;
}

PriorityQueue::~PriorityQueue()
{
  delete head;
}

bool PriorityQueue::contains(Node* toSearch) const
{
  // Go through list
  Node* node = head;
  while (node->next != NULL)
  {
    if (toSearch->x == node->next->x && toSearch->y == node->next->y)
    {
      return true;
    }
    node = node->next;
  }
  return false;
}

void PriorityQueue::insert(Node* toInsert)
{
  toInsert->next = head->next;
  head->next = toInsert;
}

Node* PriorityQueue::pop()
{
  Node* previous = head;
  Node* node = head->next;
  Node* toReturnParent = NULL;
  Node* toReturn = NULL;
  // Find the node to return
  while (node != NULL)
  {
    if (toReturn == NULL || node->heuristicDistance < toReturn->heuristicDistance)
    {
      toReturnParent = previous;
      toReturn = node;
    }
    previous = node;
    node = node->next;
  }
  // Remove toReturn
  if (toReturn != NULL)
  {
    toReturnParent->next = toReturn->next;
  }
  return toReturn;
}

void PriorityQueue::clear()
{
  head->next = NULL;
}

String PriorityQueue::toString() const
{
  String s = "";
  Node* node = head->next;
  while (node != NULL)
  {
    s += String(node->heuristicDistance) + "(" + String(node->x) + ", " + String(node->y) + ") ";
    node = node->next;
  }
  return s;
}

