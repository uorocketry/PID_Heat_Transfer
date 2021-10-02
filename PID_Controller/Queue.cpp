#include "Queue.h"

Queue::Queue()
{
  front = 0;
  rear = -1;
}


bool Queue::empty()
{
  std::lock_guard<std::mutex> lock(m); // lock on creation
  if(rear == -1 || front > rear)
  {
    return true;
  }
  else
  {
    return false;
  }
} // unlock at destruction



Queue_rtd::Queue_rtd()
{

}


void Queue_rtd::enqueue(struct rtd temps)
{
  std::lock_guard<std::mutex> lock(m); // lock on creation
  if(rear == MAX_RTD-1)
  {
      //Serial.println("RTD QUEUE IS FULL!");
      // wrap around overwrite front of queue
      front = 0; 
      rear = -1;
  }

  // increment rear then assign value to index
  buff[++rear] = temps; 
} // unlock at destruction


struct rtd Queue_rtd::dequeue()
{ 
  std::lock_guard<std::mutex> lock(m); // lock on creation
  struct rtd temps = {-1,-1,-1,-1,true};

  if(rear == -1 || front > rear)
  {
      //Serial.println("RTD QUEUE IS EMPTY!");
      return temps;
  }

  temps = buff[front];
  front++;
  return temps;
} // unlock at destruction
