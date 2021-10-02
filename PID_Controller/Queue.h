#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>
#include "TeensyThreads.h"


// Queue Size, vars for RTD and IMU data 
#define MAX_RTD 30


struct rtd
{
  String date;   
  float t1; 
  float t2;
  float t3;
  int   pulseWidthModulation; 
  bool error;
};


// Base class
class Queue {
  
  protected:
    std::mutex m;
    int front;
    int rear;

  public:
    Queue();
    bool empty();
   
}; 

// Derived class
class Queue_rtd: public Queue
{

  public: 
    Queue_rtd();
    void enqueue(struct rtd temps);
    struct rtd dequeue();
    
  private:
    struct rtd buff[MAX_RTD];
    
};


#endif 
