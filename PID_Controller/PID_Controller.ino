/****************************************************************
 * Sensor Libraries used:
 *    Adafruit PT100/P1000 RTD Sensor w/MAX31865 library by Limor Fried/Ladyada @ Adafruit Industries
 * 
 ***************************************************************/
#include "Queue.h"
#include "Rtd.h"
#include "PID.h"

#include "TeensyThreads.h"
#include <Adafruit_MAX31865.h>
#include <SD.h>
#include <TimeLib.h>


// wrap around after 50 days
uint32_t RTD_TIME = 0; 

#define ENABLE_SD_CARD  true
#define ENABLE_RTD      true 
#define SERIAL_MONITOR  true 


// Adafruit_MAX31865 RTD 
Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(CS_PIN1);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(CS_PIN2);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(CS_PIN3);


// heater 
Heater heat(PWM_PIN);


// pid controller 
PID pid(KP_PID, kI_PID, KD_PID,
        TAU_PID,
        INTEGRATE_MAX_PID, INTEGRATE_MIN_PID,
        OUT_MAX_PID, OUT_MIN_PID,
        SAMPLE_TIME,
        SP_PID);


// queue for polling and logging sensor data 
Queue_rtd rtd_queue; 


bool Init_RTD(bool flag)
{
  if(flag)
  {
    thermo1.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo2.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo3.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    //Serial.println("Adafruit MAX31865 PT100 Sensors initialized");
    return true;
  }

  return false;
} // unlock at destruction


void LogToCSV(String dataString, const char* csvName)
{
   // log data 
   File dataFile = SD.open(csvName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
  dataFile.print(dataString);
  dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    //Serial.println("error opening file");
  }
}


void InitSdCard(bool flag)
{
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  if (flag)
  {
    // wait for serial to get ready 
    while (!Serial)
    {
      Serial.println("Waiting for SD card to initialize..."); 
      delay(1000);
    }

    // wait for SD card to get ready 
    while (!SD.begin(BUILTIN_SDCARD))
    {
      Serial.println("SD card initialization failed. Check:");
      Serial.println("1. is SD card inserted?");
      Serial.println("2. is your wiring correct?");
      Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
      Serial.println("");
      delay(1000);
    }
  
    // write header of csv
    LogToCSV(RTD_HEADER_CSV, RTD_CSV_NAME);
    //Serial.println("SD Card initialization done"); 
  }
}


void setup() 
{
  Serial.begin(115200);

  bool rtd_is_initialized = Init_RTD(ENABLE_RTD);
  // wait for 1.5 sec
  delay(1500);
  InitSdCard(ENABLE_SD_CARD); 

  // create thread to poll RTD 
  if (rtd_is_initialized)
  {
      std::thread t_rtd(PollRTD);
      t_rtd.detach();
  }

}


void loop()
{
  
  // log every "RTD_SAMPLE_PERIOD" ms
  if (millis() >= RTD_TIME + RTD_SAMPLE_PERIOD)
  {
    //Serial.println("  RTDLogger");
    LogRTD(RTD_CSV_NAME);
    RTD_TIME += RTD_SAMPLE_PERIOD;
  }
  
}


// sample rtd and input 
void PollRTD()
{

  float t1, t2, t3;
  String date; 
  int pidOutput;
  
  while(true)
  {
    //Serial.println("PollRTD");
    t1 = GetTemp(&thermo1);
    t2 = GetTemp(&thermo2);
    t3 = GetTemp(&thermo3);

    // control heater 
    pidOutput = pid.control(t3);
    heat.pwm(pidOutput);
  
    // serial plotter 
    if (SERIAL_MONITOR)
    {
      Serial.print(t1);
      Serial.print("\t");
      Serial.print(t2);
      Serial.print("\t");
      Serial.print(t3);
      Serial.print("\t");
      Serial.println(pidOutput);
    }
   
    date = TimeStr();
    struct rtd temps = {date, t1, t2, t3, pidOutput, false};

    rtd_queue.enqueue(temps);
    threads.delay(RTD_SAMPLE_PERIOD);
    threads.yield();
  }
} 


void LogRTD(const char *csvName)
{
    struct rtd temps = rtd_queue.dequeue();
    if(temps.error == false && temps.date != "-1")
    {      
      // log data 
      File dataFile = SD.open(csvName, FILE_WRITE);
      
      // if the file is available, write to it:
      if (dataFile) 
      {
        // yyyy-mm-dd hh:mm:ss
        dataFile.print(temps.date);
        dataFile.print(",");
        dataFile.print(temps.t1);
        dataFile.print(",");
        dataFile.print(temps.t2);
        dataFile.print(",");
        dataFile.print(temps.t3);
        dataFile.print(",");
        dataFile.print(temps.pulseWidthModulation);
        dataFile.print("\n");
        dataFile.close();
      }
      // if the file isn't open, pop up an error:
      else 
      {
        //Serial.println("error opening RTD_CSV_NAME");
      }
    }
} 
