
/*

  Seismograph 2 - really a strong motion acclerograph
  
  Features
   - uses Bosch BMA180 3-axis accelerometer
   - stores event in a file whne an event is detected
   - ring buffer stores data from before the event start
   - stores data on an SD card
   - status lights show state of data colection and storage
   - 16x2 LCD for basic status info
  

  ToDo

   - experiment with low-pass filter settings
   - review electrical setup to see if noise floor can be lowered 
   - better re-zero calibration - use data in ring buffer
   - web server admin and data retrieval (maybe)

    
  Can poll the acceleromter 1100/sec
 
--
Copyright (C) 2011 by Integrated Mapping Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


*/

#include <Wire.h>
#include <bma180.h>
#include <LiquidCrystal.h>

#include <SdFat.h>
#include <SdFatUtil.h>
#include <TimerTwo.h>

#include <Time.h>  
#include <DS1307RTC.h>  // DS1307 lib is close enough for the DS3231 in the chronodot


// SPI CS pins
const int16_t CS_SD = 4;
const int16_t CS_ETHERNET = 10;

// uncomment for debugging messages on the serial port
//#define DEBUG 1

/*-------------------------------------------------------------------------------------
  the timer interrupt has limits as to how long the interval can be, so the timer
  interrupt is triggered every TICK_USEC microseconds and the data is captured
  every TICKS_PER_LOG called to the readBMA180 interrupt service routine, giving
  LOG_INTERVAL_USEC microseconds between samples.
  
*/

// interval in usec between accelerometer samples
const uint32_t LOG_INTERVAL_USEC = 5000L;

// interval between timer interrupts in microseconds
const uint16_t TICK_USEC = 500;

// number of ticks between data points
const uint16_t TICKS_PER_LOG = LOG_INTERVAL_USEC / TICK_USEC;

// keep writing data this many milliseconds after no more signal is detected
const int32_t EVENT_TIMEOUT = 5000L;

/*-------------------------------------------------------------------------------------*/
// File system object
SdFat sd;

// file for logging data
SdFile file;

char fileName[13];

// store error strings in flash
#define error(s) sd.errorHalt_P(PSTR(s));

// flush the SD buffer this many data points
const int16_t SYNC_COUNT = 100;



/*-------------------------------------------------------------------------------------*/
// ring buffer definitions. head points to the location in the buffer beyond the highest
// filled data location, therefore the buffer contins on empty cell. whne head==tail, the
// buffer is empty

// structure to store the reading. 'serial' is a sequential serial number for each reading.
// useful for spotting problems with dropped samples or ring buffer bugs
typedef struct {
  int16_t serial;
  int16_t x;
  int16_t y;
  int16_t z;
} accReading_t;


// ring buffer for binary accelerometer data
const uint16_t RING_SIZE = 400;
accReading_t ring[RING_SIZE];

// pointers to the ring buffer
volatile uint16_t head = 0;
volatile uint16_t tail = 0;

// number of samples available in the ring buffer
inline uint16_t ringAvailable() {
  return (head >= tail ? 0 : RING_SIZE) + head - tail;
}

// next value for head/tail
inline uint16_t ringGetNext(uint16_t i) {return i < (RING_SIZE - 1) ? i + 1 : 0;}



/*-------------------------------------------------------------------------------------*/
BMA180 bma180;

// use this many samples to calibrate the sensor no-signal readings
const int16_t calibrationSamples = 200;

// set the recording trigger threshold to max/min deviation from mean, multiplied by this
const float thresholdFactor = 1.9;


// flag to signal and event has been detected
volatile uint8_t eventDetected = 0;


// stores the accelerometer no-signal offsets
int32_t accX0;
int32_t accY0;
int32_t accZ0;

// sampling trigger thresholds
int32_t accX0Threshold;
int32_t accY0Threshold;
int32_t accZ0Threshold;

// must be at least this many consecutive counts > the x,y or z threshold
static uint8_t consectiveEventThreshold = 3;


/*-------------------------------------------------------------------------------------*/
// stop pushbutton
const int16_t pinStopSwitch = 5;

// indicates ring buffer full
const int16_t pinRedLED = 22;         

// indicates data being written to the SD card
const int16_t pinGreenLED = 23;


// Connect via SPI. Data pin is #3, Clock is #2 and Latch is #4
LiquidCrystal lcd(30, 31, 32);
  
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
  

/*=====================================================================================*/
void setup()
{

  
#ifdef DEBUG
  Serial.begin(9600);
#endif
    
  initUI();
  
  // enable the SD card on Ethernet Shield
  pinMode(CS_SD, OUTPUT);
  pinMode(CS_ETHERNET, OUTPUT);
  digitalWrite(CS_SD, LOW);
  digitalWrite(CS_ETHERNET, HIGH);
   
  // enable  I2C
  Wire.begin();
  
  msgStatus("Calibrating");
  
  syncTime();
  
  initBMA180();
  
  // initialize file system.  If chipselect is not SS use sd.init(csPin).
  if (!sd.init(CS_SD)) sd.initErrorHalt();

  // create a file to store the data
  openNewFile();  
  
#ifdef DEBUG
  Serial.println(FreeRam());
  Serial.println(TICKS_PER_LOG);
#endif

  // initialise the interrupt timer
  if (!TimerTwo::init(TICK_USEC, readBMA180, false) || TICK_USEC != TimerTwo::period()) {
    // TICK_TIME_USEC is too large or period rounds to a different value
    error("TimerTwo::init");
  }
  
  // clear the indicator lights to show we are ready to go
  digitalWrite(pinRedLED, LOW);
  digitalWrite(pinGreenLED, LOW);  
  msgReady();

  // start sampling
  TimerTwo::start();

}




/*=====================================================================================*/
void loop()
{
 
  static uint8_t sampleCt = 0;                         // samples written to the SD card
  static uint32_t eventStart = -(EVENT_TIMEOUT+100);   // time in milliseconds the event started
  static uint8_t lastEventState = 0;                   // tracks if an event was in progress last time 'loop' was called
  int inEvent = 0;                                     // true is an event is in progress
  char buf[35];

  // how many samples are available in the ring buffer
  cli();
  uint16_t n = ringAvailable();
  sei();
 
  // if an event was detected, write the time to the start of the file
  // if it's the start of an event. clear eventDetected and record the start time
  // in the file
  if (eventDetected) {
    if (!lastEventState) {
      
      // start time of data is now (st + frac) - number of points in the buffer (offset ms)
      time_t st = now();
      uint32_t frac = millis() % 1000L;
      uint32_t offset = ((n * LOG_INTERVAL_USEC)/1000.0);

      sprintf(buf ,"START %lu %lu %lu", st, frac, offset);
      file.println(buf);   
      
      sprintf(buf ,"%d-%02d-%02d %02d:%02d:%02d", year(st), month(st), day(st), hour(st), minute(st), second(st));
      file.println(buf);   
      
    }
    eventDetected = 0;
    eventStart = millis();  
  }

  // in event if timeout hasn't been reached
  inEvent = (millis() - eventStart < EVENT_TIMEOUT);

  // if in an event and there is data, write it out
  if (inEvent && (n > 0)) {
    
    // signal in an event and writing data
    digitalWrite(pinGreenLED, HIGH);
  
    // keep going till the buffer is empty. this loop will empty the 400 sample
    // buffer in 30-50ms, i.e. has no trouble keeping up at 200Hz sample rate
    for (uint16_t i = 0; i < n; i++) {
   
      sprintf(buf, "%d\t %d\t %d\t %d\t %d\t %u\n", ring[tail].x, ring[tail].y, ring[tail].z, head, tail, ring[tail].serial);
      
      if (!file.write(buf, strlen(buf))) error("file.write");
      
      if (sampleCt++ > SYNC_COUNT) {
        if (!file.sync()) error("file.sync");
        sampleCt = 0;
      }
      
      // get next sample - disable interrupts to make it atomic
      cli();
      tail = ringGetNext(tail);
      sei();
      
    }
    
  } else
    digitalWrite(pinGreenLED, LOW);
    
    
  // house keeping if not in an event
  if (!inEvent) {
    
    // if event just ended
    if (lastEventState) 
      handleEventEnded();
 
  }  
    

  // pressed the stop switch, sync the file and close it and shut down.
  if (digitalRead(pinStopSwitch) == LOW) 
    doStop();
  
  lastEventState = inEvent;

}


/*-------------------------------------------------------------------------------------*/
void handleEventEnded() {
  
  // event just ended. close the data file, open another one and
  // recalibrate the accelerometer. note that the sampling interrupt
  // must be diabled for this

  // close the file and open another
  if (!file.sync()) error("file.sync");
  if (!file.close()) error("file.close");
  
  // disable sampling interrupt. need to use RTC clock and re-zero the accelerometer
  TimerTwo::stop();
  
  openNewFile();
  msgData(fileName);
  
  // redo calibration ** REFACTOR so uses data from the ring buffer
  initReferences();
  
  // re-enable sample interrupts
  TimerTwo::start();
  
  msgReady();  

}



/*-------------------------------------------------------------------------------------*/
void doStop() {
    // close the data file and stop the program

  if (!file.sync()) error("file.sync");
  if (!file.close()) error("file.close");
  cli();
  
  PgmPrintln("Stopped!");
  msgStatus("Stopped!");
  
  digitalWrite(pinRedLED, LOW);
  digitalWrite(pinGreenLED, LOW);
  
  while (1);
}




/*-------------------------------------------------------------------------------------*/
void syncTime() {
  // get time from the RTC clock chip
  
  // this sets the RTC clock as the time provider and forces a
  // the time library to sync its time from the RTC
  setSyncProvider(RTC.get);
  
#ifdef DEBUG
  if(timeStatus()!= timeSet) 
     Serial.println("Unable to sync with the RTC");
  else {
     Serial.println("RTC has set the system time");  
  }  
#endif

  // disable time sync'ing - we don't want conflicts with the accelerometer
  setSyncProvider(0);

}





/*-------------------------------------------------------------------------------------*/
void msgStatus(char *msg) {
  
  // status messages on the top line of the display
  lcd.setCursor(0, 0);
  lcd.print(msg); 
  
  for (int i = strlen(msg); i < LCD_WIDTH-1; i++)  
    lcd.print(" ");

}


/*-------------------------------------------------------------------------------------*/
void msgData(char *msg) {
  
  // data messages on the bottom line of the display
  lcd.setCursor(0, 1);
  lcd.print(msg); 
  
  for (int i = strlen(msg); i < LCD_WIDTH-1; i++)  
    lcd.print(" ");

}


/*-------------------------------------------------------------------------------------*/
void msgReady() {
 
  // print ready message and trigger thresholds
  char buf[20];
  sprintf(buf, "Rdy %3lu %3lu %3lu", accX0Threshold, accY0Threshold, accZ0Threshold); 
  msgStatus(buf);
  
  
}


/*-------------------------------------------------------------------------------------*/
void initUI() {
  
  // initialise indicator lights 
  pinMode(pinRedLED, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);
  digitalWrite(pinRedLED, HIGH);
  digitalWrite(pinGreenLED, HIGH);  
  
  // stop switch
  pinMode(pinStopSwitch, INPUT);
  digitalWrite(pinStopSwitch, HIGH); 
  
  //lcd
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  msgStatus("Startup");
  
}


/*-------------------------------------------------------------------------------------*/
void openNewFile() {
  
  //resync the time against the RTC
  syncTime();
  
  sprintf(fileName, "%02d%02d%02d%02d.CSV", day(), hour(), minute(), second());
  
#ifdef DEBUG
  Serial.println(fileName);
#endif

  if (!file.open(fileName, O_WRITE | O_CREAT | O_EXCL)) {
    msgStatus("File open err");
    error("file.open");
  }
  
  if (!file.isOpen()) {
    msgStatus("File not open");
    error("file.open");
  }

  file.println(LOG_INTERVAL_USEC); 
 
} 





/*-------------------------------------------------------------------------------------*/
void initBMA180() {
  
  // init the BMA180
  
  bma180.SetAddress((int)BMA180_DEFAULT_ADDRESS);
  bma180.SoftReset();
  bma180.enableWrite();
  
#ifdef DEBUG
  int sversion;
  int id;
  bma180.getIDs(&id,&sversion);
  Serial.print("Id = ");
  Serial.print(id,DEC);
  Serial.print(" v.");
  Serial.println(sversion,HEX);
#endif
  
  // ultra-low noise mode
//  bma180.setRegValue(0x30, 0x01, 0x03);

  doBMA180Calibration();

  // Turn off adv_int 
  bma180.setRegValue(0x21, 0x00, 0x04);  
  
  // enable low pass filter - expect seismic signals to be <50Hz
  bma180.SetFilter(bma180.F40HZ);
  
  // set to 1g full scale
  bma180.setGSensitivty(bma180.G1);
 
  bma180.disableWrite();
  delay(2000);
     
  initReferences(); 

}


/*-------------------------------------------------------------------------------------*/
void doBMA180Calibration() {
  // do a calibration

  // full calibration
  bma180.setRegValue(0x22, 0x03, 0x03);
  
  // calibrate each axis in turn. set the en_offset_* bit to request
  // the offset be set and wait until the bit is cleared, indicating
  // the calibration is compete. Takes a couple of seconds
  bma180.setRegValue(0x0E, 0x01 << 5, 0xE0);
  while ((bma180.getRegValue(0x0E) & 0xE0)) delay(5);
  
  bma180.setRegValue(0x0E, 0x01 << 6, 0xE0);
  while ((bma180.getRegValue(0x0E) & 0xE0)) delay(5);
  
  bma180.setRegValue(0x0E, 0x01 << 7, 0xE0);
  while ((bma180.getRegValue(0x0E) & 0xE0)) delay(5);
}




/*-------------------------------------------------------------------------------------*/
void initReferences() {
  
  // initial zero readings
  accX0 = 0;
  accY0 = 0;
  accZ0 = 0;
  
  // initial ma/min values out of range of possible sensor readings
  int32_t accX0min = 99999;
  int32_t accY0min = 99999;
  int32_t accZ0min = 99999;

  int32_t accX0max = -99999;
  int32_t accY0max = -99999;
  int32_t accZ0max = -99999;
  
  msgStatus("Calibrating");
  doBMA180Calibration();

  msgStatus("Set Threshold");
  // sum the readings and collect max/min values for samples at 20ms intervals
  for (int16_t i = 0; i < calibrationSamples; i++) {
    
    bma180.readAccel();
    accX0 += bma180.x;
    accY0 += bma180.y;
    accZ0 += bma180.z;

    accX0min = min(accX0min, bma180.x);
    accY0min = min(accY0min, bma180.y);
    accZ0min = min(accZ0min, bma180.z);
  
    accX0max = max(accX0max, bma180.x);
    accY0max = max(accY0max, bma180.y);
    accZ0max = max(accZ0max, bma180.z);
  
    delay(20);
    
  }
 
  // average readings
  accX0 = accX0 / calibrationSamples;
  accY0 = accY0 / calibrationSamples;
  accZ0 = accZ0 / calibrationSamples;
  
  // set thresholds
  accX0Threshold = max(abs(accX0 - accX0min), abs(accX0 - accX0max)) * thresholdFactor;
  accY0Threshold = max(abs(accY0 - accY0min), abs(accY0 - accY0max)) * thresholdFactor;
  accZ0Threshold = max(abs(accZ0 - accZ0min), abs(accZ0 - accZ0max)) * thresholdFactor;


#ifdef DEBUG
Serial.println(accX0);
Serial.println(accY0);
Serial.println(accZ0);

Serial.println(" ");

Serial.println(accX0Threshold);
Serial.println(accY0Threshold);
Serial.println(accZ0Threshold);
#endif
     
}


/*-------------------------------------------------------------------------------------*/
void readBMA180() {
  
  // interrupt service routine for accelerometer read.  called by TimerTwo library.
  
  // counts ticks between logging
  static uint16_t tickCt = TICKS_PER_LOG;
  static uint16_t serialNo = 0;
  static uint8_t consecutiveEvents = 0;
  
  // return if not time to log data
  if (tickCt++ > TICKS_PER_LOG) {
    
    // reset tick count
    tickCt = 0;
  
    // get the next empty cell  
    uint16_t next = ringGetNext(head);

    // if next == tail, then the ring is full - discard one from the tail
    if (next == tail) {
      tail = ringGetNext(tail);
      digitalWrite(pinRedLED, HIGH);
    } else
      digitalWrite(pinRedLED, LOW);

    // enable interrupts - readAccel takes a while
    sei();
   
    // read accelerometer
    bma180.readAccel();
    ring[head].serial = serialNo++;
    ring[head].x = bma180.x - accX0;
    ring[head].y = bma180.y - accY0;
    ring[head].z = bma180.z - accZ0;
  
    // flag of acceleration sensed - must be over the threshold for n consecutive samples
    if (abs(ring[head].x) > accX0Threshold || abs(ring[head].y) > accY0Threshold || abs(ring[head].z) > accZ0Threshold ) {
      
      if (consecutiveEvents++ >= consectiveEventThreshold) {
        eventDetected = -1;
        consecutiveEvents = 0;
      } else {
        eventDetected = 0;
      } 
      
    } else
      consecutiveEvents = 0;
 
    head = next;
       
  }

  
}





