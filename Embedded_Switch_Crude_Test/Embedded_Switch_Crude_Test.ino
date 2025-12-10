#include <Arduino_GFX_Library.h>
#include <ArduinoHttpClient.h>
#include <RPi_Pico_TimerInterrupt.h>
#include "Arduino.h"
#include "Wire.h"
#include <WiFi.h>

/*  Pin Definitions */
/*  --------------- */
/*  Buttons         */
/*  Physical PUp    */
/*  Active Low      */
/*  No Debounce     */
#define SW0 10      // Left Button
#define SW1 11      // Right Button

/*  LEDs            */
#define LED0 12     //  Left Greeen LED
#define LED1 13     //  Right Green LED
#define LED2 14     //  Left Red LED

/*  Display         */
#define TFT_CS 0    //  Chip Select
#define TFT_DC 2    //  ???
#define TFT_RST 1   //  RESET
#define GFX_BL 8    //  Backlight

/*  ADC             */
#define I0_OUT  27  //  Current on Relay 0's Channel      - ADC1  - Board Pin 32
#define I0_CH   1   //  5 A is approximately 2035 on this
#define I1_OUT  26  //  Current on Relay 1's Channel      - ADC0  - Board Pin 31
#define I1_CH   0   //  Measured 132-143 with 40 W 120 V Bulb, getting 116 V at L2 Outlet, so ~0.344827586207 A, ~382.8 conversion constant
#define ACV_IN  28  //  Current on Input AC Line Voltage  - ADC2  - Board Pin 34
#define AC_CH   2   //  Measured ~2920 at 118 VAC, divide by 24.7457627119 to convert

/*  Relays          */
/*  20ms Pulse      */
/*  Idle Low, ActH  */
#define R0_RST  19  //  Relay 0 Reset Line
#define R1_RST  17  //  Relay 1 Reset Line
#define R0_SET  18  //  Relay 0 Set Line
#define R1_SET  16  //  Relay 1 Set Line

/*  I2C             */
#define SDA     20
#define SCL     21
/*  --------------- */
/*  Magic Numbers   */
#define QSize         40000 //  Size of inter-task communication buffer
#define TIMER_MICROS  200   //  200 microseconds per interrupt for granular task timing
#define CONV_ADC_AMPS 4850
#define CONV_ADC_VLTG 24.74576

/*  Networking Constants  */
const char ssid[] = "DemoNet24";            //  Name of target wireless network
const char pass[] = "1234Demo";             //  Password to authenticate connection
IPAddress ip(192, 168, 0, 5);               //  Byte Array Format Address
const char serverAddr[] = "192.168.0.241";  //  String Representation
const int serverPort = 8000;                //  Port the server is listening on

/*  Class Initialization  */
/*  Timer for accurate tracking of task timing  */
RPI_PICO_Timer ScheduleTimer(1);
/*  WiFi and HTTP Clients for server connection and formatted requests  */
WiFiClient wfc;
HttpClient httpc = HttpClient(wfc, serverAddr, serverPort);
/*  Communication buses required for built-in display */
Arduino_DataBus *bus = new Arduino_RPiPicoSPI(TFT_DC /* DC */, TFT_CS /* CS */, 6 /* SCK */, 7 /* MOSI */, 16 /* MISO */, spi0 /* spi */);
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 1 /* rotation */, false /* IPS */);

/*  ------------------------------------------------------------- */
/*  Struct and Interface Functions for Circular Buffer FIFO Queue */
typedef struct _QueueDbl {
    double buf[QSize];  //  Requisition Memory for full queue
    long front;         //  Track the currently active ends of the circle
    long rear;
    unsigned char cnt;  //  Keep count of stored values for iteration over them.
} QueueDbl;             //  Indicate that the queue stores doubles

void QueueInit (QueueDbl *Q) {
    (*Q).cnt = 0;       //  Start with Empty Queue
    (*Q).front = -1;    //  Allow progression through buffer on all pushes
    (*Q).rear = -1;
}

unsigned char QueueFull(QueueDbl Q) {   //  Next position along buffer being the next value out means it's full
    return (((Q.rear + 1) % QSize) == Q.front);
}

unsigned char QueueEmpty(QueueDbl Q) {      //  Front index with catch up with rear index 
    return (Q.front == Q.rear);             //  once all values are popped
}

void QueuePush(QueueDbl *Q, double item) {  //  Add a value to the Queue
    if (!QueueFull(*Q)) {                   //  Unless it's full
      if ((*Q).front == -1) {               //  When values are added to the empty buffer,
        (*Q).front = 0;                     //  End tracking indices must be in-bounds
      }
      (*Q).rear = ((*Q).rear + 1) % QSize;  //  Advance rear index, unless it goes beyond total space, then circle back
      (*Q).buf[(*Q).rear] = item;           //  Set value at that index to pushed value
      (*Q).cnt++;                           //  Track count of pushed values
    }
}

double QueuePop(QueueDbl *Q) {                  //  Remove First Value pushed onto the Queue
    double item = (*Q).buf[(*Q).front];         //  Retrieve value from front of Queue

    if (!QueueEmpty(*Q)) {                      //  Advance/reset pointers if not empty
      if ((*Q).front == (*Q).rear) {
        (*Q).front = -1;
        (*Q).rear = -1;
      } else {
        (*Q).front = ((*Q).front + 1) % QSize;
      }
      (*Q).cnt--;                               //  Track decreased count of queued data
    } else {
      item = 0;                                 //  If Empty, return a zero, prevent 32 bit limit ints from undefined space
    }
    
    return(item);
}

QueueDbl SendQ;                                 //  Claim structure memory
/*  ------------------------------------------------------------- */

/*  Constants Relating to Task Parameters */
const unsigned int numTasks = 5;
const unsigned long periodScheduler = 1;
const unsigned long periodLoad1 = 20000;
const unsigned char L1_Index = 0;
const unsigned long periodLoad2 = 20000;
const unsigned char L2_Index = 1;
const unsigned long periodGFX = 500000;
const unsigned long periodSampler = 800;
const unsigned long periodSender = 1000000;

/*  Structure and memory claim for Task State Tracking  */
typedef struct task {
  int state;
  unsigned long period;
  unsigned long elapsedTime;
  unsigned char isActive;
  int (*Function) (int);
} task;

task tasks[numTasks];
/*  --------------------------------------------------  */

/*  Graphical Constants */
const uint16_t XRED = 0xFA8E;
int32_t w, h, n, n1, cx, cy, cx1, cy1, cn, cn1; //  Not const due to calculation at runtime setup
uint8_t tsaa, tsa, tsb, tsc, ds;                //  but effectively similar past that point

/*  Shared Variables for Global State Awareness */
double SamplePower;
unsigned char L1Active = 1;
unsigned char L2Active = 1;
unsigned char L1_ISRActive = 0;
unsigned char L2_ISRActive = 0;
int L1Prev = 2;
int L2Prev = 2;
int L1APrev = 1;
int L2APrev = 1;

/*  State enums for the various tasks */
enum  L1_States {L1_Off, L1_WaitBounce, L1_On};
enum  L2_States {L2_Off, L2_WaitBounce, L2_On};
enum  GFX_States {GFXUpdating, GFXOff};
enum  Sampler_States {Sampling, SamPaused};

/*  ISR and control functions for registering button presses and software debouncing  */
void L1ToggleISR(); // Blank function defs for reference. Should really be in a .h file
void L2ToggleISR(); //  but adding a whole header file for two functions seems extra
/*  To prevent severe blocking without hardwarw debounce, ISR must be able to toggle its trigger  */
void L1_ISR() {
  tasks[L1_Index].isActive = 1;                         //  Activate Task for Scheduler
  tasks[L1_Index].elapsedTime = tasks[L1_Index].period; //  Make Ready on next tick
  digitalWrite(LED0, HIGH);
  if (tasks[L2_Index].state == L2_WaitBounce) digitalWrite(LED2, HIGH);
  L1ToggleISR();                                        //  Ignore Bounces until state change
}

void L2_ISR() {
  tasks[L2_Index].isActive = 1;
  tasks[L2_Index].elapsedTime = tasks[L2_Index].period;
  digitalWrite(LED1, HIGH);
  if (tasks[L1_Index].state == L1_WaitBounce) digitalWrite(LED2, HIGH);
  L2ToggleISR();
}
/*  ISR toggle must be able to reference the ISR function */
void  L1ToggleISR() {
  switch (L1_ISRActive) {
    case 0: attachInterrupt(digitalPinToInterrupt(SW0), L1_ISR, FALLING);
            L1_ISRActive = 1;
            break;
    case 1: detachInterrupt(digitalPinToInterrupt(SW0));
            L1_ISRActive = 0;
            break;
  }
}

void  L2ToggleISR() {
  switch (L2_ISRActive) {
    case 0: attachInterrupt(digitalPinToInterrupt(SW1), L2_ISR, FALLING);
            L2_ISRActive = 1;
            break;
    case 1: detachInterrupt(digitalPinToInterrupt(SW1));
            L2_ISRActive = 0;
            break;
  }
}

/*  State Machine Tick Functions  */

/*  Load 1 Relay State Controller */
int L1_TickSM(int state) {
  
  /*  Transition Logic  */
  switch (state) {
    case  L1_Off:         state = L1Active || digitalRead(SW0) ? L1_Off : L1_WaitBounce;
                          break;
    case  L1_WaitBounce:  state = !digitalRead(SW0) ? L1_WaitBounce : (L1Active ? L1_Off : L1_On);
                          break;
    case  L1_On:          state = !L1Active || digitalRead(SW0) ? L1_On : L1_WaitBounce;
                          break;
    default:              state = L1_Off;
                          break;
  }
  
  /*  Action Logic      */
  switch (state) {
    case  L1_Off:         if (!L1Active) {
                            tasks[L1_Index].isActive = 0;
                            digitalWrite(LED0, LOW);
                            digitalWrite(LED2, LOW);
                            digitalWrite(R0_RST, LOW);
                            if (!L1_ISRActive) L1ToggleISR();
                          } else {
                            digitalWrite(R0_RST, HIGH);
                            L1Active = 0;
                          }
                          break;
    case  L1_WaitBounce:  break;  // Nothing to do but wait here
    case  L1_On:          if (L1Active) {
                            tasks[L1_Index].isActive = 0;
                            digitalWrite(R0_SET, LOW);
                            if (!L1_ISRActive) L1ToggleISR();
                          } else {
                            digitalWrite(R0_SET, HIGH);
                            L1Active = 1;
                          }
                          break;
  }

  return state;
}

/*  Load 2 Relay State Controller */
int L2_TickSM(int state) {
  /*  Transition Logic  */
  switch (state) {
    case  L2_Off:         state = L2Active || digitalRead(SW1) ? L2_Off : L2_WaitBounce;
                          break;
    case  L2_WaitBounce:  state = !digitalRead(SW1) ? L2_WaitBounce : (L2Active ? L2_Off : L2_On);
                          break;
    case  L2_On:          state = !L2Active || digitalRead(SW1) ? L2_On : L2_WaitBounce;
                          break;
    default:              state = L2_Off;
                          break;
  }
  
  /*  Action Logic      */
  switch (state) {
    case  L2_Off:         if (L2Active) {                       //  On first tick of SM, ensure relay inactive
                            digitalWrite(R1_RST, HIGH);         //  Begin RESET pulse
                            L2Active = 0;                       //  Set global status tracker inactive
                          } else {                              //  On the next tick in this state, deactivate the task
                            tasks[L2_Index].isActive = 0;
                            digitalWrite(LED1, LOW);            //  Disable LED button/state change indicators
                            digitalWrite(LED2, LOW);
                            digitalWrite(R1_RST, LOW);          //  End the RESET pulse after 1 period (50 ms > 20 ms)
                            if (!L2_ISRActive) L2ToggleISR();   //  Last, enable ISR to listen for reactivation
                            if (!tasks[3].isActive) tasks[3].isActive = 1;
                            if (!tasks[4].isActive) tasks[4].isActive = 1;
                          }
                          break;
    case  L2_WaitBounce:  break;                                //  Nothing to do but wait here
    case  L2_On:          if (!L2Active) {                      //  If newly arrived in this state, begin SET pulse
                            digitalWrite(R1_SET, HIGH);
                            L2Active = 1;                       //  Set global relay status active, as it will be in 20ms
                          } else {                              //  Once relay active, deactivate task, button LEDs, and SET pulse
                            tasks[L2_Index].isActive = 0;
                            digitalWrite(R1_SET, LOW);
                            if (!L2_ISRActive) L2ToggleISR();   //  And re-enable button listening ISR
                          }
                          break;
  }

  return state;
}

/*  Display output State Control  */
int UpdateGFX(int state) {
  
  /*  Transition Logic  */
  switch (state) {
    case GFXUpdating: state = GFXUpdating;
                      break;
    case GFXOff:      state = GFXOff;
                      break;
  }

  /*  Action Logic      */
  switch (state) {
    case GFXUpdating: int ADCVal = analogRead(I0_OUT);
                      gfx->fillRect(70, 160, 18*15, 44, RGB565_BLACK);
                      gfx->flush();
                      gfx->setCursor(70, 160);
                      gfx->setTextSize(3);
                      gfx->setTextColor(XRED);
                      gfx->print(F("Load 1: "));
                      gfx->print((double)ADCVal / (double)CONV_ADC_AMPS);
                      gfx->print(F(" A"));

                      ADCVal = analogRead(I1_OUT);
                      gfx->setCursor(70, 183);
                      gfx->print(F("Load 2: "));
                      gfx->print((double)ADCVal / (double)CONV_ADC_AMPS);
                      gfx->print(F(" A"));
                      
                      gfx->setCursor(140, 238);
                      gfx->fillRect(250, 237, 120, 23, XRED);
                      gfx->setTextColor(RGB565_BLACK);
                      gfx->print(F("Pwr -> "));
                      gfx->print(SamplePower);
                      gfx->print(F(" W"));

                      /*  Update Display for Relay SM Current State */
                      if (L1Prev != tasks[L1_Index].state) {
                        gfx->fillRect(166, 80, 20, 28, RGB565_BLACK);
                        gfx->flush();
                        gfx->setCursor(166, 80);
                        gfx->setTextSize(4);
                        gfx->setTextColor(RGB565_RED);
                        gfx->print(tasks[L1_Index].state);
                      }
                      if (L2Prev != tasks[L2_Index].state) {
                        gfx->fillRect(382, 80, 20, 28, RGB565_BLACK);
                        gfx->flush();
                        gfx->setCursor(382, 80);
                        gfx->setTextSize(4);
                        gfx->setTextColor(RGB565_RED);
                        gfx->print(tasks[L2_Index].state);
                      }
                      //  Track State To Reduce Unnecessary Blocking Refreshes
                      L1Prev = tasks[L1_Index].state;
                      L2Prev = tasks[L2_Index].state;

                      /*  Update Display for Current Relay Position */
                      if (L1APrev != L1Active) {
                        gfx->fillRect(166, 120, 20, 28, RGB565_BLACK);
                        gfx->flush();
                        gfx->setCursor(166, 120);
                        gfx->setTextSize(4);
                        gfx->setTextColor(RGB565_GREEN);
                        gfx->print(L1Active);
                      }

                      if (L2APrev != L2Active) {
                        gfx->fillRect(382, 120, 20, 28, RGB565_BLACK);
                        gfx->flush();
                        gfx->setCursor(382, 120);
                        gfx->setTextSize(4);
                        gfx->setTextColor(RGB565_GREEN);
                        gfx->print(L2Active);
                      }
                      //  Track Changes to Only Update as Needed
                      L1APrev = L1Active;
                      L2APrev = L2Active;
  }

  return state;
}

/*  Sensor Sampling task with period of 800 ns, allowing at least 1kHz rate even with some timer overrun  */
int TickSampler(int state) {
  double L1Current;
  double L2Current;
  double ACVoltage;
  
  switch (state) {
    case Sampling:  state = Sampling;   break;
    case SamPaused: state = SamPaused;  break;
  }

  switch (state) {
    case Sampling:  L1Current = (double)analogRead(I0_OUT) / (double)CONV_ADC_AMPS;
                    L2Current = (double)analogRead(I1_OUT) / (double)CONV_ADC_AMPS;
                    ACVoltage = (double)analogRead(ACV_IN) / (double)CONV_ADC_VLTG;
                    SamplePower = ACVoltage * (L1Current + L2Current);
                    if (!QueueFull(SendQ)) {
                      QueuePush(&SendQ, ACVoltage);
                      QueuePush(&SendQ, L1Current);
                      QueuePush(&SendQ, L2Current);
                      QueuePush(&SendQ, SamplePower);
                    }
                    break;
                    
  }

  return state;
}



int tickSendSamples(int state) {
  String CmdByte;     //  Storage for command info return on system voltage POST
  double VSum = 0;    //  doubles to store the sum of up to 10,000 sampled doubles
  double O1CSum = 0;
  double O2CSum = 0;
  double TPSum = 0;
  int sampleCnt = SendQ.cnt/4;  //  Every four doubles pushed into the queue represent one sample
  int i, j;                     //  Iterators for each sample set, and each sample type within those sets

  for (i = 0; i < sampleCnt; i++) {
    for (j = 0; j < 4; j++) {
      switch (j) {
        case 0: VSum += QueuePop(&SendQ);   break;//Vs[i] = QueuePop(&SendQ);   break;
        case 1: O1CSum += QueuePop(&SendQ); break;//O1C[i] = QueuePop(&SendQ);  break;
        case 2: O2CSum += QueuePop(&SendQ); break;//O2C[i] = QueuePop(&SendQ);  break;
        case 3: TPSum += QueuePop(&SendQ);  break;//TP[i] = QueuePop(&SendQ);   break;
      }
    }
  }

  //  POST request to the host server sharing voltage sample average, receiving command info
  httpc.beginRequest();
  httpc.post("/voltage");
  httpc.sendHeader("Val", String(VSum/(double)sampleCnt));
  httpc.endRequest();
  if (httpc.responseStatusCode() == 200) {
    CmdByte = httpc.responseBody();
  } else {
    httpc.responseBody(); //  The body of an error response is not needed, but must be purged from in-queue memory
  }

  //  POST of averaged current through Load 1
  httpc.beginRequest();
  httpc.post("/outlet1/current");
  httpc.sendHeader("Val", String(O1CSum/(double)sampleCnt));
  httpc.endRequest();
  if (httpc.responseStatusCode() != 200) {
    httpc.responseBody();
  }

  //  POST of averaged current through Load 2
  httpc.beginRequest();
  httpc.post("/outlet2/current");
  httpc.sendHeader("Val", String(O2CSum/(double)sampleCnt));
  httpc.endRequest();
  if (httpc.responseStatusCode() != 200) {
    httpc.responseBody();
  }

  //  POST of averaged power calculations
  httpc.beginRequest();
  httpc.post("/total_power");
  httpc.sendHeader("Val", String(TPSum/(double)sampleCnt));
  httpc.endRequest();
  if (httpc.responseStatusCode() != 200) {
    httpc.responseBody();
  }

  //  POST of current relay state for Load 1
  httpc.beginRequest();
  httpc.post("/outlet1/state");
  httpc.sendHeader("Val", L1Active);
  httpc.endRequest();
  if (httpc.responseStatusCode() != 200) {
    httpc.responseBody();
  }

  //  POST of current relay state for Load 2
  httpc.beginRequest();
  httpc.post("/outlet2/state");
  httpc.sendHeader("Val", L2Active);
  httpc.endRequest();
  if (httpc.responseStatusCode() != 200) {
    httpc.responseBody();
  }

  //  Flush Rx data not already processed to prevent memory leakage
  httpc.flush(); 

  //  Given the encoding systems involved, sending/decoding a single UTF-8 char was the simplest and most memory efficient method
  switch (CmdByte.charAt(0)) {
    case '`': //  Disable Both
              if (tasks[L1_Index].state == L1_On) {
                tasks[L1_Index].state = L1_WaitBounce;
                tasks[L1_Index].isActive = 1;
                tasks[L1_Index].elapsedTime = tasks[L1_Index].period;
              }
              if (tasks[L2_Index].state == L2_On) {
                tasks[L2_Index].state = L2_WaitBounce;
                tasks[L2_Index].isActive = 1;
                tasks[L2_Index].elapsedTime = tasks[L2_Index].period;
              }
              break;
    case 'a': //  Enable L1, Disable L2
              if (tasks[L1_Index].state == L1_Off) {
                tasks[L1_Index].state = L1_WaitBounce;
                tasks[L1_Index].isActive = 1;
                tasks[L1_Index].elapsedTime = tasks[L1_Index].period;
              }
              if (tasks[L2_Index].state == L2_On) {
                tasks[L2_Index].state = L2_WaitBounce;
                tasks[L2_Index].isActive = 1;
                tasks[L2_Index].elapsedTime = tasks[L2_Index].period;
              }
              break;
    case 'b': //  Disable L1, Enable L2
              if (tasks[L1_Index].state == L1_On) {
                tasks[L1_Index].state = L1_WaitBounce;
                tasks[L1_Index].isActive = 1;
                tasks[L1_Index].elapsedTime = tasks[L1_Index].period;
              }
              if (tasks[L2_Index].state == L2_Off) {
                tasks[L2_Index].state = L2_WaitBounce;
                tasks[L2_Index].isActive = 1;
                tasks[L2_Index].elapsedTime = tasks[L2_Index].period;
              }
              break;
    case 'c': //  Enable both
              if (tasks[L1_Index].state == L1_Off) {
                tasks[L1_Index].state = L1_WaitBounce;
                tasks[L1_Index].isActive = 1;
                tasks[L1_Index].elapsedTime = tasks[L1_Index].period;
              }
              if (tasks[L2_Index].state == L2_Off) {
                tasks[L2_Index].state = L2_WaitBounce;
                tasks[L2_Index].isActive = 1;
                tasks[L2_Index].elapsedTime = tasks[L2_Index].period;
              }
              break;
  }
  
  return state; //  This really isn't a state machine, but implemented in the SM Task Structure, it's easiest to go along with the pattern
}

void setup() {
  /*  GPIO/ADC Init */
  pinMode(SW0, INPUT);
  pinMode(SW1, INPUT);
  pinMode(I0_OUT, INPUT);
  pinMode(I1_OUT, INPUT);
  pinMode(ACV_IN, INPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(R0_RST, OUTPUT);
  pinMode(R1_RST, OUTPUT);
  pinMode(R0_SET, OUTPUT);
  pinMode(R1_SET, OUTPUT);
  analogReadResolution(12);

  /*  Task Setup    */
  tasks[L1_Index].state = L1_Off;
  tasks[L1_Index].isActive = 1;
  tasks[L1_Index].period = periodLoad1;
  tasks[L1_Index].elapsedTime = tasks[L1_Index].period;
  tasks[L1_Index].Function = &L1_TickSM;
  tasks[L2_Index].state = L2_Off;
  tasks[L2_Index].isActive = 1;
  tasks[L2_Index].period = periodLoad2;
  tasks[L2_Index].elapsedTime = tasks[L2_Index].period;
  tasks[L2_Index].Function = &L2_TickSM;
  tasks[2].state = GFXUpdating;
  tasks[2].isActive = 1;
  tasks[2].period = periodGFX;
  tasks[2].elapsedTime = tasks[2].period;
  tasks[2].Function = &UpdateGFX;
  tasks[3].state = Sampling;
  tasks[3].isActive = 0;
  tasks[3].period = periodSampler;
  tasks[3].elapsedTime = 0;
  tasks[3].Function = &TickSampler;
  tasks[4].state = 0;
  tasks[4].isActive = 0;
  tasks[4].period = periodSender;
  tasks[4].elapsedTime = 0;
  tasks[4].Function = &tickSendSamples;

  QueueInit(&SendQ);

  /*  Display Init  */
  gfx->begin();
  w = gfx->width();
  h = gfx->height();
  n = min(w, h);
  n1 = n - 1;
  cx = w / 2;
  cy = h / 2;
  cx1 = cx - 1;
  cy1 = cy - 1;
  cn = min(cx1, cy1);
  cn1 = cn - 1;
  tsaa = ((w <= 90) || (h <= 160)) ? 1 : (((w <= 240) || (h <= 240)) ? 2 : 3); // text size A
  tsa = ((w <= 176) || (h <= 160)) ? 1 : (((w <= 240) || (h <= 240)) ? 2 : 3); // text size A
  tsb = ((w <= 272) || (h <= 220)) ? 1 : 2;                                    // text size B
  tsc = ((w <= 220) || (h <= 220)) ? 1 : 2;                                    // text size C
  ds = (w <= 160) ? 9 : ((w <= 280) ? 10 : 12);                                // digit size
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
  gfx->fillScreen(RGB565_BLACK);
  gfx->fillRect(40, 40, 400, 240, XRED);
  gfx->fillRect(44, 44, 392, 232, RGB565_BLACK); // BLACK
  gfx->fillRect(44, 220, 392, 56, XRED);
  gfx->fillRect(64, 30, 208, 24, RGB565_BLACK);
  gfx->setCursor(71, 32);
  gfx->setTextSize(3);
  gfx->setTextColor(XRED);
  gfx->print(F("StateConfig"));
  gfx->setCursor(70, 80);
  gfx->setTextSize(4);
  gfx->setTextColor(XRED);
  gfx->print(F("L1:   -- L2:  "));
  gfx->setCursor(70, 120);
  gfx->print(F("R0:   -- R1:  "));

  /*  WiFi Initialization */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    if (WiFi.status() == WL_CONNECT_FAILED) WiFi.begin(ssid, pass);
    delay(1000);
  }

  /*  Primary Scheduling Timer Interrupt  */
  ScheduleTimer.attachInterruptInterval(TIMER_MICROS, ScheduleTimerISR);
}

/*  As the hardware timer ISR will always have priority, no delay is required     */
/*  Tasks may be checked as interrupts allow, and timing will be accurately kept  */
void loop() {
  unsigned char i;
  for (i = 0; i < numTasks; i++) {
    if (tasks[i].isActive == 1 && tasks[i].elapsedTime >= tasks[i].period) {
      tasks[i].state = tasks[i].Function(tasks[i].state);
      tasks[i].elapsedTime = 0;
    }
  }
}

/*  Timer ISR to elapse a consistent time on a precise interval */
bool ScheduleTimerISR(struct repeating_timer *t) {
  (void) t;
  unsigned char i;
  for (i = 0; i < numTasks; i++) {
    if (tasks[i].isActive == 1) {
      tasks[i].elapsedTime += TIMER_MICROS;
    }
  }

  return true;
}
