/*------------------------------------------------------------------------------\
| Final Project for Embedded Computational Systems                              |
| Student 1: Saúl Carvalho                                                      |
| Student 2: Cristiano Moreira                                                  |
\------------------------------------------------------------------------------*/
#include "Arduino.h"
#include "stdio.h"
#include "Adafruit_BusIO_Register.h"
#include "Adafruit_ILI9341.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"
#include "BluetoothSerial.h"

// Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

// RTOS Function Declarations
static void vTaskHandler_PIR(    void *pvParameters );
static void vTaskHandler_Brain(  void *pvParameters );
static void vTaskHandler_Touch(  void *pvParameters );
static void vTaskHandler_BLT(    void *pvParameters );
static void vTaskHandler_Buzzer( void *pvParameters );
static void vTaskHandler_LCD(    void *pvParameters );
static void IRAM_ATTR vIntHandler_PIR( void );
bool my_vApplicationIdleHook( void );

// RTOS Services
SemaphoreHandle_t xBinSemaphore_PIR;    // Binary Semaphore
QueueHandle_t     xQueue_PIR, xQueue_Touch, xQueue_Buzzer, xQueue_LCD, xQueue_recvBLT, xQueue_sendBLT;  // Queues

// LCD Pin Definitions
#define TFT_MISO  19  // SD0
#define TFT_SCK   18  // Clock
#define TFT_MOSI  23  // SDI
#define TFT_DC    0   // 
#define TFT_RESET 2   // RESET
#define TFT_CS    15  // Chip Select
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RESET, TFT_MISO);

// Other Pin Definitions
#define pinPIR     35
#define pinTouch   4
#define pinBuzzer  32
#define pinRedLED  12
#define pinBlueLED 27

// PWM settings for Buzzer and LEDs
const uint8_t freq = 5000;    // Frequency
const uint8_t res  = 8;       // Resolution
const uint8_t chBuzzer  = 0;  // Buzzer Channel
const uint8_t chBlueLED = 1;  // Blue LED Channel
const uint8_t chRedLED  = 2;  // Red LED Channel

// Struct definition for LCD data
struct dataLCD {    // struct for data transfer to LCD
  uint8_t code;     // Code for which screen to show
  int8_t  counter;  // Counter variable
};

/*-----------------------------------------------------------------------------------------------------------------------\
|                                                   SETUP                                                                |
\-----------------------------------------------------------------------------------------------------------------------*/
void setup( void ) {
  Serial.begin( 115200 );   // Initiate USART and set baud-rate to 115200bps
  Serial.print( "\t>SYSTEM: System configuration started.\r\n" );

  SerialBT.begin( "ESP32_Alarm" );  // Bluetooth device name
  Serial.print( "\t>SYSTEM: Bluetooth device has started as ESP32_Alarm.\r\n" );

  // LCD Configurations
  tft.begin();          // Initialize LCD
  tft.setRotation(1);   // Landscape mode
  tft.setTextSize(3);   // Text Size
  tft.setTextColor(ILI9341_WHITE);  // Text Color
  tft.setTextWrap(true);// Changes line (\n) upon reaching screen's border

  // FreeRTOS Configurations
  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);           // Set loopTask max priority before deletion
  esp_register_freertos_idle_hook(my_vApplicationIdleHook);   // Register idle hook callback
  vSemaphoreCreateBinary( xBinSemaphore_PIR );                // Creates binary semaphore for PIR interrupt
  // Creation of queues for data transfer between module
  xQueue_PIR     = xQueueCreate( 1, sizeof( bool )    );      // Motion Sensor Queue
  xQueue_Touch   = xQueueCreate( 1, sizeof( uint8_t ) );      // Capacitor Touch Queue
  xQueue_Buzzer  = xQueueCreate( 1, sizeof( bool )    );      // Buzzer Queue
  xQueue_recvBLT = xQueueCreate( 1, sizeof( String )  );      // RX Bluetooth Terminal Queue
  xQueue_sendBLT = xQueueCreate( 1, sizeof( String )  );      // TX Bluetooth Terminal Queue
  xQueue_LCD     = xQueueCreate( 1, sizeof( dataLCD ) );      // LCD Queue

  // Check if queues and semaphores created successfully
  if ( xBinSemaphore_PIR != NULL && xQueue_PIR != NULL && xQueue_Touch != NULL && xQueue_Buzzer != NULL && xQueue_recvBLT != NULL && xQueue_sendBLT != NULL && xQueue_LCD != NULL ) {
    // Header ->             TaskHandler           TaskName           StackSize  InParameters  Priority  TaskReference  Core
    xTaskCreatePinnedToCore( vTaskHandler_PIR,     "Task PIR",        800,       NULL,         6,        NULL,          1 );
    xTaskCreatePinnedToCore( vTaskHandler_Brain,   "Task Brain",      1000,      NULL,         5,        NULL,          1 );
    xTaskCreatePinnedToCore( vTaskHandler_BLT,     "Task Bluetooth",  900,       NULL,         4,        NULL,          0 );
    xTaskCreatePinnedToCore( vTaskHandler_Touch,   "Task Touch",      1000,      NULL,         3,        NULL,          0 );
    xTaskCreatePinnedToCore( vTaskHandler_Buzzer,  "Task Buzzer",     800,       NULL,         2,        NULL,          1 );
    xTaskCreatePinnedToCore( vTaskHandler_LCD,     "Task LCD",        1200,      NULL,         1,        NULL,          0 );
    //vTaskStartScheduler();
  } else {
    if ( xBinSemaphore_PIR == NULL) {
      Serial.print( "\t>SYSTEM: Error creating PIR Binary Semaphore.\r\n" );
    }
    if ( xQueue_PIR == NULL ) {
      Serial.print( "\t>SYSTEM: Error creating PIR Queue\r\n" );
    }
    if ( xQueue_Touch == NULL ) {
      Serial.print( "\t>SYSTEM: Error creating Touch Queue\r\n" );
    }
    if ( xQueue_Buzzer == NULL ) {
      Serial.print( "\t>SYSTEM: Error creating Buzzer Queue\r\n" );
    }
    if ( xQueue_recvBLT == NULL ) {
      Serial.print( "\t>SYSTEM: Error creating Receive Bluetooth Queue\r\n" );
    }
    if ( xQueue_sendBLT == NULL ) {
      Serial.print( "\t>SYSTEM: Error creating Send Bluetooth Queue\r\n" );
    }
    if ( xQueue_LCD == NULL ) {
      Serial.print( "\t>SYSTEM: Error creating LCD Queue\r\n" );
    }
  }

  // Pin Configurations
  pinMode( pinTouch,  INPUT_PULLUP );
  pinMode( pinPIR,    INPUT_PULLUP );
  attachInterrupt( digitalPinToInterrupt(pinPIR), &vIntHandler_PIR, RISING );
  pinMode( pinBuzzer, OUTPUT );           // Default Buzzer settings (Initialized OFF)
  ledcSetup( chBuzzer, freq, res );
  ledcAttachPin( pinBuzzer, chBuzzer );
  ledcWrite(chBuzzer, 0);
  pinMode( pinBlueLED, OUTPUT );          // Default Blue LED settings (Initialized ON)
  ledcSetup( chBlueLED, freq, res );
  ledcAttachPin( pinBlueLED, chBlueLED );
  ledcWrite(chBlueLED, 255);
  pinMode( pinRedLED,  OUTPUT );          // Default Red LED settings (Initialized OFF)
  ledcSetup( chRedLED, freq, res );
  ledcAttachPin( pinRedLED, chRedLED );
  ledcWrite(chRedLED, 0);

  //  for( ;; );
  //  return 0;

  Serial.print( "\t>SYSTEM: System configuration done.\r\n" );
}

/*-----------------------------------------------------------------------------------------------------------------------\
|                                             FUNCTIONS & TASKS                                                          |
\-----------------------------------------------------------------------------------------------------------------------*/
/* ----- PIR Interrupt Handler ----- */
static void  IRAM_ATTR vIntHandler_PIR( void ) {
  static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Interrupt detected -> gives back the binary semaphore
  xSemaphoreGiveFromISR( xBinSemaphore_PIR, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

  // High priority task woken -> force context switch to unblock task ( vHandlerTaskPIR )
  if ( xHigherPriorityTaskWoken == pdTRUE ) {
    portYIELD_FROM_ISR();   // goes back to the highest priority task ( vHandlerTaskPIR )
  }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Task Handler PIR ----- */
static void vTaskHandler_PIR( void *pvParameters ) {
  bool firstRead = true, statePIR = false; // Flags
  portBASE_TYPE xStatus_PIR;

  xSemaphoreTake( xBinSemaphore_PIR, 0 ); // Runs after yield from ISR, takes semaphore

  // Semaphore waits for event. Function only returns once semaphore is taken.
  for ( ;; ) {
    xSemaphoreTake( xBinSemaphore_PIR, portMAX_DELAY ); // Takes Semaphore
    Serial.print( "\t\t\t-----------------------------------  TASK_PIR  -----------------------------------\r\n" );
    Serial.print( "\t\t\t>TASK_PIR: \tPIR generated an interrupt!\r\n" );
    if (firstRead) { // Ignores first reading -> MCU pin settle
      firstRead = false;
    } else {
      statePIR  = true;
      xStatus_PIR = xQueueSendToBack( xQueue_PIR, &statePIR, 0 );   // Queue: PIR -> Brain
    }

    if ( xStatus_PIR == pdPASS ) {
      Serial.print( "\t\t\t>QUEUE_PIR: \tSend to PIR Queue \t\t\t-> State: TRUE\r\n" );
    } else {
      // ----- Queue state report to Serial Monitor -----
      Serial.print( "\t\t\t>QUEUE_PIR: \tSend to PIR Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_PIR ) == 1 ) {
        Serial.print( " \t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_PIR ) == 0 ) {
        Serial.print( " \t\t[Queue empty\r\n" );
      } else {
        Serial.println();
      }
    }
    Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
  }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Task Handler Bluetooth ----- */
static void vTaskHandler_BLT( void *pvParameters ) {
  uint8_t serialBuffer[20] = {};
  char    charBuffer[20]   = {};
  uint8_t charCounter      = 0;
  uint8_t failCounter      = 0;
  String  incomingMSG      = "";
  String  sendMSG          = "";
  portBASE_TYPE xStatus_recvBLT, xStatus_sendBLT;
  const TickType_t xDelay50ms = 50 / portTICK_PERIOD_MS; // allows to read up to 5 chars in serial per task brain cycle
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    if ( SerialBT.available() ) { // if text is written in bluetooth serial
      serialBuffer[charCounter] = SerialBT.read();  // Get character
      charCounter++;
    }
    // charCounter > 2 means there is at least a Carriage Return (CR) and New Line (NL) sent
    if ( charCounter > 2) {
      if ( serialBuffer[charCounter - 2] == 13 && serialBuffer[charCounter - 1] == 10 ) {    // ASCII -> CR = 13 (\r) and NL = 10 (\n), means full text has arrived
        Serial.print( "\t\t\t------------------------------------ TASK_BLT ------------------------------------\r\n" );
        Serial.print( "\t\t\t>TASK_BLT: \tReceive from serial \t\t\t<- Text: " );
        for ( uint8_t i = 0; i < charCounter - 2; i++ ) {
          charBuffer[i] = char( serialBuffer[i] );
          //Serial.print( charIn[i] );      // DEBUG
        }
        incomingMSG = String( charBuffer ); // ASCII -> Char -> String
        Serial.println( incomingMSG );

        xStatus_recvBLT = xQueueSendToBack( xQueue_recvBLT, &incomingMSG, 0 );  // Queue: BLT -> Brain
        
        // ----- Queue state report to Serial Monitor -----
        if ( xStatus_recvBLT == pdPASS ) {
          Serial.print( "\t\t\t>QUEUE_BLT: \tSend to BLT Queue \t\t\t-> Text: " );  Serial.print( incomingMSG );  Serial.println();
        } else {
          Serial.print( "\t\t\t>QUEUE_BLT: \tSend to BLT Queue failed!" );
          if ( uxQueueMessagesWaiting( xQueue_recvBLT ) == 1 ) {
            Serial.print( " \t\t[Queue full]\r\n" );
          } else if ( uxQueueMessagesWaiting( xQueue_recvBLT ) == 0 ) {
            Serial.print( " \t\t[Queue empty]\r\n" );
          } else {
            Serial.println();
          }
        }

        // --- Clear Buffers ---
        for ( uint8_t i = 0; i < charCounter - 2; i++ ) {
          serialBuffer[i] = NULL;
          charBuffer[i] = NULL;
        }
        charCounter = 0;
        Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
      }
    }

    xStatus_sendBLT = xQueueReceive( xQueue_sendBLT, &sendMSG, 0 ); // Queue: Brain -> Bluetooth Terminal
    if ( xStatus_sendBLT == pdPASS ) {
      Serial.print( "\t\t\t------------------------------------ TASK_BLT ------------------------------------\r\n" );
      Serial.print( "\t\t\t>QUEUE_BLT: \tReceive from BLT Queue \t\t\t-> Text: " );  Serial.print( sendMSG );  Serial.println();
      sendMSG.concat("\r\n\0"); // Message received from Brain
      for ( uint8_t i = 0; i < sendMSG.length(); i++ ) {  // String -> CharArray
        sendMSG.toCharArray( charBuffer, sendMSG.length() );
      }
      for ( uint8_t i = 0; i < sendMSG.length(); i++ ) {  // Writes message to BT Serial Terminal
        SerialBT.write( charBuffer[i] );
      }
      // --- Clear Buffer --- 
      for ( uint8_t i = 0; i < sendMSG.length(); i++ ) {
        charBuffer[i] = NULL;
      }
      Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
    } else {
      // ----- Queue state report to Serial Monitor -----
      failCounter++;
      if ( failCounter == 4 ) {
        Serial.print( "\t\t\t------------------------------------ TASK_BLT ------------------------------------\r\n" );
        Serial.print( "\t\t\t>QUEUE_BLT: \tReceive from BLT Queue failed!" );
        if ( uxQueueMessagesWaiting( xQueue_sendBLT ) == 1 ) {
          Serial.print( " \t\t[Queue full]\r\n" );
        } else if ( uxQueueMessagesWaiting( xQueue_sendBLT ) == 0 ) {
          Serial.print( " \t\t[Queue empty]\r\n" );
        } else {
          Serial.println();
        }
        failCounter = 0;
        Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
      }
    }
    vTaskDelayUntil( &xLastWakeTime, xDelay50ms );  // "Smart" Delay (50ms)
  }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Task Handler Touch ----- */
static void vTaskHandler_Touch( void *pvParameters ) {
  bool firstRead = true;
  uint8_t touchVal, touchState;
  portBASE_TYPE xStatus_Touch;
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    Serial.print( "\t\t\t----------------------------------- TASK_TOUCH -----------------------------------\r\n" );
    if ( firstRead ) { // ignores first reading -> MCU pin settle
      firstRead = false;
    } else {
      touchVal = touchRead(pinTouch);
      xStatus_Touch = xQueueSendToBack( xQueue_Touch, &touchState, 0 ); // Queue: Touch -> Brain
    }

    if ( xStatus_Touch == pdPASS ) {
      if ( touchVal < 40 ) {  // Touch "sensitivity" by median
        touchState = true;    // Touch detected
        Serial.print( "\t\t\t>QUEUE_TOUCH: \tSend to Touch Queue \t\t\t-> State: 1" );  Serial.print( " (" );  Serial.print( touchVal );  Serial.print( ")" );  Serial.println();
      } else {
        touchState = false;   // Nothing detected
        Serial.print( "\t\t\t>QUEUE_TOUCH: \tSend to Touch Queue \t\t\t-> State: 0" );  Serial.print( " (" );  Serial.print( touchVal );  Serial.print( ")" );  Serial.println();
      }
    } else {
      // ----- Queue state report to Serial Monitor -----
      Serial.print( "\t\t\t>QUEUE_TOUCH: \tSend to Touch Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_Touch ) == 1 ) {
        Serial.print( " \t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_Touch ) == 0 ) {
        Serial.print( " \t\t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }
    Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
    vTaskDelayUntil( &xLastWakeTime, xDelay250ms ); // "Smart" Delay (250ms)
  }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Task Handler Brain ----- */
static void vTaskHandler_Brain( void *pvParameters ) {
  // Time constants
  const uint8_t waitTime = 4; // delay = 250ms x4 -> 1s   <---- change values if time base is changed
  // PIR
  bool statePIR = false;      // PIR State
  uint8_t counterPIR = 1;     // Used for PIR state settle down
  // Bluetooth
  String pass   = "abcd";     // Entry and Alarm Password
  String recvTextBLT = "";
  String sendTextBLT = "";
  bool   flagBLT = false;     // BLT Flag
  bool   isPassOk = false;    // Entry Given/Denied Flag
  uint16_t counterResetPass = 0;  // Counter for authentication timeout
  // Touch
  uint8_t touchState = 0, nSample = 3, sampleTouch[nSample] = {0}, sumTouch = 0, avgTouch = 0;
  // Buzzer
  bool stateBuzzer = false;   // Buzzer Flag
  // LCD
  struct dataLCD d = { 1, 0 };// LCD Screen Struct
  uint8_t codeLCD = 1;        // LCD Screen Code
  int8_t counterLCD = 0;      // LCD Screen Counter
  // Others
  portBASE_TYPE xStatus_PIR, xStatus_Touch, xStatus_Buzzer, xStatus_LCD, xStatus_recvBLT, xStatus_sendBLT;
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Semaphore waits for event. Function only returns once semaphore is taken.
  for ( ;; ) {
    Serial.print( "\t--------------------------------------------------- TASK_BRAIN ---------------------------------------------------\r\n" );
    // ##################################################### QUEUE RECEIVE FROM PIR #####################################################
    xStatus_PIR = xQueueReceive( xQueue_PIR, &statePIR, 0 ); // Queue: PIR -> Brain
    // ----- Queue state report to Serial Monitor -----
    if ( xStatus_PIR == pdPASS ) {
      Serial.print( "\t>QUEUE_PIR: \tReceive from PIR Queue! \t\t\t\t<- State: TRUE\r\n" );
    } else {
      Serial.print( "\t>QUEUE_PIR: \tReceive from PIR Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_PIR ) == 1 ) {
        Serial.print( " \t\t\t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_PIR ) == 0 ) {
        Serial.print( " \t\t\t\t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }

    // PIR state settle down
    if ( statePIR ) {
      if ( counterPIR < waitTime * 2 ) { // retains movement detected state for 2s
        counterPIR++;
      } else {
        Serial.print( "\t>TASK_BRAIN: \tPIR interrupt settle down! \t\t\t\t<- State: FALSE\r\n" );
        statePIR = false;
        counterPIR = 0;
      }
    }

    // ##################################################### QUEUE RECEIVE FROM BLUETOOTH #####################################################
    if ( codeLCD == 1 || codeLCD == 6 || codeLCD == 7 ) {
      xStatus_recvBLT = xQueueReceive( xQueue_recvBLT, &recvTextBLT, 0 ); // Queue: Brain -> Bluetooth Terminal
      if ( xStatus_recvBLT == pdPASS ) {
        Serial.print( "\t>QUEUE_BLT: \tReceive from BLT Queue! \t\t\t\t<- Text:  " );  Serial.print( recvTextBLT ); Serial.println();
        if ( pass.equals( recvTextBLT ) ) { // Password Correta
          Serial.print( "\t>TASK_BRAIN: \tText received matches password!\r\n" );
          sendTextBLT = "Password matches!";
          isPassOk = true;
        } else {      // Password Incorreta
          Serial.print( "\t>TASK_BRAIN: \tText received doesn't match password!\r\n" );
          sendTextBLT = "Password doesnt match!";
          isPassOk = false;
        }
        flagBLT = true;
      } else {
        // ----- Queue state report to Serial Monitor -----
        Serial.print( "\t>QUEUE_BLT: \tReceive from BLT Queue failed!" );
        if ( uxQueueMessagesWaiting( xQueue_recvBLT ) == 1 ) {
          Serial.print( " \t\t\t\t[Queue full]\r\n" );
        } else if ( uxQueueMessagesWaiting( xQueue_recvBLT ) == 0 ) {
          Serial.print( " \t\t\t\t[Queue empty]\r\n" );
        } else {
          Serial.println();
        }
      }
    }

    // ##################################################### QUEUE RECEIVE FROM TOUCH #####################################################
    xStatus_Touch = xQueueReceive( xQueue_Touch, &touchState, 0 ); // Queue: Touch -> Brain
    // ----- Queue state report to Serial Monitor -----
    if ( xStatus_Touch == pdPASS ) {
      Serial.print( "\t>QUEUE_TOUCH: \tReceive from Touch Queue! \t\t\t\t<- State: " );   Serial.print( touchState );   Serial.println();
    } else {
      Serial.print( "\t>QUEUE_TOUCH: \tReceive from Touch Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_Touch ) == 1 ) {
        Serial.print( " \t\t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_Touch ) == 0 ) {
        Serial.print( " \t\t\t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }

    // MOVING AVERAGE FOR TOUCH VALUES -> removes outliers
    // n sample cycle holder -> 3 sample holder
    for (uint8_t i = (nSample - 1); i > 0; i-- ) {  // Saves the previous 2 values (nSample = 3)
      sampleTouch[i] = sampleTouch[i - 1];
    }
    sampleTouch[0] = touchState; // Most recent sample
    // Sum of samples
    for (uint8_t i = 0; i < nSample; i++) {
      sumTouch = sumTouch + sampleTouch[i];
    }
    // Moving average
    avgTouch = sumTouch / nSample;
    Serial.print( "\t>TASK_BRAIN: \tSum for moving average: \t\t\t\t-- Value: " );         Serial.print( sumTouch );   Serial.println();
    Serial.print( "\t>TASK_BRAIN: \tMoving average of touch state: \t\t\t\t-- Value: " );  Serial.print( avgTouch );   Serial.println();
    sumTouch = 0;  // reset sum

    // ##################################################### QUEUE SEND TO BLUETOOTH #####################################################
    if ( flagBLT ) {
      xStatus_sendBLT = xQueueSendToBack( xQueue_sendBLT, &sendTextBLT, 0 );  // Queue: Brain -> BLT
      // ----- Queue state report to Serial Monitor -----
      if ( xStatus_sendBLT == pdPASS ) {
        Serial.print( "\t>QUEUE_BLT: \tSend to BLT Queue! \t\t\t\t\t<- Text:  " );  Serial.print( sendTextBLT ); Serial.println();
      } else {
        Serial.print( "\t>QUEUE_BLT: \tSend to BLT Queue failed!" );
        if ( uxQueueMessagesWaiting( xQueue_sendBLT ) == 1 ) {
          Serial.print( " \t\t\t\t[Queue full]\r\n" );
        } else if ( uxQueueMessagesWaiting( xQueue_sendBLT ) == 0 ) {
          Serial.print( " \t\t\t\t[Queue empty]\r\n" );
        } else {
          Serial.println();
        }
      }
    }

    // ##################################################### QUEUE SEND TO BUZZER #####################################################
    xStatus_Buzzer = xQueueSendToBack( xQueue_Buzzer, &stateBuzzer, 0 );  // Queue: Brain -> Buzzer
    if ( xStatus_Buzzer == pdPASS ) {
      if ( codeLCD == 7 ) { // Alarm ON
        Serial.print( "\t>QUEUE_Buzzer: \tSend to Buzzer Queue! \t\t\t\t\t-> Value: TRUE\r\n" );
        stateBuzzer = true;
      } else {              // Alarm OFF
        Serial.print( "\t>QUEUE_Buzzer: \tSend to Buzzer Queue! \t\t\t\t\t-> Value: FALSE\r\n" );
        stateBuzzer = false;
      }
    } else {
      // ----- Queue state report to Serial Monitor -----
      Serial.print( "\t>QUEUE_Buzzer: \tSend to Buzzer Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_Buzzer ) == 1 ) {
        Serial.print( " \t\t\t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_Buzzer ) == 0 ) {
        Serial.print( " \t\t\t\t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }

    // Blue LED Settings (Normally ON; OFF when Motion Detected)
    if ( !stateBuzzer ) {
      Serial.print( "\t>TASK_BRAIN: \tSetting LED \t\t\t\t\t\t-> BLUE_LED: ON\r\n" );
      ledcWrite(chBlueLED, 255);
    } else {
      Serial.print( "\t>TASK_BRAIN: \tSetting LED \t\t\t\t\t\t-> BLUE_LED: OFF\r\n" );
      ledcWrite(chBlueLED, 0);
    }

    // ##################################################### QUEUE SEND TO LCD #####################################################
    // LCD DECIDE SCREEN
    if ( codeLCD == 1 ) {                                        // ########## Enter Password ##########
      if ( counterLCD < waitTime * 2 ) {                         // 2 * 4 -> 2s (min on screen time)
        counterLCD++;
      }
      if ( flagBLT && isPassOk && counterLCD == waitTime * 2 ) { // Right Password
        counterLCD = 0;                                          // Go to: "Password Right"
        codeLCD = 2;
      }
      if ( flagBLT && !isPassOk && counterLCD == waitTime * 2 ) {// Wrong Password
        counterLCD = 0;                                          // Go to: "Password Wrong"
        codeLCD = 3;
      }

    } else if ( codeLCD == 2 ) {                               // ########## Password Right ##########
      if ( counterLCD < waitTime * 3 ) {                       // 3 * 4 -> 4s (min on screen time)
        counterLCD++;
      } else {                                                 // Go to: "Arm Alarm" after time
        counterResetPass = 0;
        counterLCD = 0;
        codeLCD = 4;
      }

    } else if ( codeLCD == 3 ) {                               // ########## Password Wrong ##########
      if ( counterLCD < waitTime * 3 ) {                       // 3 * 4 -> 4s (min on screen time)
        counterLCD++;
      } else {                                                 // Go to: "Enter Eassword" after time
        counterLCD = 0;
        codeLCD = 1;
      }

    } else if ( codeLCD == 4 ) {                               // ########## Arm Alarm ##########
      if ( counterLCD < waitTime * 2 ) {                       // 2 * 4 -> 4s (min on screen time)
        counterLCD++;
      }
      if ( avgTouch == 1 && counterLCD == waitTime * 2 ) {     // Cap. Touch pressed -> arm alarm countdown
        counterLCD = 0;                                        // Go to: "Arming Alarm"
        codeLCD = 5;
      }
      if ( !isPassOk && counterLCD == waitTime * 2 ) {         // Authentication Timeout (1min)
        counterLCD = 0;                                        // Go to: "Enter Password"
        codeLCD = 1;
      }

    } else if ( codeLCD == 5 ) {                               // ########## Arming Alarm ##########
      if ( counterLCD < waitTime * 10 ) {                      // 10 * 4 -> 10s countdown
        counterLCD++;
      } else {                                                 // Go to: "Alarm Armed"
        counterLCD = 0;
        codeLCD = 6;
      }

    } else if ( codeLCD == 6 ) {                               // ########## Alarm Armed ##########
      if ( counterLCD < waitTime * 3 ) {                       // 3 * 4 -> 4s (min on screen time)
        counterLCD++;
      }
      if ( statePIR && counterLCD >= waitTime) {               // Motion Detected
        counterLCD = 0;                                        // Go to: "Motion Detected / Buzzer ON"
        codeLCD = 7;
      } else if ( isPassOk && counterLCD == waitTime * 3 ) {   // Right Password entered
        counterLCD = 0;                                        // Go to: "Alarm Disarmed"
        codeLCD = 8;
      }

    } else if ( codeLCD == 7 ) {                               // ########## Motion Detected / Buzzer ON ##########
      if ( counterLCD < waitTime * 2 ) {                       // 2 * 4 -> 2s (min on screen time)
        counterLCD++;
      }
      if ( isPassOk && counterLCD == waitTime * 2 ) {          // Right Password entered
        counterLCD = 0;                                        // Go to: "Alarm Disarmed"
        codeLCD = 8;
      }

    } else if ( codeLCD == 8 ) {                               // ########## Alarm Disarmed ##########
      if ( counterLCD < waitTime * 3 ) {                       // 3 * 4 -> 4s (min on screen time)
        counterLCD++;
      } else {
        counterLCD = 0;                                        // Authentication still retained (1min has not passed)
        codeLCD = 4;                                           // Go to: "Arm Alarm"
      }
    }
    d.code = codeLCD;
    d.counter = counterLCD;

    // Retain pass for 1min after entry before arming alarm (Authentication Timeout after 1min of no action taken)
    if ( isPassOk && counterResetPass < waitTime * 60 ) {      // 60 * 4 -> 60s
      counterResetPass++;
    }

    // Clear pass if Alarm ON or Timeout after 1 min past entry
    if ( codeLCD == 6 || !(isPassOk && counterResetPass < waitTime * 60) ) {
      isPassOk = false;
      counterResetPass = 0;
    }
    
   
    xStatus_LCD = xQueueSendToBack( xQueue_LCD, &d, 0 );  // Queue: Brain -> LCD
    
    if ( xStatus_LCD == pdPASS ) {
      Serial.print( "\t>QUEUE_LCD: \tSend to LCD Queue! \t\t\t\t\t-> Code:  " );  Serial.print( d.code );     Serial.println();
      Serial.print( "\t\t\t\t\t\t\t\t\t\t-> Counter: " );                         Serial.print( d.counter );
      // Shows when max value has been reached, display auxiliary string
      if ( codeLCD == 1 && counterLCD == waitTime * 2 ) {
        Serial.print( " -> OK\r\n" );
      } else if ( codeLCD == 2 && counterLCD == waitTime * 3 ) {
        Serial.print( " -> DONE\r\n" );
      } else if ( codeLCD == 3 && counterLCD == waitTime * 3 ) {
        Serial.print( " -> DONE\r\n" );
      } else if ( codeLCD == 4 && counterLCD == waitTime * 2 ) {
        Serial.print( " -> OK\r\n" );
      } else if ( codeLCD == 5 && counterLCD == waitTime * 10 ) {
        Serial.print( " -> DONE\r\n" );
      } else if ( codeLCD == 6 && counterLCD == waitTime * 3 ) {
        Serial.print( " -> OK\r\n" );
      } else if ( codeLCD == 7 && counterLCD == waitTime * 2 ) {
        Serial.print( " -> OK\r\n" );
      } else if ( codeLCD == 8 && counterLCD == waitTime * 3 ) {
        Serial.print( " -> DONE\r\n" );
      } else {
        Serial.println();
      }
    } else {
      // ----- Queue state report to Serial Monitor -----
      Serial.print( "\t>QUEUE_LCD: \tSend to LCD Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_LCD ) == 1 ) {
        Serial.print( " \t\t\t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_LCD ) == 0 ) {
        Serial.print( " \t\t\t\t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }
    Serial.print( "\t------------------------------------------------------------------------------------------------------------------\r\n" );
    flagBLT = false;
    vTaskDelayUntil( &xLastWakeTime, xDelay250ms ); // "Smart" Delay (250ms)
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Task Handler Buzzer ----- */
static void vTaskHandler_Buzzer( void *pvParameters ) {
  bool stateBuzzer = false, toggleBuzzer = false;
  portBASE_TYPE xStatus_Buzzer;
  const TickType_t xDelay250ms = 250 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    xStatus_Buzzer = xQueueReceive( xQueue_Buzzer, &stateBuzzer, 0 ); // Queue: Brain -> Buzzer
    
    Serial.print( "\t\t\t----------------------------------- TASK_BUZZER ----------------------------------\r\n" );
    if ( xStatus_Buzzer == pdPASS ) {
      if ( stateBuzzer ) {
        Serial.print( "\t\t\t>QUEUE_BUZZER: \tReceive from Buzzer Queue  \t\t<- Value: TRUE\r\n" );
        if (!toggleBuzzer) {
          Serial.print( "\t\t\t\t\t\t\t\t\t\t-> RED_LED: Blink ON Up\r\n" );
          ledcWrite(chRedLED, 255);
          Serial.print( "\t\t\t\t\t\t\t\t\t\t-> Buzzer:  High Tone\r\n" );
          ledcWrite(chBuzzer, 0);
          toggleBuzzer = true;
        } else {
          Serial.print( "\t\t\t\t\t\t\t\t\t\t-> RED_LED: Blink ON Down\r\n" );
          ledcWrite(chRedLED, 0);
          Serial.print( "\t\t\t\t\t\t\t\t\t\t-> Buzzer:  Low Tone\r\n" );
          ledcWrite(chBuzzer, 255);
          toggleBuzzer = false;
        }
      } else {
        Serial.print( "\t\t\t>QUEUE_BUZZER: \tReceive from Buzzer Queue  \t\t<- Value: FALSE\r\n" );
        Serial.print( "\t\t\t\t\t\t\t\t\t\t-> RED_LED: OFF\r\n" );
        ledcWrite(chRedLED, 0);
        Serial.print( "\t\t\t\t\t\t\t\t\t\t-> Buzzer:  OFF\r\n" );
        ledcWrite(chBuzzer, 0);
        toggleBuzzer = false;
      }
    } else {
      // ----- Queue state report to Serial Monitor -----
      Serial.print( "\t\t\t>QUEUE_BUZZER: \tReceive from Buzzer Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_Buzzer ) == 1 ) {
        Serial.print( " \t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_Buzzer ) == 0 ) {
        Serial.print( " \t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }
    Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
    vTaskDelayUntil( &xLastWakeTime, xDelay250ms ); // "Smart" Delay (250ms)
  }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Task Handler LCD ----- */
static void vTaskHandler_LCD( void *pvParameters ) {
  struct  dataLCD d;
  uint8_t newCode = 1, lastCode = 0;
  int8_t  counter = 0, trueCounter = 0, auxCounter = 0;
  portBASE_TYPE xStatus_LCD;
  const TickType_t xDelay500ms = 500 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    Serial.print( "\t\t\t-----------------------------------  TASK_LCD  -----------------------------------\r\n" );

    xStatus_LCD = xQueueReceive( xQueue_LCD, &d, 0 ); // Queue: Brain -> LCD
    newCode = d.code;
    counter = d.counter;

    // ----- Queue state report to Serial Monitor -----
    if ( xStatus_LCD == pdPASS ) {
      Serial.print( "\t\t\t>QUEUE_LCD: \tReceive from LCD Queue  \t\t<- Code:  " );  Serial.print( newCode );  Serial.println();
      Serial.print( "\t\t\t\t\t\t\t\t\t\t<- Counter: " );  Serial.print( counter );  Serial.println();
    } else {
      Serial.print( "\t\t\t>QUEUE_LCD: \tReceive from LCD Queue failed!" );
      if ( uxQueueMessagesWaiting( xQueue_LCD ) == 1 ) {
        Serial.print( " \t\t[Queue full]\r\n" );
      } else if ( uxQueueMessagesWaiting( xQueue_LCD ) == 0 ) {
        Serial.print( " \t\t[Queue empty]\r\n" );
      } else {
        Serial.println();
      }
    }

    // Print screen on LCD
    if ( newCode == 1 && lastCode != 1 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 1 - Insert password screen. #####\r\n" );       // insert pass
      tft.fillScreen(ILI9341_BLACK);    // Background color
      tft.setCursor(70, 100);
      tft.println("INSERT PASS");

    } else if ( newCode == 2 && lastCode != 2 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 2 - Password matches screen. #####\r\n" );      // pass right
      tft.fillScreen(ILI9341_BLACK);    // Background color
      tft.setCursor(70, 100);
      tft.println("ENTRY GIVEN");

    } else if ( newCode == 3 && lastCode != 3 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 3 - Password doesnt match screen. #####\r\n" ); // pass wrong
      tft.fillScreen(ILI9341_RED);    // Background color
      tft.setCursor(60, 100);
      tft.println("ENTRY DENIED");

    } else if ( newCode == 4 && lastCode != 4 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 4 - Arm alarm screen. #####\r\n" );             // arm alarm
      tft.fillScreen(ILI9341_YELLOW); // Background color
      tft.fillRoundRect(25, 95, 270, 50, 25, ILI9341_BLACK); //tft.fillRect(25,95,270,50, ILI9341_BLACK);    // x_inicio, y_inicio, x_size, y_size, color
      tft.setCursor(80, 110);  // x, y
      tft.println("ARM ALARM");

    } else if ( newCode == 5 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 5 - Arming alarm time screen. " );  Serial.print( trueCounter );  Serial.print(" #####\r\n");
      if ( newCode == 5 && lastCode != 5 ) {
        tft.fillScreen(ILI9341_RED);    // Background color
        tft.setCursor(50, 100);
        tft.println("ARMING ALARM");
      }
      
      trueCounter = map( counter, 0, 40, 10, 0 );   // Brain base time = 250ms = 1 count -> 4 counts = 1s -> 40 counts = 10s

      if ( auxCounter != trueCounter ) {
        if (trueCounter == 10) {
          tft.setCursor(136, 170);
        } else {
          tft.setCursor(142, 170);
        }
        tft.fillCircle(150, 180, 30, ILI9341_BLACK);
        tft.print(trueCounter);
      }
      
      auxCounter = trueCounter;     // Auxiliary code to prevent a screen from unecessarily refreshing

    } else if ( newCode == 6 && lastCode != 6 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 6 - Alarm armed screen. #####\r\n" );     // alarm armed
      tft.fillScreen(ILI9341_BLACK);    // Background color
      tft.setCursor(110, 100);
      tft.println("DISARM");

    } else if ( newCode == 7 && lastCode != 7 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 7 - Alarm active screen. #####\r\n" );    // alarm active with buzzer
      tft.setTextColor(ILI9341_RED);
      tft.setCursor(30, 160);
      tft.println("MOTION DETECTED");
      tft.setTextColor(ILI9341_WHITE);

    } else if ( newCode == 8 && lastCode != 8 ) {
      Serial.print( "\t\t\t>TASK_LCD: \t##### Screen 8 - Alarm disarmed. #####\r\n" );         // alarm disarmed
      tft.fillScreen(ILI9341_GREEN);  // Background color
      tft.setCursor(85, 100);
      tft.println("ALARM OFF");
    }
    lastCode = newCode;   // Auxiliary code to prevent a screen from unecessarily refreshing
    Serial.print( "\t\t\t----------------------------------------------------------------------------------\r\n" );
    vTaskDelayUntil( &xLastWakeTime, xDelay500ms ); // "Smart" Delay (500ms)
  }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/* ----- Idle Hook Callback ----- */
bool my_vApplicationIdleHook( void ) {
  return true;
}

/* ----- Loop ----- */
void loop() {
  vTaskDelete( NULL );
}
