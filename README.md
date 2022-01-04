# ESP32 FreeRTOS - Bluetooth Access Control & Alarm
## Description:
<p> Developed for the class Embedded Computational Systems. <br>
    The objective of this project was to create a Real-Time system using an ESP32 or an Arduino Mega with FreeRTOS.  <br>
    I ended up using the board ESP32 WeMos LOLIN32 Lite which I already had, and it became easier to work from home. </p>

## Project Specifications
<p> The project had some objectives to fulfill, which can be divided in ESP32 and FreeRTOS functionalities. </p>

### ESP32 Framework: 
Modules that must be included in the project:
  - EUSART
  - I/O ports
  - External interrupt(s)
  - ADC/DAC (ex: Cap touch)
  - PWM
  - I2C/SPI
  
Modules that cannot be included in the project:
  - Timers
  - Delays
  - Anything else that blocks the processor 
<br>

### FreeRTOS:
Include at least 5 of the following modules in the project: 
  - Tasks
  - Idle Task Hook
  - Task Priority Change
  - Task Deletion
  - Message Queues
  - Interrupts Management
  - Semaphores:
    - Binary Semaphores
    - Counting Semaphores
  - Resource Management
  - Critical Sections
  - Mutexes
