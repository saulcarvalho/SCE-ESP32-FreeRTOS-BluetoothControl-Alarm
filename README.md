# ESP32 FreeRTOS

This repository contains all the development files for a Real-time based project implemented in an ESP32 WeMos LOLIN32 Lite with FreeRTOS.

## Project Specifications:
#### ESP32 Framework:
  - Modules that must be included in the project:
    - EUSART
    - I/O ports
    - External interrupts
    - ADC/DAC, Cap touch
    - PWM
    - I2C/SPI 
  - Modules that cannot be included in the project:
    - Timers
    - Delays or anything else that blocks the processor
#### FreeRTOS:
  - Inlude at least 5 of the following modules in the project:
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
