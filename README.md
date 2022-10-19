<h1> ESP32 FreeRTOS - Bluetooth Access Control & Alarm </h1>
<h2> Description: </h2>
<p align="justify"> <b><i>DISCLAIMER:</i></b> Developed for the class Embedded Computational Systems. The objective of this project was to create a Real-Time system using an ESP32 or an Arduino Mega with FreeRTOS. I ended up using the board ESP32 WeMos LOLIN32 Lite which I already had, and it became easier to work from home. </p>

<h2> Project Specifications </h2>
<p align="justify">  The project had some objectives to fulfill, which can be divided in ESP32 and FreeRTOS functionalities. </p>

<h3>  ESP32 Framework: </h3>
<p align="justify">  Modules that must be included in the project: </p>
<ul>
    <li> <p align="justify">  EUSART </p> </li>  
    <li> <p align="justify">  I/O ports </p> </li>  
    <li> <p align="justify">  External interrupt(s) </p> </li> 
    <li> <p align="justify">  ADC/DAC (ex: Cap touch) </p> </li>  
    <li> <p align="justify">  PWM </p> </li>  
    <li> <p align="justify">  I2C/SPI </p> </li>  
</ul>
  
Modules that cannot be included in the project:
<ul>
    <li> <p align="justify">  Timers </p> </li>  
    <li> <p align="justify">  Delays </p> </li>  
    <li> <p align="justify">  Anything else that blocks the processor </p> </li>  
</ul>

<h3>  FreeRTOS: </h3>
Include at least 5 of the following modules in the project:
<ul>
    <li> <p align="justify">  Tasks </p> </li>  
    <li> <p align="justify">  Idle Task Hook </p> </li>  
    <li> <p align="justify">  Task Priority Change </p> </li>  
    <li> <p align="justify">  Task Deletion </p> </li>  
    <li> <p align="justify">  Message Queues </p> </li>  
    <li> <p align="justify">  Interrupts Management </p> </li>  
    <li> <p align="justify">  Semaphores: </p> </li>  
    <ul>
        <li> <p align="justify">  Binary Semaphores </p> </li>  
        <li> <p align="justify">  Counting Semaphores </p> </li>  
    </ul>
    <li> <p align="justify">  Resource Management </p> </li>  
    <li> <p align="justify">  Critical Sections </p> </li>  
    <li> <p align="justify">  Mutexes </p> </li>  
</ul>
