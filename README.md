<h1> ESP32 FreeRTOS - Bluetooth Access Control & Alarm </h1>
<h2> Description: </h2>
<p> Developed for the class Embedded Computational Systems. <br>
    The objective of this project was to create a Real-Time system using an ESP32 or an Arduino Mega with FreeRTOS.  <br>
    I ended up using the board ESP32 WeMos LOLIN32 Lite which I already had, and it became easier to work from home. </p>
    
<p> In the end, the code ended up being used for a ESP32 Dev Board. </p>

<h2> Project Specifications </h2>
<p> The project had some objectives to fulfill, which can be divided in ESP32 and FreeRTOS functionalities. </p>

<h3>  ESP32 Framework: </h3>
<p> Modules that must be included in the project: </p>
<ul>
    <li> <p> EUSART </p> </li>  
    <li> <p> I/O ports </p> </li>  
    <li> <p> External interrupt(s) </p> </li> 
    <li> <p> ADC/DAC (ex: Cap touch) </p> </li>  
    <li> <p> PWM </p> </li>  
    <li> <p> I2C/SPI </p> </li>  
</ul>
  
Modules that cannot be included in the project:
<ul>
    <li> <p> Timers </p> </li>  
    <li> <p> Delays </p> </li>  
    <li> <p> Anything else that blocks the processor </p> </li>  
</ul>

<h3>  FreeRTOS: </h3>
Include at least 5 of the following modules in the project:
<ul>
    <li> <p> Tasks </p> </li>  
    <li> <p> Idle Task Hook </p> </li>  
    <li> <p> Task Priority Change </p> </li>  
    <li> <p> Task Deletion </p> </li>  
    <li> <p> Message Queues </p> </li>  
    <li> <p> Interrupts Management </p> </li>  
    <li> <p> Semaphores: </p> </li>  
    <ul>
        <li> <p> Binary Semaphores </p> </li>  
        <li> <p> Counting Semaphores </p> </li>  
    </ul>
    <li> <p> Resource Management </p> </li>  
    <li> <p> Critical Sections </p> </li>  
    <li> <p> Mutexes </p> </li>  
</ul>
