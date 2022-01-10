<h2> <p align="left"> Improvement Notes </p> </h2>
<p> The following points are suggestions of improvement for <i><b>proj_code</b></i>: </p>
<ul>
    <li> <p>   
        All debug prints should be incased in a macro declared like #if DEBUG in the beginning, commenting the macro would enable/disable program prints.   
    </p> </li>
    <li> <p>  
        To improve code modularity (adding/removing specific components from the project), all the module configurations and initial states of all modules should be                       declared inside their respective task handler before the for ( :: );   <br>   
        Only task creation should be in the setup(), essentially.   
    </p> </li>
    <li> <p>   
        The touch task should have an average to pre-establish the value for the threshold instead of having a static value.   <br>   
        The threshold for the cap touch changes from ESP32 to ESP32, it needs to be manually checked. Example: One ESP32 can have threshold of 40, while another may have a 
        threshold of 25. This usually happens when using different ESP32 models but might also happen with same model - briefly tested.   
    </p> </li>
    <li> <p>   
        Some functionalities of variable resetting like the PIR settle down for 2s should be changed. The variable reset should be made inside the LCD screen where movement 
        is detected.   
    </p> </li>
    <li> <p>   
        Idle Hook function isn't being used at all. Misinterpretation of functionality.
    </p> </li>
</ul>
