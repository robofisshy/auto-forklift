# auto-forklift
We design an auto-forklift based on Visual Slam. 
The localization and control system run on Jetson TX1 made by Nvidia.
The path planning and dispatching system run on terminal---a common pc,communicate with TX1 by wifi.
The ensor data processing system run on Arduino, communicate with TX1 by I2C bus.

## Terminal<----->TX1<----->Arduino
-------------------------------------------------------------------------------------------------------
## Software architecture  

### fork_program  
----DA_Init&ensp&ensp&ensp&ensp&ensp&ensp&enspInitialization shell  
----fork  
      -----location           visual location code  
      -----multi_thread     multi thread control  
      -----motor_control    velocity smooth and motion track  
      -----detect           sensor state process  
      -----forklift         program entry  
      -----utils            some little tools  
### terminal_program  
----Terminal  
      -----Map              map building,path plan and interpolation(based on Boost Graph Library)  
      -----Communication    socket send and receive thread  
      -----Terminal         command reception thread and mapping entry  
----Terminal_multiFork  
      -----Map              on the basis of Terminal,add dynamic time window  
      -----Listener         listen thread,build Socket object for each connected fork  
      -----Socket           socket send and receive thread  
      -----Terminal         command reception thread and mapping entry  
