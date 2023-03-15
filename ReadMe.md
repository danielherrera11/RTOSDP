DanielPlata  Entrega1 15/03/2023

In this space i want to detail the development proyect of this first task.

I used 3 LEDS rather than one RGB LED, because in my local store they did not have a RGB LED. I used one 1K resistor for each LED, in order to prevent a burnout caused by the current of the ESP32. I used female-male jumpers to connect the pins of the PCB with the protobard and finally I used male-male jumpers to connect the different parts in the protoboard

When i connect the PCB to the computer, the first LED begin to glow, it starts with low intensity, increases gradually until it has full intensity (The intensity of the LED is directly proportional with the duty cycle of the PWM used in each LED), at this point, the intensity of the LED decreases at the same rate. This process will continue for ever. When I press the botton, the first LED turn-off and the second LED begins to glow, the same happens with 2/3 and 3/1, it is a state machine.  

I used a task to control the button, the task contain a process that function indefinitely (while 1), it changed the LED that glos and has a little delay of 1000ms to aboid a debouncing problem.