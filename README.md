# WRO_Future_Engineers_Saturn_NCD
The Robot vehicle chassis design is built using LEGO pieces. It is provided with Arduino Mega microcontroller and a Raspberry Pi 4 controller. The microcontroller is provided with five ultrasonic sensors, and the controller is provided with Raspberry pi camera V2. Two motors are used for movement, Servo motor for Acherman steering and a DC motor (j sumo 1000 rpm) for driving where the movement is distributed to the back wheels by a differential gear. Li-Poly RC batteries are used for supply purposes.

For the qualifying matches, the algorithm is programmed by Arduino c (look Round_1_2.ino). The robot follows the wall along the straight section according to the distances measured by the two side sensors. The front sensor and one of the side-front sensors determine the direction of turning (right or left).

For the final matches, Raspberry Pi detects the colour of billars (image processing) and sends a specific string to the Arduino using serial connection (look RaspberryPI_Python_CODE.py). The distance between the robot and the billar is calculated depending on the size of billar's pixels. Avoiding cube is also  programmed by Arduino c (look Round_3_4.ino). The Raspberry Pi sends another character to Arduino to complete wall following.

Finally, the parking in all matches depends on the value of a specific counter and the distance measured by the front sensor.

## License

This project is licensed under the Creative Commons Attribution 4.0 International License. For more details, you can view the [full license](https://creativecommons.org/licenses/by/4.0/).
