# Renobot
Lego EV3 robot for mapping out the perimeter and area of a room autonomously

The RenoBot was designed to aid humans in construction and renovation. Its primary function is
to autonomously move around a space and calculate its area and perimeter. Additionally, the RenoBot will return
information regarding the amount of material needed for that space. Finally, the RenoBot can also
measure a specific angle, and then draw it out on paper. These functions can all be called by the main
menu, which is a navigable interface made for the user to interact with the robot. This main menu is how
the robot is started and shut down, and once a function is called the robot moves autonomously until it has
completed a task, and returns to the main menu.

The criteria and constraints of this project focus on both the mechanical and software side of the
robot. They drove the design of the robot by setting a requirement on size, degree of accuracy, and
autonomy.

The mechanical design of the robot was built entirely from Lego Mindstorms EV3 kit parts. The
main sub-assemblies involved in the RenoBot’s design are the chassis, the motor drive mechanism, the
touch sensor/bumper, and the pencil actuator. Major considerations that were addressed while designing
the robot were the EV3 brick placement, the ultrasonic sensor placement, and the IR sensor placement.
The overall assembly of the RenoBot was executed with its dimensions and functionality in mind. As a
result, RenoBot was built with a considerable degree of excellence in its functionality, user interface, and
overall design.

The main menu contains all the functions of the RenoBot. The menu allows the user to cycle
through all the function options. It uses functions such as getButtonValue and mainMenuScreen to make
it possible to navigate the different areas of the menu.

The main functions of the robot are called through the user interface, they call a number of
subfunctions responsible for basic functionality. For example, subfunctions such as drive_dist, and rotate
were called numerous times in all main functions. Careful consideration was given to the timing of
functions and the logic behind all calculations.

The Renobot was verified based on the stated criteria and constraints. Based on these, it
performed excellently in the demo. Apart from one small autonomy issue, which was a result of provided
hardware, everything worked to plan.
