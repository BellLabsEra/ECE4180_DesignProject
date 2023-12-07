# ECE 4180 Final Design Project – IMU Pencil

Repo of our final project in ECE 4180, for Fall 2023

Team Members: Elio Zebinato, Christopher Semali, Mae Frank

**Georgia Institute of Technology**

Watch our presentation and demo:

Presentation:

Demo:

These are images of our IMU Pencil breadboard:

Table of Contents

[Project Idea](#project-idea)

[Parts List](#parts-list)

[Schematic and Diagrams](#schematic-and-diagrams)

[Source Code](#source-code)

[Future Improvements](#future-improvements)

## Project Idea

Our inertia pencil will be fulfilling the requirement of building a new Mbed hardware I/O interface and developing a simple Windows VS C\# GUI application. We will use a Mbed connected to a PC that will run the demo program with the GUI display. Like in Lab 2, we will be using an I2C IMU for user-imperceptible localization. IMU captures the effects of movement on its inertial frame of reference. The benefits of using the IMU for tracking user movement of the pencil are that it is not reliant on wireless-based localization and does not require a transmitter-receiver pair to work. The IMU has a high sampling rate that results in accurate tracking of movement - however, it is known to have a drifting error as the time of tracking increases. Due to the limited time of the project, we will not have time to fully address this issue, but it is useful to keep in mind. For the software component, we will be using a series of previously calculated formulas (from the cited research paper) to implement the inertia pencil movement onto the GUI. The GUI will be developed in C\#, using Visual Studio as our platform, and it will be built on the .NET Framework like the projects created in Lab 5. Our C\# GUI will display the inertia movement as a “drawing” on the PC. In addition to the IMU, we will use other parts of our 4180 parts kit including Bluetooth, the brushless dc motors, SD cards, push buttons, and a uLCD. Bluetooth will be used to change the setting of the inertia pencil on a phone. Push buttons are an option for basic functions such as power. There are several options on how to move forward with our design. Still, the final design will happen during the creation process by adjusting the complexity of the design to fit our goals within the set timeline. Our plan for the project is to follow the outline of the cited research paper and to modify it to comply with our constraints: the 4180 parts kit and 3-week time span.

## Parts List

-   1 mbed LPC1768 (<https://www.sparkfun.com/products/retired/9564>)
-   1k Ohm Resistor (<https://www.sparkfun.com/products/14492>)
-   1 Adafruit BNO055 IMU (<https://www.adafruit.com/product/2472>)
-   H-Bridge (<https://www.sparkfun.com/products/14450>)
-   1 DC Barrel Jack Adapter (<https://www.sparkfun.com/products/10811>)
-   Brushed DC Motor (<https://www.sparkfun.com/products/13302>)
-   2 Push Buttons (<https://www.sparkfun.com/products/97>)
-   Jumper Wires (<https://www.sparkfun.com/products/12794>)

## Schematic and Diagrams

\*\*insert System Architecture and Software Flow diagrams\*\*

## Source Code

Mbed Code

```
#include "mbed.h"
…
```

Unity Code

```
#include "mbed.h"
…
```

## Future Improvements

For future improvement…

-   Current = 2D drawing
-   Future = 3D drawing
