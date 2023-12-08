# ECE 4180 Final Design Project – IMU Pencil

Repo of our final project in ECE 4180, for Fall 2023

Team Members: Elio Zebinato, Christopher Semali, Mae Frank

**Georgia Institute of Technology**

Watch our presentation and demo:

Presentation: <https://drive.google.com/file/d/14KUnDt1Ti68P850mrDOKhtqP7hBQmZFL/view?usp=drive_link>

Demo: <https://docs.google.com/presentation/d/1ax4yqrgerWUf8rvh5xPlwQ1ZmiABBNkj/edit?usp=sharing&ouid=108049071814918597007&rtpof=true&sd=true>

These are images of our IMU Pencil breadboard from several angles:

**![A circuit board with wires Description automatically generated](media/ead43c2eccd1bda9b37e285bcbf580d7.jpeg)**

**![A circuit board with wires Description automatically generated](media/b74ca8f37f3e9d06dd1a483bd2019379.jpeg)**

**![A circuit board with wires Description automatically generated](media/ee7bfed211b775deb1aaae06d2278b85.jpeg)**

**![A close-up of a circuit board Description automatically generated](media/425103712010be2fabcb1e60bc61da8f.jpeg)**

**Table of Contents**

[Project Idea](#project-idea)

[Parts List](#parts-list)

[Schematic and Diagrams](#schematic-and-diagrams)

[Source Code](#source-code)

[Future Improvements](#future-improvements)

## Project Idea

Our IMU pencil is a new Mbed hardware I/O interface and developing a Unity VS C\# application. The Mbed is connected to a PC that runs the C\# programs with the Unity display. Due to our knowledge from previous labs, we used an I2C IMU for user-imperceptible localization. IMU captures the effects of movement on its inertial frame of reference. The benefits of using the IMU for tracking user movement of the pencil are that it is not reliant on wireless-based localization and does not require a transmitter-receiver pair to work. The IMU has a high sampling rate that results in accurate tracking of movement - however, it is known to have a drifting error as the time of tracking increases. Due to the limited time of the project, we did not have time to fully address this issue, but it is useful to keep this in mind. The GUI will be developed on Unity’s application with VS C\# programs; it will be built on the .NET Framework like the projects created in Lab 5. Our GUI will display the IMU movement as a “drawing” on the PC. In addition to the IMU, we will use other parts of our 4180 parts kit including brushed DC motors, a dual motor driver, a DC barrel jack, resistors, and push buttons. One push button does. The other push button does. The purpose of the DC motor and driver is to indicate the power of the pencil, while the barrel jack supplies the motor's power.

## Parts List

-   Mbed LPC1768 (<https://www.sparkfun.com/products/retired/9564>)
-   1k Ohm Resistor (<https://www.sparkfun.com/products/14492>)
-   Adafruit BNO055 IMU (<https://www.adafruit.com/product/2472>)
-   Dual H-Bridge Motor Driver (<https://www.sparkfun.com/products/14450>)
-   DC Barrel Jack Adapter (<https://www.sparkfun.com/products/10811>)
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

Upon concluding this project. We achieved our main goal of building a “pencil device” with an IMU tracking system of its coordinates; along with the Mbed being able to send the IMU data to the Unity game engine. In the future, we would like to connect the IMU pencil system to Microsoft HoloLens. This will allow the user to fully experience the 3D drawing environment.
