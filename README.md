# Quidditch Robot

Building and coding a robot to track and retrieve balls in a game of "quidditch" ([Video](https://www.youtube.com/watch?v=ziFcgGgQwKg)).

<img src="docs/ref/robot_sideview.jpg" alt="robot_sideview" width="400"/>

## Authors

- [Jonathan Cochran](https://github.com/ionzzu)
- Cameron Retzlaff

## Background

- Timeline of 5 weeks to build, wire, program, and successfully test a “quidditch” robot
- "Quidditch" is defined as:
    - Rotate the IR Beacon to assign the robot to Team 1 or Team 2
    - If the robot bumps into an obstruction it must recover to keep playing
    - Actively track and catch balls then deposit them on the respective side
    - If a yellow ball, or “snitch,” enters the field, the robot must prioritize catching that ball
    - Must not intentionally damage other robots

## Directory

- [Algorithm](#algorithm)
    - [Tracking](#tracking)
    - [Retrieval](#retrieval)
- [PID Control](#pid-control)
- [Robot Design](#robot-design)

## Algorithm

### Tracking

- Radio transmits position of robot, balls, and other robots on the field
- Designed state machine algorithm to automatically focus robot on the closest ball
- Filters balls that are already retrieved
- Developed algorithm to prioritize capturing a ball within the “golden snitch” hue range
    - Capturing this ball "wins" the game

#### Tracking Diagram

<img src="docs/ref/balltrack.png" alt="balltrack" width="600"/>

### Retrieval

- Oriented robot and balls in same reference frame with global and robot (local) reference frames
- Determined angle between the robot and ball in the robot’s reference frame
- Developed algorithm to reorient robot's approach angle as it neared a ball
- Detected if ball within capture zone with IR distance sensors
- Closed gate and retrieved ball to home side if ball in capture zone

#### Retrieval Diagram

<img src="docs/ref/ballretrieve.png" alt="ballretrieve" width="600"/>

## PID Control

- Required to rotate a DC motor to a precise angle
- Tuned with Ziegler-Nichols method
- Selected PD controller due to fastest rise time and no overshoot
- Resulted in reaching desired angle quickly

#### PI

<img src="docs/ref/pi.png" alt="pi" width="600"/>

#### PD

<img src="docs/ref/pd.png" alt="pd" width="600"/>

#### PID

<img src="docs/ref/pid.png" alt="pid" width="600"/>

#### "No Overshoot" PID

<img src="docs/ref/pid_no_overshoot.png" alt="pid_no_overshoot" width="600"/>

## Robot Design

- Aluminum body frame, two driving wheels, and an unpowered rear drifting wheel
- Capture balls with gate created from 3D printed parts and popsicle sticks
    - Powered by a standard servo motor
- Detect if a ball is within the gate with IR distance sensors
- Impact with obstacles detected by limit switches attached to front and back bumpers
- Detect IR beacon with an IR frequency sensor, rotated with a DC motor
    - Fixed on the front of the robot
- Detect robot with camera above playing field via a unique marker atop the robot
- Broadcast position on playing field via radio to local radio chip on the robot's Arduino board

#### Robot Front View

<img src="docs/ref/robot_frontview_annotated.PNG" alt="robot_frontview_annotated" width="600"/>

#### Robot Side View

<img src="docs/ref/robot_sideview_annotated.PNG" alt="robot_sideview_annotated" width="600"/>

#### Arduino Wiring

<img src="docs/ref/arduino_wiring.jpg" alt="arduino_wiring" width="600"/>

#### 3ft x 3ft Playing Field

<img src="docs/ref/playing_field.png" alt="playing_field" width="600"/>
