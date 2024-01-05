This is where Mr. Healy will throw sample code, and use for his diabolical experiments.
(Muahahaha....)

More seriously, here's what some of these samples/experiments are:

| Directory  | Description |
| ------------- | ------------- |
| `JustOneSpark-Beta3` | Port of some test code from 2023, which can be used to test CAN-connected Spark MAX controllers |
| `SimpleSimulationExample-cpp` | As simple an example as I could quickly frame in C++ of setting up an abstract drivebase parent class that defines a hardware abstraction layer (HAL), and then chooses between two different *actual* (derived) drivebase types when the code runs.<p/>Time permitting, I'll try to make it simpler yet, though there's still some complexity to be added, since I didn't actually **implement** the drivebase class to control real hardware. |
| `SimulationExp-Beta3` | Java code for getting multiple types of drive bases running ("big bots" using Quasics' standard motor layout, XRPs, Romis, and pure simulation), using an abstract base class that defines a HAL.<p/>This also includes PID and feed-forward controls for motors (implemented in the base class), and an example of Lighting control (intended to show how that can be used trivially under simulation). |
| `SimulationExp-Beta4-cpp` | C++ code for getting multiple types of drive bases running ("big bots" using Quasics' standard motor layout, XRPs, and pure simulation), using an abstract base class that defines a HAL.<p/>This also includes PID and feed-forward controls for motors (implemented in the base class). |
