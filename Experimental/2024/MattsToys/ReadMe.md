This is where Mr. Healy will throw sample code, and use for his diabolical experiments.
(Muahahaha....)

More seriously, here's what some of these samples/experiments are:

| Directory  | Description |
| ------------- | ------------- |
| `JustOneSpark-Beta3` | Port of some test code from 2023, which can be used to test CAN-connected Spark MAX controllers |
| `SimpleSimulationExample-cpp` | As simple a C++ example as I could quickly frame up (in about an hour) of setting up an abstract drivebase parent class that defines a hardware abstraction layer (HAL), and then chooses between three different *actual* (derived) drivebase types when the code runs:<ul><li>a "big bot" when we're not running under the simulator</li><li>either an XRP "little bot" or pure simulation (selected by a constant `bool` in the `RobotContainer`) when the simulator *is* being used.</li></ul>Time permitting, I'll try to make it simpler yet, though there's still some complexity to be added, since I didn't actually **implement** the drivebase class to control real hardware. |
| `SimulationExp-Beta3` | Java code for getting multiple types of drive bases running via a HAL, and then chooses between them when the code runs, including: <ul><li>"big bots" (using Quasics' standard motor layout) when we're not running under the simulator</li><li>either an XRP "little bot" (tested and working), "pure simulation" (tested and working), or a Romi (not yet tested), with the choice when under the simulator being made via an `enum` data member in the `RobotContainer`</li></ul>This also includes PID and feed-forward controls for motors (implemented in the base class), and an example of Lighting control (intended to show how that can be used trivially under simulation). |
| `SimulationExp-Beta4-cpp` | A port of the Java example (above) to C++ code, but without Romi support at present.<br/><br/>This also includes PID and feed-forward controls for motors (implemented in the abstract IDrivebase class). |
