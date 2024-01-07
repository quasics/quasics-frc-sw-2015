This is where Mr. Healy will throw sample code, and use for his diabolical experiments.
(Muahahaha....)

More seriously, here's what some of these samples/experiments are:

| Directory  | Description |
| ------------- | ------------- |
| `JustOneSpark-Beta3` | Port of some test code from 2023, which can be used to test CAN-connected Spark MAX controllers |
| `SimpleSimulation-cpp` | As simple a C++ example as I could quickly frame up (in about an hour) of setting up an abstract drivebase parent class that defines a hardware abstraction layer (HAL), and then chooses between three different *actual* (derived) drivebase types when the code runs:<ul><li>a "big bot" when we're not running under the simulator</li><li>either "pure simulation" or controlling an XRP "little bot" (selected by a constant `bool` in the `RobotContainer`) when the simulator *is* being used.</li></ul>Time permitting, I'll try to make it simpler yet, though there's still some complexity to be added, since I didn't actually **implement** the drivebase class to control real hardware. |
| `SimulationExp-Beta3` | Java code for getting multiple types of drive bases running via a HAL, and then chooses between them when the code runs, including: <ul><li>"big bots" (using Quasics' standard motor layout) when we're not running under the simulator</li><li>either "pure simulation", an XRP "little bot", or a Romi "little bot" (not yet tested) (selected via an `enum` data member in the `RobotContainer`) when the simulator *is* being used</li></ul>This also includes PID and feed-forward controls for motors (implemented in the base class) to provide more stable control of the robots (e.g., trying to compensate for motor variance on either side), and an example `Lighting` subsystem (intended to show how addressable LED strips are fully supported under emulation). |
| `SimulationExp-Beta4-cpp` | A port of the Java example (above) to C++ code, but without Romi support at present.<br/><br/>This also includes PID and feed-forward controls for motors (implemented in the abstract IDrivebase class), and an example `Lighting` subsystem. |
