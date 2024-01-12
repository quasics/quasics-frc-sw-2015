This is where Mr. Healy will throw sample code, and use for his diabolical experiments.
(Muahahaha....)

More seriously, here's what some of these samples/experiments are:

| Directory  | Description |
| ------------- | ------------- |
| `JustOneSpark-Beta3` | Port of some test code from 2023, which can be used to test CAN-connected Spark MAX controllers |
| `SimpleSimulation-cpp` | As simple a C++ example as I could quickly frame up (in about an hour) of setting up an abstract drivebase parent class that defines a hardware abstraction layer (HAL), and then chooses between three different *actual* (derived) drivebase types when the code runs:<ul><li>a "big bot" when we're not running under the simulator</li><li>either "pure simulation" or controlling an XRP "little bot" (selected by a constant `bool` in the `RobotContainer`) when the simulator *is* being used.</li></ul>Time permitting, I'll try to make it simpler yet, though there's still some complexity to be added, since I didn't actually **implement** the drivebase class to control real hardware. |
| `SimulationExp-Java` | Java code for getting multiple types of drive bases running via a HAL, and then chooses between them when the code runs, including: <ul><li>"big bots" (using Quasics' standard motor layout) when we're not running under the simulator</li><li>either "pure simulation", an XRP "little bot", or a Romi "little bot" (not yet tested) (selected via an `enum` data member in the `RobotContainer`) when the simulator *is* being used</li></ul>This example also includes:<ul><li>PID and feed-forward controls for motors (implemented in the base class) to provide more stable control of the robots (e.g., trying to compensate for motor variance on either side).</li><li>An implementation of `SysId` logging support for drive base characterization (implemented in the base class).</li><li>An example `Lighting` subsystem (intended to show how addressable LED strips are fully supported under emulation).</li></ul> |
| `SimulationExp-cpp` | A port of the Java example (above) to C++ code, but without Romi support at present.<br/><br/>This example also includes:<ul><li>PID and feed-forward controls for motors (implemented in the base class) to provide more stable control of the robots (e.g., trying to compensate for motor variance on either side).</li><li>An implementation of `SysId` logging support for drive base characterization (implemented in the base class).</li><li>An example `Lighting` subsystem (intended to show how addressable LED strips are fully supported under emulation).</li></ul> |

To-do:
* Update the sample implementations for our "big bots" to remove the use of
  `MotorControllerGroup` (which is being deprecated), in favor of "CAN
  follower" configuration.  (This will require that the controllers in our
  drive bases be updated in order to configure a motor on each side as the
  leader, which the other motor on that side will follow.)
* Complete the removal of `MotorControllerGroup` objects from the simulated
  drive base code, in favor of the `PWMMotorController` class's `addFollower`
  method.  (The Java sample has been updated, but the C++ example still
  requires modifications.)
