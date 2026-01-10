Pieces here

* ISingleMotorThing interface
   * This isn't a key piece of what I'm trying to demonstrate.  I just don't like working directly
     with classes as the "exposed types" in Java.
* SingleMotorThing
   * Core of this example
   * A base class in which all of the functionality might be implemented.
   * This classes uses interfaces or base types for any of the underlying hardware controllers/sensors
     (e.g., working with "MotorController" objects, rather than a concrete "PWMTalonFX").
   * Allocating the concrete controllers/sensors is done in derived classes (or by clients that)
     just hand stuff off to this class.
* SingleMotorThingTalon
   * Derived class from SingleMotorThing.
   * Its sole job is to create the concrete controllers/sensors (i.e., Spark Max controllers), and
     pass them in to the base class.
* SingleMotorThingSpark
   * Derived class from SingleMotorThing.
   * Its sole job is to create the concrete controllers/sensors (i.e., TalonFX controllers), and pass
     them in to the base class.
