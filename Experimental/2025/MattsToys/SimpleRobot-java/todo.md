# Some things that Matt should do in this code
* Add (again) a _simple_ example of a Vision subsystem, supporting enumeration of visible targets and estimated distance/angle to each.
   * Status: in progress.
   * I need to look at replacing the use of "PhotonVision.getLatestResult" with something (preferably in a reusable base class) to try to cache data; when it's read in periodic() (at least under simulation), I frequently see 1+ targets being reported, and then "nothing visible" for an iteraton, and then the targets are visible again, despite the robot not moving.
      * This could be that we're just not guaranteed to get data fast enough from the (simulated?) camera to always have a reading, in which case any code using the target data would need to take this into account.
      * It could also be a bug someplace in my handling.

* Add a simple "drive to target" command. :-)
