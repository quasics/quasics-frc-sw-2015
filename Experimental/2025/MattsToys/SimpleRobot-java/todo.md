# Some things that Matt should do in this code
* Add (again) a _simple_ example of a Vision subsystem, supporting enumeration of visible targets and estimated distance/angle to each.
   * Status: in progress.
   * I need to look at replacing the use of "PhotonVision.getLatestResult" with something (preferably in a reusable base class) to try to cache data; when it's read in periodic() (at least under simulation), I frequently see 1+ targets being reported, and then "nothing visible" for 1-3 iteratons, and then the targets are visible again, despite the robot not moving.  As a result, I think that there's a possible interaction with how getAllUnreadResults() works, which is preventing the results from being cached, regardless of what the docs for "PhotonCamera.getLatestResult" seeming to suggest.

* Add a simple "drive to target" command. :-)
