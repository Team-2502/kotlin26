# Robot Design

This is an FRC Robot written in Kotlin for the 2026 REBUILT season.

* Orientation
    * Front is away from intake
    * Angles are counter-clockwise
* Drive
    * Four swerve modules, one on each corner of the robot
    * Forward is towards the intake
* Intake
    * Facing to the rear of the robot, but is considered "forward"
    * Spins only when trigger is pressed
    * Series of 3 motors, moves the ball from outside the robot to the shooter handoff
* Shooter
    * Flywheels should be spinning when mode is set to not-idle
    * Hood angle should be constantly adjusting for current flywheel speed
        * Requires physical zeroing on first boot
    * Anticipates future position and estimates current velocity for shoot-on-the-fly
* Turret
    * Requires physical zeroing on first boot
    * Should be tracking target when mode is set to not-idle
    * Somewhat independent physically from intake/shooter
        * No motor coordination required for operation, however the balls shot will miss unless tracking is in-sync with
          shooter
* Pose Estimation
  * Swerve is used for odometry
  * Vision has two cameras: one on the front (away from intake), and one on the side (90 degrees CCW)
  * Vision updates are applied when available from the Limelights
  * Required for field-relative driving and scoring

* Autonomous
  * Should run a pre-defined route defined by Choreo
  * Intake should spin nearly the entire time (sometimes defined by the position on the field)
  * Shooter should spin and 