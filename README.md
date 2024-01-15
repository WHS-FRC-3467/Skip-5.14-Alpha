
# 3467 Base Alpha Code

- This is the code base for the Alpha robot. Repos for subsequent robots will most likely be based on this code as well.
- Started with Jason Daming's improved version of the CTRE Swerve Example (see notes below).
- Added basic Subsystems for Shooter and Intake.
- Cleaned up and reorganized RobotContainer.jave to make it clearer for new programmers.
- Shooter subsystem features TunableNumber constants for tuning Velocity PID via the Shuffleboard.
- Both Shooter and Intake use Command Factories for their Commands.
- Right now, Shooter only runs and reaches specified setpoint. There is no Shooter release mechanism on the robot yet.
- Shooter uses Falcons and therefore the Phoenix 6 library. Intake uses Talon SRX, and therefore the Phoenix 5 library.
- Right now, all autonomous commands are created fully in the PathPlanner GUI. We will need to see if this will be adequate for us, or if we need to bring some commands into the code (like we did last year).
- This code uses the CTRE Swerve Generator built into Tuner X. After generating a project using the Generator, copy the newly-generated TunerConstants.java into this code.


# CTRE Swerve Example

Features:
- Improved Limelight support
- Improved PathPlanner autonomous setup
- Added SysID options for running swerve drivetrain characterization
- Different xbox joystick controller layout to try for driver comfort
- Easily scale down the max speed to help newer drivers learn
- "Turtle Mode" (left bumper) temporarily slows the drivetrain down for fine adjustments

This is an expanded version of the CTRE [SwerveWithPathPlanner](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner) example using the CTRE Swerve Builder.  To use it copy the generated/TunerConstants.java file from the generated project and replace the generated/TunerConstants.java file in this project.  You will also need to set your team number.

To use the limelight ensure you have a 36h11 pipeline properly configured and then change the [enable constant](https://github.com/jasondaming/ctre_swerve/blob/master/src/main/java/frc/robot/Vision/Limelight.java#L22) to true.

Will add specific steps on characterization after testing.
