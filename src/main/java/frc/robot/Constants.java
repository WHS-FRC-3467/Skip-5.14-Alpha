// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be
 * used for any other purpose. All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CanConstants {

        // Drivebase CAN IDs are 1 -> 13
        // See generated/TunerConstants.java

        // Shooter CAN IDs
        public static final int ID_ShooterLeftLeader = 15;
        public static final int ID_ShooterLeftFollower = 16;
        public static final int ID_ShooterRightLeader = 17;
        public static final int ID_ShooterRightFollower = 18;

        // Intake CAN IDs
        public static final int ID_IntakeMotor = 19;
        public static final int ID_IntakeCtrRoller = 20;

        // Stage CAN IDs
        public static final int ID_StageLeft = 23;
        public static final int ID_StageRight = 24;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;

    }

    public static final class DIOConstants {
        public static final int StageBeamBreak = 0;
        public static final int ENCODER_ARM = 1;
    }

    public static final class RobotConstants {
        public static final boolean tuningMode = true;
    }

    public static final class ShooterConstants {

        public static final double kSubwooferVelocity = 50.0;
        public static final double kPodiumVelocity = 70.0;
        public static final double kShooterTolerance = 10.0;

    }

    public static final class ArmConstants {

        //Arm offsets
        public static final double VERTICAL_ANGLE_UPPER = 58.7;
    
        public static final double UPPER_ANGLE_OFFSET = 12.4 - VERTICAL_ANGLE_UPPER;

        //PID Gains
        public static final Gains GAINS_UPPER_JOINT = new Gains(0.02, 0.0, 0.0, 0.0, 50);
    
        //PID Tolerance in Degrees
        public static final double TOLERANCE_POS = 6.0;

        //(Upper) joint Config
        public static final double UPPER_LENGTH = 1.07;
        public static final double UPPER_MOI = 0.4;
        public static final double UPPER_CGRADIUS = 1.0;
        public static final double UPPER_MASS = 4.0;
        public static final DCMotor UPPER_MOTOR = DCMotor.getFalcon500(1).withReduction(144);

        /* Motor neutral dead-band : Range 0.001 -> 0.25 */
	    //public static final double NEUTRAL_DEADBAND = 0.005;

        //Nominal Outputs
        public static final double NOMINAL_OUTPUT_FORWARD = 0;
        public static final double NOMINAL_OUTPUT_REVERSE = 0;
        public static final double PEAK_VOLTAGE_FORWARD = 12.0;
        public static final double PEAK_VOLTAGE_REVERSE = -12.0;

        //Soft Limits
        public static final int FORWARD_SOFT_LIMIT_UPPER = 3300;
        public static final int REVERSE_SOFT_LIMIT_UPPER = 500;

        //Timeout ms
        public final static int TIMEOUT = 10;

        // Profiled PID Constants
        public static final double UPPER_CRUISE = 200.0;
        public static final double UPPER_ACCELERATION = 200.0;

        //Duty cycle constants
        public static final double DUTY_CYCLE_MIN = 1.0/1025.0;
        public static final double DUTY_CYCLE_MAX = 1024.0/1025.0;
  
        // Arm Setpoints
        // (Encoder position, name of command ex. Subwoofer)
        public static final double subwooferEncoder = 30;

        public static final double UPPER_JOINT_OFFSET = 58.7;

        //Measured when the lower angle is vertical using 1x1
        public static final double ANGLE_OFFSET = 12.4 - VERTICAL_ANGLE_UPPER;

        public enum ArmState{
            STOWED, DOWN, AMP, SUBWOOFER, PODIUM, HALF_COURT, SUBSTATION, INTERMEDIATE, OTHER
        }
        
        
        /*
         * Constants for moveArmCommand(maxVelocity, maxAccel, goalRotations) - what the
         * max velocity, max acceleration and the # of desired motor rotations are
         */
        public static final double maxVelocity = 4.0;
        public static final double maxAccel = 3.0;
        public static final double goalRotationsDown = 30;
        public static final double goalRotationsUp = -30;
        
        // Noah has the below constants just in case we need them later, otherwise
        // unused
        public static final double kArmDownOffset = 0.0;
        public static final double kArmUpOffset = 0.0;

        public static final double kSVolts = 1.0;
        public static final double kAVoltSecondSquaredPerRad = 1.0;
        public static final double kMaxAccelerationRadPerSecSquared = 4.0;
        public static final double kMaxVelocityRadPerSecond = 10000.0;
        public static final double kGVolts = 1.0;
        public static final double kVVoltSecondPerRad = 1.0;
        public static final double kP = 1.0;
        public static final double kArmOffsetRads = 0.0;

    }

    public static final class StageConstants {

        public static final double kStageSpeed = 1.0;

    }
}
