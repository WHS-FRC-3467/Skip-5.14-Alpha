// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;

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
        //public static final int ID_ShooterLeftFollower = 16;
        public static final int ID_ShooterRightLeader = 17;
        //public static final int ID_ShooterRightFollower = 18;

        // Intake CAN IDs
        public static final int ID_IntakeMotor = 19;
        public static final int ID_IntakeCtrRoller = 20;

        // Stage CAN IDs
        public static final int ID_StageMotor = 23;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;
    }

    public static final class RobotConstants {

        public static final boolean kIsTuningMode = true;
    }

    public static final class DIOConstants {

        public static final int kStageBeamBreak = 1;
    }

    public static final class StageConstants {

        public static final double kIntakeSpeed = 0.75;
        public static final double kFeedToShooterSpeed = 1.0;
        public static final double kFeedToAmpSpeed = 0.7;
        public static final double kFeedToTrapSpeed = 0.5;

        public static final double kFeedToShooterTime = 5.0;
        public static final double kFeedToAmpTime = 5.0;
        public static final double kFeedToTrapTime = 5.0;
    }

    public static final class ShooterConstants {

        public static final double kSubwooferVelocity = 50.0;
        public static final double kPodiumVelocity = 70.0;
        public static final double kShooterTolerance = 10.0;
    }

    public static final class ArmConstants {
        // Joint Config
        public static final double LENGTH = 0.0;
        public static final double MOI = 0.0;
        public static final double CGRadius = 0.0;
        public static final double MASS = 3.0;
        public static final DCMotor MOTOR = DCMotor.getFalcon500(1).withReduction(192);

        public static final double DUTY_CYCLE_MIN = 1.0/1025.0;
        public static final double DUTY_CYCLE_MAX = 1024.0/1025.0;

        // Motor Neutral dead-band : Range 0.001 -> 0.25
        public static final double NEUTRAL_DEADBAND = 0.005;

        // Profiled PID Constants
        public static final double ARM_CRUISE = 0.0;
        public static final double ARM_ACCELERATION = 0.0;

        //PID Tollerance in Degrees
        public static final double TOLERANCE_POS = 6.0;

        public static final double ARM_OFFSET = 0.0;
        public static final double ARM_ANGLE_OFFSET = 12.4 - ARM_OFFSET;

        // Nominal Outputs
        public static final double NOMINAL_OUTPUT_FORWARD = 0;
        public static final double NOMINAL_OUTPUT_REVERSE = 0;
        public static final double PEAK_OUTPUT_FORWARD = 1.0;
        public static final double PEAK_OUTPUT_REVERSE = -1.0;

        //  Timeout ms
        public static final int TIMEOUT = 10;


        



    }

}
