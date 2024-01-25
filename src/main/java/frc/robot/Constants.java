// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be
 * used for any other purpose. All constants should be declared globally (i.e.
 * public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CanConstants {

        // non drivebase CAN IDs
        public static final int ID_ShooterLeftLeader = 15;
        public static final int ID_ShooterLeftFollower = 16;
        public static final int ID_ShooterRightLeader = 17;
        public static final int ID_ShooterRightFollower = 18;
        public static final int IntakeMotor = 19;
        public static final int StageLeft = 23;
        public static final int StageRight = 14;
        public static final int ArmLeft = 21;
        public static final int ArmRight = 22;
        // Delete these when merging with Wilk Branch
        public static final int ShooterLeft = 23;
        public static final int ShooterRight = 24;
        public static final int IntakeLeft = 25;
        public static final int IntakeRight = 26;
    }

    public static final class PHConstants {

        public static final int IntakeForwardSolenoid = 0;
        public static final int IntakeReverseSolenoid = 1;
    }

    public static final class DIOConstants {
        public static final int StageBeamBreak = 0;
        public static final int MidTowerBeamBreak = 1;
        public static final int UpperTowerBeamBreak = 2;
    }

    public static final class RobotConstants {

        public static final boolean tuningMode = true;

    }

    public static final class ShooterConstants {

        public static final double kSubwooferVelocity = 975.0;
        public static final double kPodiumVelocity = 1960.0;
        public static final int kShooterTolerance = 100;

    }

    public static final class ArmConstants {

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
