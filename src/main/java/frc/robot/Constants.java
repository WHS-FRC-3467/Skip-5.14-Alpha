// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
        public static final int ID_IntakeLeftRoller = 20;
        public static final int ID_IntakeRightRoller = 21;

        // Stage CAN IDs
        public static final int ID_StageLeft = 23;
        public static final int ID_StageRight = 24;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;


    }

    public static final class PHConstants {

        public static final int IntakeForwardSolenoid = 0;
        public static final int IntakeReverseSolenoid = 1;
    }

    public static final class DIOConstants {
        public static final int EntryBeamBreak = 0;
        public static final int MidTowerBeamBreak = 1;
        public static final int UpperTowerBeamBreak = 2;
    }

    public static final class RobotConstants {

        public static final boolean tuningMode = true;

    }

    public static final class ShooterConstants {

        public static final double kSubwooferVelocity = 50.0;
        public static final double kPodiumVelocity = 70.0;
        public static final double kShooterTolerance = 10.0;


    }


}
