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
        public static final int ShooterLeft = 15;
        public static final int ShooterRight = 16;
        public static final int IntakeLeft = 19;
        public static final int IntakeRight = 20;
        public static final int StageLeft = 17;
        public static final int StageRight = 18;
        public static final int ArmMotor = 21;
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

        public static final double kArmDownOffset = 0.0;
        public static final double kArmUpOffset = 0.0;

    }
}
