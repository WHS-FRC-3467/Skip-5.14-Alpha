// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be
 * used for any other purpose. All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Shooter and Arm State enumeration
    public enum SharmState {

        ss_Intake(0, 0, "INTAKE - DEFAULT POSITION"),
        ss_Subwoofer(30, 0, "SHOOT FROM SUBWOOFER"),
        ss_Amp(20, 10, "SCORE IN AMP"),
        ss_Podium(50, 5, "SHOOT FROM PODIUM"),
        ss_Wing(70, 7, "SHOOT FROM THE WING"),
        ss_PrepareToClimb(0, 10, "PREPARE TO CLIMB"),
        ss_ClimbChain(0, 0, "CLIMB ON CHAIN");

        private final int setpointShooter;
        private final int setpointArm;
        private final String name;

        private SharmState(int spShooter, int spArm, String name) {
            this.setpointShooter = spShooter;
            this.setpointArm = spArm;
            this.name = name;
        }

        public int getShooterSetpoint() {
            return this.setpointShooter;
        }
        
        public int getArmSetpoint() {
            return this.setpointArm;
        }

        public String getName() {
            return this.name;
        }
    }
    
    public static final class CanConstants {

        // Drivebase CAN IDs are 1 -> 13
        // See generated/TunerConstants.java

        // Shooter CAN IDs
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

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
        public static final int armAbsEncoder = 0;
    }

    public static final class StageConstants {

        public static final double kIntakeSpeed = 0.6;
        public static final double kFeedToShooterSpeed = 1.0;
        public static final double kFeedToAmpSpeed = 0.7;
        public static final double kFeedToTrapSpeed = 0.5;

        public static final double kFeedToShooterTime = 1.0;
        public static final double kFeedToAmpTime = 4.0;
        public static final double kFeedToTrapTime = 4.0;
    }

    public static final class IntakeConstants {

        public static final double kIntakeSpeed = 1.0;
        public static final double kEjectSpeed = -0.75;
    }

    public static final class ShooterConstants {

        public static final double kSubwooferVelocity = 50.0;
        public static final double kPodiumVelocity = 70.0;
        public static final double kShooterTolerance = 10.0;
        
        public static final double shooterSpeed = 10.0;
        public static final double podiumRangeMin = .5;
        public static final double podiumRangeMax = 1;
        public static final double subwooferRangeMin = .01;
        public static final double subwooferRangeMax = .5;
    }

    public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));



    public static final class ArmConstants {

        public static final double kS = .5;  // The Static Gain, in volts
        public static final double kG = .25;  // The Gravity Gain, in volts
        public static final double kV = 3.45;  // The Velocity Gain, in volt seconds per radian
        public static final double kA = .01;  // The acceleration gain, in volt seconds^2 per radian

        public static final DCMotor MOTOR = DCMotor.getFalcon500(1).withReduction(192);

        public static final double DUTY_CYCLE_MIN = 1.0/1025.0;
        public static final double DUTY_CYCLE_MAX = 1024.0/1025.0;

        // Motor Neutral dead-band : Range 0.001 -> 0.25
        public static final double NEUTRAL_DEADBAND = 0.005;

        // Profiled PID Constants
        public static final double ARM_CRUISE = 100.0;
        public static final double ARM_ACCELERATION = 100.0;

        //PID Tollerance in Degrees
        public static final double TOLERANCE_POS = 6.0;

        public static final double ARM_OFFSET = 0.0;

        // Nominal Outputs
        public static final double NOMINAL_OUTPUT_FORWARD = 0;
        public static final double NOMINAL_OUTPUT_REVERSE = 0;
        public static final double PEAK_OUTPUT_FORWARD = 1.0;
        public static final double PEAK_OUTPUT_REVERSE = -1.0;

        public static final double testPos0 = .35;

        //  Timeout ms
        public static final int TIMEOUT = 10;

    
        



    }

}
