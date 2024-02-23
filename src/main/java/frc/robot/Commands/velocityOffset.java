// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Micro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class velocityOffset extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    StageSubsystem m_stageSubsystem;

    boolean m_isDone;

    Pose2d robotPose;
    Translation2d currentPos;
    Double currentAngleToSpeaker;
    Translation2d futureRobotPose;
    Double futureAngleToSpeaker;
    ChassisSpeeds speeds;
    Alliance alliance;
    Double correctionAngle;
    Double timeUntilShot;
    Double xDelta;
    Double yDelta;
    Translation2d moveDelta;
    Rotation2d correctedPose;

    /** Creates a new velocityOffset. */
    public velocityOffset(CommandSwerveDrivetrain drivetrain, StageSubsystem stage) {
        m_drivetrain = drivetrain;
        m_stageSubsystem = stage;
        addRequirements(drivetrain,stage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        currentPos = m_drivetrain.getState().Pose.getTranslation();
        currentAngleToSpeaker = m_drivetrain.calcAngleToSpeaker();
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds();

        timeUntilShot = Constants.ShooterConstants.timeToShoot - m_stageSubsystem.getTimeOfShot();

        xDelta = timeUntilShot*(speeds.vxMetersPerSecond);
        yDelta = timeUntilShot*(speeds.vyMetersPerSecond);
        moveDelta = new Translation2d(xDelta,yDelta);
        futureRobotPose = currentPos.plus(moveDelta);
        futureAngleToSpeaker = m_drivetrain.calcAngleToSpeaker(futureRobotPose);

        correctionAngle = currentAngleToSpeaker - futureAngleToSpeaker;
        correctedPose = Rotation2d.fromDegrees(-correctionAngle).plus(m_drivetrain.RotToSpeaker());

        

        if (true) {
            SmartDashboard.putNumber("Robot Angle To Speaker",m_drivetrain.calcAngleToSpeaker());
            SmartDashboard.putNumber("Robot Dist To Speaker",m_drivetrain.calcDistToSpeaker());
            SmartDashboard.putNumber("xDelta", xDelta);
            SmartDashboard.putNumber("yDelta", yDelta);
            SmartDashboard.putNumber("futureang", futureAngleToSpeaker);
            SmartDashboard.putNumber("Correction Angle", correctionAngle);
            SmartDashboard.putNumber("timeUntilShot", timeUntilShot);
        }
        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }

    public Rotation2d getCorrectedTarget() {
        return this.correctedPose;
    }
}
