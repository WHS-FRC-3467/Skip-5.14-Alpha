// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.Setpoints;
import frc.robot.Util.ShooterPreset;
import frc.robot.Util.VisionLookUpTable;

public class LookUpShot extends Command {

    Setpoints m_setpoints;
    ArmSubsystem m_armSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    boolean m_isDone;
    boolean isAtAngle = false;
    ShooterPreset m_shotInfo;
    VisionLookUpTable m_VisionLookUpTable;
    DoubleSupplier m_distance;
    

    /** Constructor - Creates a new prepareToShoot. */
    public LookUpShot(ArmSubsystem armSub, ShooterSubsystem shootSub, DoubleSupplier distance) {
        
        m_armSubsystem = armSub;
        m_shooterSubsystem = shootSub;
        m_VisionLookUpTable = new VisionLookUpTable();
        m_distance = distance;
        //m_shotInfo = m_VisionLookUpTable.getShooterPreset(distance);
        

        addRequirements(armSub, shootSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //shotInfo = mVisionLookUpTable.getShooterPreset((m_Drivetrain.calcDistToSpeaker()));
        m_isDone = false;
        if (!m_armSubsystem.isEnabled()) m_armSubsystem.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shotInfo = m_VisionLookUpTable.getShooterPreset(m_distance.getAsDouble());
        SmartDashboard.putNumber("LookUp Distance", m_distance.getAsDouble());
        SmartDashboard.putNumber("Shot Info Angle", m_shotInfo.getArmAngle());
        
        m_armSubsystem.updateArmLookUp(m_shotInfo.getArmAngle());

        // Bring Shooter to requested speed
        m_shooterSubsystem.runShooter(m_shotInfo.getLeftShooter(), m_shotInfo.getRightShooter());

        if (m_armSubsystem.isArmJointAtSetpoint()) {
            //m_isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Don't turn off anything unless we have been commanded to STOWED position
        //m_armSubsystem.disable();
        m_shooterSubsystem.stopShooter();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
