// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.math.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;
import frc.robot.Util.ShooterPreset;
import frc.robot.Util.VisionLookUpTable;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

public class LookUpShot extends Command {

    Setpoints m_setpoints;
    ArmSubsystem m_armSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    boolean m_isDone;
    boolean isAtAngle = false;
    ShooterPreset m_shotInfo;

    /** Constructor - Creates a new prepareToShoot. */
    public LookUpShot(ArmSubsystem armSub, ShooterSubsystem shootSub, ShooterPreset shotInfo) {
        
        m_armSubsystem = armSub;
        m_shooterSubsystem = shootSub;
        m_shotInfo = shotInfo;
        

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
        SmartDashboard.putNumber("Shot Info Angle", m_shotInfo.getArmAngle());
        
        m_armSubsystem.updateArmLookUp(m_shotInfo.getArmAngle());

        // Bring Shooter to requested speed
        m_shooterSubsystem.runShooter(m_shotInfo.getLeftShooter(), m_shotInfo.getRightShooter());

        if (m_armSubsystem.isArmJointAtSetpoint()) {
            m_isDone = true;
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
