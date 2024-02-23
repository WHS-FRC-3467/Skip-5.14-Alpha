// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem.LEDSegment;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;

public class prepareToShoot extends Command {

    Setpoints m_setpoints;
    ArmSubsystem m_armSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    LEDSubsystem m_blinker;
    BooleanSupplier m_haveNote;
    boolean m_isDone;
    boolean m_runShooter;

    /** Constructor - Creates a new prepareToShoot. */
    public prepareToShoot(Setpoints setpoints, BooleanSupplier haveNote, ArmSubsystem armSub, ShooterSubsystem shootSub, LEDSubsystem blinker) {
    
        m_setpoints = setpoints;
        m_armSubsystem = armSub;
        m_shooterSubsystem = shootSub;
        m_haveNote = haveNote;
        m_blinker = blinker;

        addRequirements(armSub, shootSub, blinker);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
        if (!m_armSubsystem.isEnabled()) m_armSubsystem.enable();

        // If Shooter setpoints are zero, don't bother to check if it is up to speed
        m_runShooter = (m_setpoints.shooterLeft != 0.0 || m_setpoints.shooterRight != 0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set Shooter setpoints...
        m_shooterSubsystem.setShooterSetpoints(m_setpoints);
        // ... then run the Shooter
        m_shooterSubsystem.runShooter();

        // After we have a Note in the Stage, bring Arm to requested position
        // Don't require a Note if we are trying to STOW the arm
        if (m_haveNote.getAsBoolean() || m_setpoints.state == GameState.STOWED) {
            m_armSubsystem.updateArmSetpoint(m_setpoints);
        }

        // Exit once Arm is at setpoint and Shooter setpoint is != 0 and Shooter is up to speed 
        if (m_armSubsystem.isArmJointAtSetpoint() && (m_runShooter && m_shooterSubsystem.areWheelsAtSpeed())) {
            LEDSegment.MainStrip.ready2Shoot(0);  // Strobe Green
            m_isDone = true;
        } else {
            LEDSegment.MainStrip.notReady2Sheet(0);  // Strobe Red
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Don't turn off anything unless we have been commanded to STOWED position
        if (m_setpoints.state == GameState.STOWED) {
            m_armSubsystem.disable();
            m_shooterSubsystem.stopShooter();
        }
        LEDSegment.MainStrip.setColor(m_blinker.white);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
