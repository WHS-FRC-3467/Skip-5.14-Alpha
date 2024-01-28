// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDefault extends Command {

    DoubleSupplier m_fwd, m_rev;
    UBIntakeSubsystem m_intake;

    /** Creates a new IntakeDefault. */
    public IntakeDefault(UBIntakeSubsystem intake, DoubleSupplier fwd, DoubleSupplier rev) {
        m_intake = intake;
        m_fwd = fwd;
        m_rev = rev;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double speed;

        if ((speed = m_fwd.getAsDouble()) > 0.1) {
            m_intake.runIntake(speed);
        } else if ((speed = m_rev.getAsDouble()) > 0.1) {
            m_intake.runIntake(-speed);
        } else {
            m_intake.stopIntake();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
