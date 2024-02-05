// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;


public class moveToSetpoint extends Command {
    ArmSubsystem m_arm;
  /** Creates a new moveToSetpoint. */
  public moveToSetpoint(ArmSubsystem arm) {
    m_arm = arm;

    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.updateArmSetpoint(360*.35);
    m_arm.runArmProfiled();
    if (m_arm.getArmJointAtSetpoint()) {
        isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setPercentOutputUpper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
