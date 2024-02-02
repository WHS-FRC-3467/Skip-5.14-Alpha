// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

/* Local */
import frc.robot.Subsystems.Stage.StageSubsystem;
//import frc.robot.Subsystems.Arm.armSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class shootNote extends Command {
  /** Creates a new shootNote. */
  StageSubsystem m_stage;
  //armSubsystem m_arm;
  ShooterSubsystem m_shooter;
  public shootNote(StageSubsystem stage, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_stage = stage;
    //m_arm = arm;
    m_shooter = shooter;
    addRequirements(m_shooter);
    addRequirements(m_stage);
    //addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
