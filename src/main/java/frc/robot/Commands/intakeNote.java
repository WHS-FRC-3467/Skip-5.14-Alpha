// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/* Local */
import frc.robot.Subsystems.Intake.UBIntakeSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
//import frc.robot.Subsystems.Arm.armSubsystem;

public class intakeNote extends Command {
    DoubleSupplier m_fwd, m_rev;
    UBIntakeSubsystem m_intake;
    StageSubsystem m_stage;
    //armSubsystem m_arm;
    /** Creates a new intakeNote. */

  public intakeNote(UBIntakeSubsystem intake, StageSubsystem stage, DoubleSupplier fwd, DoubleSupplier rev) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_stage = stage;
    //m_arm = arm;
    m_fwd = fwd;
    m_rev = rev;
    
    addRequirements(m_intake);
    addRequirements(m_stage);
    //addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;

    /* We will need to do some conditional checks before we can run the intake and the stage
     * 1. You need to make sure that the arm is at the Home Position
     * 2. You need to make sure that the beam break does not report 0/1 depending on the signal that states if a game piece is in the shooter
      */

    if ((speed = m_fwd.getAsDouble()) > 0.1) {
        m_intake.runIntake(speed);
        m_stage.runStage(speed);
    } else if ((speed = m_rev.getAsDouble()) > 0.1) {
        m_intake.runIntake(-speed);
        //m_stage.runStage(-speed);
    } else {
        m_intake.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_stage.stopStage();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
