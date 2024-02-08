// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Intake.UBIntakeSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
/**
 *  Intake a Note
 * - Bring Arm to Intake position
 * - Run Intake & Stage
 * - Stop everything when a Note is in the stage
 */
public class intakeNote extends Command {

    ArmSubsystem m_armSubsystem;
    UBIntakeSubsystem m_intakeSubsystem;
    StageSubsystem m_stageSubsystem;
    boolean m_isDone;

    /** Constructor - Creates a new intakeNote */
    public intakeNote(ArmSubsystem armSub, UBIntakeSubsystem intakeSub, StageSubsystem stageSub) {

        m_armSubsystem = armSub;
        m_intakeSubsystem = intakeSub;
        m_stageSubsystem = stageSub;
        addRequirements(armSub, intakeSub, stageSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Make sure the Arm is in the INTAKE position
        //m_armSubsystem.updateArmSetpoint(RobotConstants.INTAKE);
        // If Arm is not down yet, return and loop back until it is.
        //if (!m_armSubsystem.isArmJointAtSetpoint())
        //    return;

        // Turn on the Intake
        m_intakeSubsystem.runIntake(IntakeConstants.kIntakeSpeed);

        // Run the Stage until a Note is inside
        if (!m_stageSubsystem.isNoteInStage()) {
            m_stageSubsystem.runStage();
        } else {
            m_stageSubsystem.stopStage();
            m_isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Turn off the Stage and Intake
        m_stageSubsystem.stopStage();
        m_intakeSubsystem.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
