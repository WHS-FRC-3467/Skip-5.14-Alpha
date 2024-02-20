// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.led.CANdle;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem.LEDSegment;
/**
 *  Intake a Note
 * - Run Intake & Stage
 * - Stop everything when a Note is in the stage
 */
public class intakeNote extends Command {

    IntakeSubsystem m_intakeSubsystem;
    StageSubsystem m_stageSubsystem;
    LEDSubsystem m_blinker;
    boolean m_isDone;

    /** Constructor - Creates a new intakeNote */
    public intakeNote(IntakeSubsystem intakeSub, StageSubsystem stageSub, LEDSubsystem blinker) {

        m_intakeSubsystem = intakeSub;
        m_stageSubsystem = stageSub;
        m_blinker = blinker;
        addRequirements(intakeSub, stageSub, blinker);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Turn on the Intake
        m_intakeSubsystem.runIntake(IntakeConstants.kIntakeSpeed);

        // Run the Stage until a Note is inside
        if (!m_stageSubsystem.isNoteInStage()) {
            m_stageSubsystem.runStage();
            LEDSegment.MainStrip.setColor(m_blinker.yellow);
        } else {
            m_stageSubsystem.stopStage();
            LEDSegment.MainStrip.setColor(m_blinker.white);
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
