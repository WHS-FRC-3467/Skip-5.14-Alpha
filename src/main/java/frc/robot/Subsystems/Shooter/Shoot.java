package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StageConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class Shoot extends Command {
    ArmSubsystem m_arm;
    StageSubsystem m_stage;
    ShooterSubsystem m_shooter;
    boolean m_end;
    public ArmState m_armState;
    boolean m_ready;
    double shooterSpeed;
    
    // Constructor
    public Shoot(ArmSubsystem Arm, StageSubsystem Stage, ShooterSubsystem Shooter) {
        m_arm = Arm;
        m_stage = Stage;
        m_shooter = Shooter;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // The goal is to have the shooter get up to speed
        m_shooter.runShooterCommand();
        // To have the stage push the note
        m_ready = m_shooter.isWheelAtSpeed();
        if (m_ready){ // and arm in shoot position
            m_stage.ejectFrontCommand(StageConstants.kStageSpeed);
        }

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop shooting and have the arm drop to intake position.
        m_shooter.stopShooterCommand();
        m_armState = ArmState.DOWN;
        m_arm.armDown();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_end;
    }

}
