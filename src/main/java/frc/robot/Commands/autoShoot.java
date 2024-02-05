
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
/* Local */
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class autoShoot extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    StageSubsystem m_stage;
    ShooterSubsystem m_shooter;

    public autoShoot(CommandSwerveDrivetrain drivetrain, StageSubsystem stage, DoubleSupplier ARMPALCEHOLDER,
            ShooterSubsystem shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drivetrain;
        m_stage = stage;
        // m_arm = arm;
        m_shooter = shooter;

        addRequirements(m_drivetrain);
        addRequirements(m_stage);
        // addRequirements(m_arm);
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = Constants.ShooterConstants.shooterSpeed;
        double dist = m_drivetrain.calcDistToSpeaker();
        if (m_stage.isNoteInStage()) {
            if (dist > Constants.ShooterConstants.podiumRangeMin && dist < Constants.ShooterConstants.podiumRangeMax) {
                // arm to podium shot pos

            } else if (dist > Constants.ShooterConstants.subwooferRangeMin
                    && dist < Constants.ShooterConstants.subwooferRangeMax) {
                // arm to sub shot pos
            }

            if (!m_shooter.areWheelsAtSpeed()) {
                m_shooter.runShooter(speed, speed);
            } else {
                m_stage.feedNote2ShooterCommand();
            }

        } else {

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_stage.stopStage();
        // m_arm.stopArm();
        m_shooter.stopShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
