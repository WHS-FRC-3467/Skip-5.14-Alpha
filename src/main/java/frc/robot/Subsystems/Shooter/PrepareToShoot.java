package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Constants.ShooterConstants;

public class PrepareToShoot extends Command {
    ArmSubsystem m_arm;
    ShooterSubsystem m_shooter;
    boolean m_end;
    public ArmState m_armState;
    boolean m_ready;
    double shooterSpeed;
    
    // Constructor
    public PrepareToShoot(ArmSubsystem Arm, ShooterSubsystem Shooter, ArmState ArmState) {
        m_arm = Arm;
        m_shooter = Shooter;
        // m_armState keeps track of where the arm SHOULD go to
        m_armState = ArmState;
        if (m_armState.getSetpoint() = PODIUM) { 
            shooterSpeed = ShooterConstants.kPodiumVelocity;
        }
        //If else armState = subwoofer{ shooterSpeed = kSubwooferVelocity}
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Get the arm in the right position
        /* if armState = PODIUM {
            m_arm.armPodium();
        } if else armState = SUBWOOFER {
            m_arm.armSubwoofer();
        } */
        // The goal is to have the shooter get up to speed
        m_shooter.runShooterCommand();

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // and arm in shoot position
            // Illuminate purple led light
            //m_led.setColor(186, 0, 255); 

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        m_end = m_shooter.isWheelAtSpeed();
        return m_end;
    }

}
