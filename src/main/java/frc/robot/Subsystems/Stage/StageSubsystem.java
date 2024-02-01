package frc.robot.Subsystems.Stage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.StageConstants;

public class StageSubsystem extends SubsystemBase {
    // Initialize Bags
    TalonSRX m_stageLead = new TalonSRX(CanConstants.ID_StageMotor);
    //TalonSRX m_stageFollow = new TalonSRX(CanConstants.ID_StageRight);
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
    boolean m_noteInStage = false;

    /** Creates a new IntakeSubsystem. */
    public StageSubsystem() {

        // Set motors to factory defaults
        m_stageLead.configFactoryDefault();
        //m_stageFollow.configFactoryDefault();

        // Invert motor2 and have it follow motor1
        //m_stageFollow.follow(m_stageLead);
        //m_stageFollow.setInverted(false);
        m_stageLead.setInverted(true);

        // Set motors to Brake
        m_stageLead.setNeutralMode(NeutralMode.Brake);
        //m_stageFollow.setNeutralMode(NeutralMode.Brake);

        // Config ramp rate and current limit
        m_stageLead.configOpenloopRamp(0.75);
        m_stageLead.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));

        /* Config the peak and nominal outputs */
        m_stageLead.configNominalOutputForward(0.0, 30);
        m_stageLead.configNominalOutputReverse(0.0, 30);
        m_stageLead.configPeakOutputForward(1.0, 30);
        m_stageLead.configPeakOutputReverse(1.0, 30);

        // slows unneeded CAN status fames
        m_stageLead.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_stageLead.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_stageLead.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

        //m_stageFollow.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        //m_stageFollow.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        //m_stageFollow.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run

        // Default command is to hold the note in place if sensor detects note
        // (may need to flip the boolean value)
        if (m_stageBeamBreak.get()) {
            m_noteInStage = true;
        }
        SmartDashboard.putNumber("Stage Current Draw", m_stageLead.getSupplyCurrent());
        SmartDashboard.putBoolean("Note In Stage", m_noteInStage);
    }

    /**
     * 
     * @param speed speed to set Stage motor at
     */
    public void runStage(double speed) {
        m_stageLead.set(ControlMode.PercentOutput, speed);
    }

    public void stopStage() {
        m_stageLead.set(ControlMode.PercentOutput, 0.0);
    }

    // Do not use if the shooter's target velocity is zero.
    public void ejectFront(double speed) {
        if (m_noteInStage) {
            this.runStage(speed);
        }
    }

    public void ejectBack(double speed) {
        if (m_noteInStage) {
            this.runStage((-1.0) * speed);
        }
    }

    public boolean isNoteInStage() {
        return m_noteInStage;
    }

    /*
     * Command Factories
     */

    // To Intake a Note, drive the Stage until the sensor says we have a Note
    public Command intakeNoteCommand() {
        return new InstantCommand(()-> this.runStage(StageConstants.kIntakeSpeed), this)
            .until(()->this.isNoteInStage())
            .andThen(()->this.stopStage());
    }
    
    // Pass the Note to the Shooter
    public Command feedNote2ShooterCommand() {
        return new InstantCommand(() -> this.ejectFront(StageConstants.kFeedToShooterSpeed), this)
            .withTimeout(StageConstants.kFeedToShooterTime)
            .andThen(()->this.stopStage());
    }

    // Feed the Note to the Amp
    public Command feedNote2AmpCommand() {
        return new InstantCommand(() -> this.ejectFront(StageConstants.kFeedToAmpSpeed), this)
            .withTimeout(StageConstants.kFeedToAmpTime)
            .andThen(()->this.stopStage());
    }

    // Feed the Note backwards to the Amp
    public Command ejectNote2AmpCommand() {
        return new InstantCommand(() -> this.ejectBack(StageConstants.kFeedToAmpSpeed), this)
            .withTimeout(StageConstants.kFeedToAmpTime)
            .andThen(()->this.stopStage());
    }

    // Feed the Note to the Trap
    public Command feedNote2TrapCommand() {
        return new InstantCommand(() -> this.ejectFront(StageConstants.kFeedToTrapSpeed), this)
            .withTimeout(StageConstants.kFeedToTrapTime)
            .andThen(()->this.stopStage());
    }

    // Command to just stop the Stage
    public Command stopStageCommand() {
        return new InstantCommand(() -> this.stopStage());
    }

}
