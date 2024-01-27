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

public class StageSubsystem extends SubsystemBase {
    // Initialize Bags
    TalonSRX m_stageLead = new TalonSRX(CanConstants.ID_StageLeft);
    TalonSRX m_stageFollow = new TalonSRX(CanConstants.ID_StageRight);
    DigitalInput stageBeamBreak = new DigitalInput(DIOConstants.StageBeamBreak);
    boolean noteInStage = false;

    /** Creates a new IntakeSubsystem. */
    public StageSubsystem() {

        // Set motors to factory defaults
        m_stageLead.configFactoryDefault();
        m_stageFollow.configFactoryDefault();

        // Invert motor2 and have it follow motor1
        m_stageFollow.follow(m_stageLead);
        m_stageFollow.setInverted(false);
        m_stageLead.setInverted(true);

        // Set motors to Brake
        m_stageLead.setNeutralMode(NeutralMode.Brake);
        m_stageFollow.setNeutralMode(NeutralMode.Brake);

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

        m_stageFollow.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_stageFollow.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_stageFollow.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Stage Current Draw", m_stageLead.getSupplyCurrent());
        SmartDashboard.putBoolean("Note Not In Stage", stageBeamBreak.get());
        // Default command is to hold the note in place if sensor detects note - may
        // need to flip the boolean value
        if (!stageBeamBreak.get()) {
            noteInStage = true;
        }
    }

    /**
     * 
     * @param speed speed to set stage motor at
     */
    public void driveStage(double speed) {
        m_stageLead.set(ControlMode.PercentOutput, speed * 1.0);
    }

    public void stopStage() {
        m_stageLead.set(ControlMode.PercentOutput, 0.0);
    }

    // Do not use if the shooter's target velocity is zero.
    public void ejectFront(double speed) {
        if (noteInStage) {
            this.driveStage(speed);
        }
    }

    public void ejectBack(double speed) {
        if (noteInStage) {
            this.driveStage(-speed);
        }
    }

    /*
     * Command Factories
     */

    // Pass the note to the shooter
    // Still need to prevent this from running if shooter is not ready (do this in
    // RobotContainer or a Command Class)
    public Command ejectFrontCommand(double speed) {
        return new StartEndCommand(() -> this.ejectFront(speed), () -> this.stopStage());
    }

    // Command that holds note in place
    public Command stopStageCommand() {
        return new InstantCommand(() -> this.stopStage());
    }

    // Discard the note or Score in Trap
    public Command ejectBackCommand(double speed) {
        return new StartEndCommand(() -> this.ejectBack(speed), () -> this.stopStage());
    }

    // Still need to add operator controls on Robot Container, will wait for the
    // operator xbox controller to be set up
}
