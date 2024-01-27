
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;

public class OBIntakeSubsystem extends SubsystemBase {

    // Initializes solenoid and talons
    DoubleSolenoid m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PHConstants.IntakeForwardSolenoid,
            PHConstants.IntakeReverseSolenoid);
    TalonSRX m_motorLead = new TalonSRX(CanConstants.ID_IntakeLeftRoller);
    TalonSRX m_motorFollow = new TalonSRX(CanConstants.ID_IntakeRightRoller);

    /** Creates a new IntakeSubsystem. */
    public OBIntakeSubsystem() {

        // Set motors to factory defaults
        m_motorLead.configFactoryDefault();
        m_motorFollow.configFactoryDefault();

        // Invert motor2 and have it follow motor1
        m_motorFollow.follow(m_motorLead);
        m_motorFollow.setInverted(false);
        m_motorLead.setInverted(true);

        // Set motors to Coast
        m_motorLead.setNeutralMode(NeutralMode.Coast);
        m_motorFollow.setNeutralMode(NeutralMode.Coast);

        // Config ramp rate and current limit
        m_motorLead.configOpenloopRamp(0.75);
        m_motorLead.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));

        /* Config the peak and nominal outputs */
        m_motorLead.configNominalOutputForward(0.0, 30);
        m_motorLead.configNominalOutputReverse(0.0, 30);
        m_motorLead.configPeakOutputForward(1.0, 30);
        m_motorLead.configPeakOutputReverse(1.0, 30);

        // slows unneeded CAN status fames
        m_motorLead.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_motorLead.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_motorLead.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

        m_motorFollow.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_motorFollow.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_motorFollow.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Current Draw", m_motorLead.getSupplyCurrent());
    }

    /**
     * 
     * @param speed speed to set intake motor at
     */
    public void driveIntake(double speed) {
        m_motorLead.set(ControlMode.PercentOutput, speed * 1.0);
    }

    // deploys intake
    public void deployIntake() {
        m_intakePiston.set(Value.kReverse);
    }

    // retracts intake
    public void retractIntake() {
        m_intakePiston.set(Value.kForward);
    }

    // toggles intake position
    public void toggleIntake() {
        // Value.kReverse = retracted
        if (m_intakePiston.get() == Value.kReverse) {
            m_intakePiston.set(Value.kForward);
        } else {
            m_intakePiston.set(Value.kReverse);
        }
    }
    
    /*
     * Command Factories
     */
    public Command runIntakeCommand(DoubleSupplier speed_Supplier) {
        return new StartEndCommand(()->this.driveIntake(speed_Supplier.getAsDouble()), ()->this.driveIntake(0.0), this);
    }

    public Command deployIntakeCommand() {
        return new InstantCommand(()->this.deployIntake(), this);
    }

    public Command retractIntakeCommand() {
        return new InstantCommand(()->this.retractIntake(), this);
    }

    public Command toggleIntakeCommand() {
        return new InstantCommand(()->this.toggleIntake(), this);
    }
}
