
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class UBIntakeSubsystem extends SubsystemBase {

    /* Initialize Talons */
    TalonFX m_intakeMotor = new TalonFX(CanConstants.ID_IntakeMotor);
    TalonSRX m_centeringLead = new TalonSRX(CanConstants.ID_IntakeLeftRoller);
    TalonSRX m_centeringFollower = new TalonSRX(CanConstants.ID_IntakeRightRoller);

    /* Current Limits config */
    //private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    /* Run Intake motor using Duty Cycle (-1.0 -> 1.0) */
    private final DutyCycleOut m_speed = new DutyCycleOut(0);

    /* Neutral output control for stopping the Intake motor */
    private final NeutralOut m_brake = new NeutralOut();

    /** Creates a new IntakeSubsystem. */
    public UBIntakeSubsystem() {

        /* Configure the Intake motor */
        var m_configuration = new TalonFXConfiguration();

        /* Set Intake motor to Brake */
        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        /* Set the Intake motor direction */
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
/*
        // Configure the motor to use a supply limit of 5 amps IF we exceed 10 amps for over 1 second
        m_currentLimits.SupplyCurrentLimit = 5; // Limit to 5 amps
        m_currentLimits.SupplyCurrentThreshold = 10; // If we exceed 10 amps
        m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 30; // Limit stator to 30 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        m_configuration.CurrentLimits = m_currentLimits;
*/

        /* Config the peak outputs */
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        /* Apply Intake motor configs */
        m_intakeMotor.getConfigurator().apply(m_configuration);

        // Set Centering motors to factory defaults
        m_centeringLead.configFactoryDefault();
        m_centeringFollower.configFactoryDefault();

        // Invert motor2 and have it follow motor1
        m_centeringFollower.follow(m_centeringLead);
        m_centeringFollower.setInverted(false);
        m_centeringLead.setInverted(true);

        // Set Centering motors to Coast
        m_centeringLead.setNeutralMode(NeutralMode.Coast);
        m_centeringFollower.setNeutralMode(NeutralMode.Coast);

        // Config ramp rate and current limit
        m_centeringLead.configOpenloopRamp(0.75);
        m_centeringLead.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));

        /* Config the peak and nominal outputs */
        m_centeringLead.configNominalOutputForward(0.0, 30);
        m_centeringLead.configNominalOutputReverse(0.0, 30);
        m_centeringLead.configPeakOutputForward(1.0, 30);
        m_centeringLead.configPeakOutputReverse(1.0, 30);

        // slows unneeded CAN status fames
        m_centeringLead.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_centeringLead.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_centeringLead.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

        m_centeringFollower.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_centeringFollower.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_centeringFollower.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Current Draw", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
    }

    /**
     * 
     * @param speed speed to set intake motor at
     */
    public void runIntake(double speed) {
        m_intakeMotor.setControl(m_speed.withOutput(speed));
        m_centeringLead.set(TalonSRXControlMode.PercentOutput, speed * 0.5);
    }

    public void stopIntake() {
        m_intakeMotor.setControl(m_brake);
        m_centeringLead.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    /*
     * Command Factories
     */
    public Command runIntakeCommand(double speed) {
        return new StartEndCommand(()->this.runIntake(speed), ()->this.stopIntake(), this);
    }

}
