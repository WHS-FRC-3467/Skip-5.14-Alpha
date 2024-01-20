
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

    TalonFX m_motor = new TalonFX(CanConstants.IntakeMotor);

    /* Current Limits config */
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    private final DutyCycleOut m_speed = new DutyCycleOut(0);

    /* Neutral output control for stopping the Intake */
    private final NeutralOut m_brake = new NeutralOut();

    /** Creates a new IntakeSubsystem. */
    public UBIntakeSubsystem() {

        /* Configure the motor */
        var m_configuration = new TalonFXConfiguration();

        /* Set motor to Brake */
        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        /* Set the motor direction */
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


        /* Apply configs */
        m_motor.getConfigurator().apply(m_configuration);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
    }

    /**
     * 
     * @param speed speed to set intake motor at
     */
    public void runIntake(double speed) {
        m_motor.setControl(m_speed.withOutput(speed));
    }

    public void stopIntake() {
        m_motor.setControl(m_brake);
    }

    
    /*
     * Command Factories
     */
    public Command runIntakeCommand(double speed) {
        return new StartEndCommand(()->this.runIntake(speed), ()->this.stopIntake(), this);
    }

}
