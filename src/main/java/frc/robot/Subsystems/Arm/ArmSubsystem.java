// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

/** Creates a new ArmSubsystem. */
public class ArmSubsystem extends SubsystemBase {

    private final TalonFX m_fxLeader = new TalonFX(Constants.CanConstants.ID_ArmLeader);
    private final TalonFX m_fxFollower = new TalonFX(Constants.CanConstants.ID_ArmFollower);
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public ArmSubsystem() {

        /* If running in Simulation, setup simulated Falcons */
        if (Utils.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(m_fxLeader, 0.001);
            PhysicsSim.getInstance().addTalonFX(m_fxFollower, 0.001);
        }

        /* Configure the devices */
        var leadConfiguration = new TalonFXConfiguration();
        var followConfiguration = new TalonFXConfiguration();

        /* set motors to Brake */
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /* Configure MotionMagic */
        MotionMagicConfigs mm = leadConfiguration.MotionMagic;
        // 5 rotations per second cruise
        mm.MotionMagicCruiseVelocity = 5;
        // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicAcceleration = 10;
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 12.8;


/*
        // Configure the lead Talon to use a supply limit of 5 amps IF we exceed 10 amps for over 1 second
        m_currentLimits.SupplyCurrentLimit = 5; // Limit to 5 amps
        m_currentLimits.SupplyCurrentThreshold = 10; // If we exceed 10 amps
        m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        leadConfiguration.CurrentLimits = m_currentLimits;
*/

        /* Config the peak outputs */
        leadConfiguration.Voltage.PeakForwardVoltage = 12.0;
        leadConfiguration.Voltage.PeakReverseVoltage = -12.0;

        /* Apply configs */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_fxLeader.getConfigurator().apply(leadConfiguration);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        /* Set up follower to follow leader but in the opposite direction */
        m_fxFollower.setControl(new Follower(m_fxLeader.getDeviceID(), true));


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // If running in simulation, update the sims
        if (Utils.isSimulation()) {
            PhysicsSim.getInstance().run();
        }
    }

    public void positionArm(double position) {
        m_fxLeader.setControl(m_mmReq.withPosition(position).withSlot(0));
    }

    public void returnArm() {
        m_fxLeader.setPosition(1);
    }

}
