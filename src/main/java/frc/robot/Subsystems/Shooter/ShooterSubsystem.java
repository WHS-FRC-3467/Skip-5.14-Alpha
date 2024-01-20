package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter Velocity setpoint (in RPS)
    double m_setPoint = 0.0;

    /* Hardware */
    TalonFX m_motorLeftLeader = new TalonFX(CanConstants.ID_ShooterLeftLeader);
    TalonFX m_motorLeftFollower = new TalonFX(CanConstants.ID_ShooterLeftFollower);
    TalonFX m_motorRightLeader = new TalonFX(CanConstants.ID_ShooterRightLeader);
    TalonFX m_motorRightFollower = new TalonFX(CanConstants.ID_ShooterRightFollower);

    /*
     * Gains for shooter tuning
     */
    // An error of 1 rotation per second results in 2V output
    private static TunableNumber m_kP = new TunableNumber("Shooter kP", 0.11);
    // An error of 1 rotation per second increases output by 0.5V every second
    private static TunableNumber m_kI = new TunableNumber("Shooter kI", 0.5);
    // A change of 1 rotation per second squared results in 0.01 volts output
    private static TunableNumber m_kD = new TunableNumber("Shooter kD", 0.0001);
    // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
    // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts/RPS
    private static TunableNumber m_kV = new TunableNumber("Shooter kV", 0.12);

    private static TunableNumber m_ShooterSetpoint = new TunableNumber("Shooter Setpoint", 0.0);

    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    /*
     * Setup the Velocity PID control object Start at velocity 0, disable FOC, no feed forward, use slot 0
     */
    private final VelocityVoltage m_voltageVelocityLeft = new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);
    private final VelocityVoltage m_voltageVelocityRight = new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);

    /* Neutral output control for disabling the Shooter */
    private final NeutralOut m_brake = new NeutralOut();

    public ShooterSubsystem() {

        /* Configure the devices */
        var leadConfiguration = new TalonFXConfiguration();
        var followConfiguration = new TalonFXConfiguration();

        /* set motors to Coast */
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


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
        leadConfiguration.Voltage.PeakReverseVoltage = 0.0; // Don't go in reverse

        /* Update Shooter Gains from TunableNumbers */
        leadConfiguration.Slot0.kP = m_kP.get();
        leadConfiguration.Slot0.kI = m_kI.get();
        leadConfiguration.Slot0.kD = m_kD.get();
        leadConfiguration.Slot0.kV = m_kV.get();

        /* Apply configs */
        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motorLeftLeader.getConfigurator().apply(leadConfiguration);
        leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorRightLeader.getConfigurator().apply(leadConfiguration);
        m_motorLeftFollower.getConfigurator().apply(followConfiguration);
        m_motorRightFollower.getConfigurator().apply(followConfiguration);

        /* Set up followers to follow leaders but in the opposite direction */
        m_motorLeftFollower.setControl(new Follower(m_motorLeftLeader.getDeviceID(), true));
        m_motorRightFollower.setControl(new Follower(m_motorRightLeader.getDeviceID(), true));

    }

    @Override
    public void periodic() {
        // Puts numbers to smart dashboard
        SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
    }

    /**
     * Update Shooter Gains from TunableNumbers
     */
    public void updateGains() {
        var leadConfiguration = new TalonFXConfiguration();

        leadConfiguration.Slot0.kP = m_kP.get();
        leadConfiguration.Slot0.kI = m_kI.get();
        leadConfiguration.Slot0.kD = m_kD.get();
        leadConfiguration.Slot0.kV = m_kV.get();

        m_motorLeftLeader.getConfigurator().apply(leadConfiguration);
        m_motorRightLeader.getConfigurator().apply(leadConfiguration);
    }

    /**
     * @param targetVelocity the velocity in RPS of the shooter
     */
    public void runShooter(double targetVelocity) {
        // Set Velocity setpoint
        m_setPoint = targetVelocity;
        m_motorLeftLeader.setControl(m_voltageVelocityLeft.withVelocity(m_setPoint));
        m_motorRightLeader.setControl(m_voltageVelocityRight.withVelocity(m_setPoint));
    }

    public void runShooter() {
        // Get Velocity setpoint from TunableNumber
        m_setPoint = m_ShooterSetpoint.get();
        SmartDashboard.putNumber("Setpoint Request", m_setPoint);
        m_motorLeftLeader.setControl(m_voltageVelocityLeft.withVelocity(m_setPoint));
        m_motorRightLeader.setControl(m_voltageVelocityRight.withVelocity(m_setPoint));
    }

    public void stopShooter() {
        m_motorLeftLeader.setControl(m_brake);
        m_motorRightLeader.setControl(m_brake);
    }

    /**
     * @return the velocity of the shooter in RPS
     */
    public double getShooterVelocity() {
        return m_motorLeftLeader.getVelocity().getValueAsDouble();
    }

    /**
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean isWheelAtSpeed() {
        return Math.abs(m_setPoint - getShooterVelocity()) < ShooterConstants.kShooterTolerance;
    }

    /*
     * Command Factories
     */
    public Command runShooterCommand(double velocity) {
        return new StartEndCommand(()->this.runShooter(velocity), ()->this.stopShooter(), this);
    }

    public Command runShooterCommand() {
        return new StartEndCommand(()->this.runShooter(), ()->this.stopShooter(), this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(()->this.stopShooter(), this);
    }

    public Command updateShooterGainsCommand() {
        return new InstantCommand(()->this.updateGains(), this);
    }
}
