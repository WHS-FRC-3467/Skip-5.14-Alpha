package frc.robot.Subsystems.Arm;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatusFrame;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
//import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
////import edu.wpi.first.wpilibj.examples.armbotoffboard.Constants.ArmConstants;
////import edu.wpi.first.wpilibj.examples.armbotoffboard.ExampleSmartMotorController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Util.TunableNumber;

public class ArmSubsystem extends SubsystemBase {

    // Declare Motors
    private final TalonFX m_armLeader = new TalonFX(CanConstants.ArmLeft);
    private final TalonFX m_armFollower = new TalonFX(CanConstants.ArmRight);

    /* Neutral output control for disabling the Arm */
    //private final NeutralOut m_brake = new NeutralOut();

    /*
     * Gains for shooter tuning
     */
    // An error of 1 rotation per second results in 2V output
    private static TunableNumber m_kP = new TunableNumber("Shooter kP", 0.11);
    // An error of 1 rotation per second increases output by 0.5V every second
    private static TunableNumber m_kI = new TunableNumber("Shooter kI", 0.5);
    // A change of 1 rotation per second squared results in 0.01 volts output
    private static TunableNumber m_kD = new TunableNumber("Shooter kD", 0.0001);
    // Voltage-based velocity requires a velocity feed forward to account for the
    // back-emf of the motor
    // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
    // volts/RPS
    private static TunableNumber m_kV = new TunableNumber("Shooter kV", 0.12);

    private static TunableNumber m_ArmSetpoint = new TunableNumber("Arm Setpoint", 0.0);

    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    public ArmSubsystem() {
        
        /* Configure the devices */
        var leadConfiguration = new TalonFXConfiguration();
        var followConfiguration = new TalonFXConfiguration();

        /* set motors to Coast */
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /*
         * Configure the lead Talon to use a supply limit of 5 amps IF we exceed 10 amps
         * for over 1 second
         */
        m_currentLimits.SupplyCurrentLimit = 5; // Limit to 5 amps
        m_currentLimits.SupplyCurrentThreshold = 10; // If we exceed 10 amps
        m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        leadConfiguration.CurrentLimits = m_currentLimits;

        /* Config the peak outputs */
        leadConfiguration.Voltage.PeakForwardVoltage = 8.0;
        leadConfiguration.Voltage.PeakReverseVoltage = 0.0; // Don't go in reverse

        /* Update Shooter Gains from TunableNumbers */
        leadConfiguration.Slot0.kP = m_kP.get();
        leadConfiguration.Slot0.kI = m_kI.get();
        leadConfiguration.Slot0.kD = m_kD.get();
        leadConfiguration.Slot0.kV = m_kV.get();

        /* Apply configs */
        m_armLeader.getConfigurator().apply(leadConfiguration);
        m_armFollower.getConfigurator().apply(followConfiguration);

        // Invert left motor and have it follow the right motor
        m_armFollower.setInverted(false);
        m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", getArmPosition());
        SmartDashboard.putNumber("Arm Current Draw", m_armLeader.getSupplyCurrent().getValueAsDouble());

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

        m_armLeader.getConfigurator().apply(leadConfiguration);
    }

    double getArmPosition() {
        // .getPosition().getValueAsDouble() gets position in motor rotations.
        // Gearing --> 1 arm rotation = 144 motor rotations. We want Arm Degrees
        return m_armLeader.getPosition().getValueAsDouble() * 51840;
    }

    // Arm moving Function, most code copied from CTR electronics
    void moveArm(double maxVelocity, double maxAccel, double goalRotations) {
        // Trapezoid profile with max velocity of maxVelocity rps, max accel of maxVelocity rps/s
        final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
        // Final target of goalRotations rot, velocity of 0 rps
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(goalRotations, 0);
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // calculate the next profile setpoint
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        // send the request to the device
        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        m_armLeader.setControl(m_request);
        m_armFollower.setControl(m_request);
    }

    public Command moveArmCommand(double maxVelocity, double maxAccel, double goalRotations) {
        return new InstantCommand(()->this.moveArm(maxVelocity, maxAccel, goalRotations), this);
    }
}





















/** A robot arm subsystem that moves with a motion profile. */
/* public class ArmSubsystem extends TrapezoidProfileSubsystem {
  private final ExampleSmartMotorController m_motor =
      new ExampleSmartMotorController(CanConstants.ArmLeft);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. 
  public ArmSubsystem() {
    super(
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
        ArmConstants.kArmOffsetRads);
    m_motor.setPID(ArmConstants.kP, 0, 0);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setSetpoint(
        ExampleSmartMotorController.PIDMode.kPosition, setpoint.position, feedforward / 12.0);
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }
}*/

/* public class ArmSubsystem extends SubsystemBase {

    /* Hardware */
    /* TalonFX m_armLeader = new TalonFX(CanConstants.ArmLeft);
    TalonFX m_armFollower = new TalonFX(CanConstants.ArmRight);

    // Gains for tuning, Noah put this in here and we'll change it to work for the
    // arm
    // An error of 1 rotation per second results in 2V output
    private static TunableNumber m_kP = new TunableNumber("Arm kP", 0.11);
    // An error of 1 rotation per second increases output by 0.5V every second
    private static TunableNumber m_kI = new TunableNumber("Arm kI", 0.5);
    // A change of 1 rotation per second squared results in 0.01 volts output
    private static TunableNumber m_kD = new TunableNumber("Arm kD", 0.0001);
    // Voltage-based velocity requires a velocity feed forward to account for the
    // back-emf of the motor
    // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
    // volts/RPS
    private static TunableNumber m_kV = new TunableNumber("Arm kV", 0.12);

    private static TunableNumber m_ArmSetpoint = new TunableNumber("Arm Setpoint", 0.0);
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();*/

    // public ArmSubsystem() {

        /* Configure the devices */
        //var leadConfiguration = new TalonFXConfiguration();
        //var followConfiguration = new TalonFXConfiguration();

        /* set motors to Coast */
        //leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //followConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /*
         * Configure the lead Talon to use a supply limit of 5 amps IF we exceed 10 amps
         * for over 1 second
         */
        /* m_currentLimits.SupplyCurrentLimit = 5; // Limit to 5 amps
        m_currentLimits.SupplyCurrentThreshold = 10; // If we exceed 10 amps
        m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        leadConfiguration.CurrentLimits = m_currentLimits;
        */
        /* Config the peak outputs */
        //leadConfiguration.Voltage.PeakForwardVoltage = 8.0;
        //leadConfiguration.Voltage.PeakReverseVoltage = 0.0; // Don't go in reverse

        /* Update Shooter Gains from TunableNumbers */
        /* leadConfiguration.Slot0.kP = m_kP.get();
        leadConfiguration.Slot0.kI = m_kI.get();
        leadConfiguration.Slot0.kD = m_kD.get();
        leadConfiguration.Slot0.kV = m_kV.get();
            */
        /* Apply configs */
        //m_armLeader.getConfigurator().apply(leadConfiguration);
        //m_armFollower.getConfigurator().apply(followConfiguration);

        /* Set up follower to follow leader but in the opposite direction */
        //m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));

        /* m_armLeader.setSafetyEnabled(true);
    }

    @Override
    public void periodic() {
        // Puts numbers to smart dashboard
        SmartDashboard.putNumber("Arm motors Velocity", getArmVelocity());
    }
    */
    /**
     * @return the velocity of the arm in RPS
     */
    /*public double getArmVelocity() {
        return m_armLeader.getVelocity().getValueAsDouble();
    } */

    // Encoder clicks over velocity, no velocity
    // trapezoidal profile pid
