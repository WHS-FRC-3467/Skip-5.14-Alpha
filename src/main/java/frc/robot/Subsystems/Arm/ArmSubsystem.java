// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Util.TunableNumber;
import frc.robot.sim.PhysicsSim;

/** Creates a new ArmSubsystem. */
public class ArmSubsystem extends SubsystemBase {
    
    // armstate enum
    public enum armState {

        armIntake(0, "INTAKE - DEFAULT POSITION"),
        armSubwoofer(0, "SHOOT FROM SUBWOOFER"),
        armAmp(60, "SCORE IN AMP"),
        armPodium(120, "SHOOT FROM PODIUM"),
        armHalfCourt(120,  "SHOOT FROM HALF COURT"),
        armClimb(150, "CLIMB");

        private final int setpoint;
        private final String name;

        private armState(int position, String name) {
            this.setpoint = position;
            this.name = name;
        }
        
        public int getSetpoint() {
            return this.setpoint;
        }

        public String getName() {
            return this.name;
        }
    }

    // Declare Motors
    private final TalonFX m_armLeader = new TalonFX(CanConstants.ID_ArmLeader);
    private final TalonFX m_armFollower = new TalonFX(CanConstants.ID_ArmFollower);

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    // Declare external arm encoder
    private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DIOConstants.ENCODER_ARM);

    /* Tunable Values */
    // Arm Setpoint
    private static TunableNumber m_armSetpoint = new TunableNumber("m_armSetpoint", 0.0);

    // For Arm PID Controller 
    private static TunableNumber m_kP = new TunableNumber("Arm kP", 1.00);
    // 
    private static TunableNumber m_kI = new TunableNumber("Arm kI", 0.00);
    // 
    private static TunableNumber m_kD = new TunableNumber("Arm kD", 0.00);

    // For ArmFeedForward
    private static TunableNumber m_kS = new TunableNumber("Arm kS", 1.0);
    //
    private static TunableNumber m_kG = new TunableNumber("Arm kG", 0.32); // Volts
    // 
    private static TunableNumber m_kV = new TunableNumber("Arm kV", 21.67); // Volts / sec / rot
    //
    private static TunableNumber m_kA = new TunableNumber("Arm kA", 0.09); // Volts / sec**2 / rot

    // For Arm Motion Magic
    private static TunableNumber m_kMM_MaxVel = new TunableNumber("MM MaxVel", 0.00); // Rot / sec
    //
    private static TunableNumber m_kMM_MaxAcc = new TunableNumber("MM MaxAcc", 0.00); // Rot / sec**2
    //
    private static TunableNumber m_kMM_Jerk = new TunableNumber("MM Jerk", 0.00); // Acceleration Derivative

    // The offset of the arm from the horizontal in its neutral position, measured from the horizontal
    public static final double kArmOffsetRads = 0.5;
 
    // Arm Feedforward
    private final ArmFeedforward m_feedforward = new ArmFeedforward( m_kS.get(), m_kG.get(), m_kV.get(), m_kA.get());

    // Current Limiting
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    // Motor braking
    private final NeutralOut m_brake = new NeutralOut();

    public ArmSubsystem() {

        /* If running in Simulation ... */
        if (Utils.isSimulation()) {
            this.simulationInit();
        }

        // Set up configurators
        TalonFXConfiguration leadConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration followConfiguration = new TalonFXConfiguration();

        // Set motors to Brake
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motor Inversion
        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Configure MotionMagic
        MotionMagicConfigs mm = leadConfiguration.MotionMagic;
        // 5 rotations per second cruise
        mm.MotionMagicCruiseVelocity = 5;
        // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicAcceleration = 10;
        // Take approximately 0.2 seconds to reach max accel
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = leadConfiguration.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kV = 0.12;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

        /*
         * // Configure the lead Talon to use a supply limit of 5 amps IF we exceed 10
         * amps for over 1 second
         * m_currentLimits.SupplyCurrentLimit = 5; // Limit to 5 amps
         * m_currentLimits.SupplyCurrentThreshold = 10; // If we exceed 10 amps
         * m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
         * m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it
         * 
         * m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
         * m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
         * leadConfiguration.CurrentLimits = m_currentLimits;
         */

        /* Config the peak outputs */
        leadConfiguration.Voltage.PeakForwardVoltage = 12.0;
        leadConfiguration.Voltage.PeakReverseVoltage = -12.0;

        /* Apply configs */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_armLeader.getConfigurator().apply(leadConfiguration);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        /* Set up follower to follow leader but in the opposite direction */
        m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putBoolean("Arm at Setpoint", isArmAtSetpoint());
        SmartDashboard.putNumber("Arm Angle", getJointDegrees());

        if (Constants.RobotConstants.tuningMode) {
            SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getJointPosition()));
            SmartDashboard.putNumber("Arm Error", getSetpointError());
            SmartDashboard.putNumber("Arm Setpoint", m_armSetpoint);
        }
        // From the Motion Magic example
        System.out.println("Pos: " + m_armLeader.getPosition());
        System.out.println("Vel: " + m_armLeader.getVelocity());

        if (Utils.isSimulation()) {
            this.simulationPeriodic();
        }

        //m_mechanisms.update(m_armLeader.getPosition(), m_armLeader.getVelocity());

    }

    /*
     * public void reset() {
     * m_upperSetpoint = getUpperJointDegrees();
     * m_controllerArm.reset(getUpperJointDegrees());
     * m_setpoint = new Setpoint(m_lowerSetpoint, m_upperSetpoint, false,
     * ClawState.IN,
     * m_lowerSetpoint, m_upperSetpoint, false, ClawState.OUT, ArmState.OTHER);
     * }
     */

    public void updateUpperSetpoint(double setpoint) {
        if (m_armSetpoint != setpoint) {
            if (setpoint < 360 && setpoint > 0) {
                m_armSetpoint = setpoint;
            }
        }
    }

    public double getSetpointError() {
        return Math.abs(m_armSetpoint - getJointDegrees());
    }

    public boolean isArmAtSetpoint() {
        return getSetpointError() < ArmConstants.TOLERANCE_POS;
    }

    /*
     * public Setpoint getSetpoint() {
     * if(m_setpoint.equals(null)){
     * reset();
     * return m_setpoint;
     * }
     * else{
     * return m_setpoint;
     * }
     * }
     */

    public void setPercentOutputUpper(double speed) {
        m_armLeader.set(speed);
    }

    public void neutralUpper() {
        m_armLeader.neutralOutput();
    }

    public double getJointPosition() {
        return m_armSensor.getAbsolutePosition();
    }

    public double getJointDegrees() {
        return dutyCycleToDegrees(getJointPosition()) + ArmConstants.ANGLE_OFFSET;
    }

    public void armDown() {
        // From the Motion Magic example
        m_armLeader.setPosition(0);
    }

    public void armSubwoofer() {
        // From the Motion Magic example
        m_armLeader.setPosition(0.25);
    }

    public void armAmp() {
        // From the Motion Magic example
        m_armLeader.setPosition(0.4);
    }

    public void armPodium() {
        // From the Motion Magic example
        m_armLeader.setPosition(0.3);
    }


        /* Manual Commands */
    public void driveManual(double speed) {
        m_motorLeader.set(speed);
        updatePosition();
    }

    public void moveArmToExactPosition(int position) {
        m_motorLeader.setPosition(position);
    }

    /* Closed Loop Motion Control */

    public void moveArmToPosition(eArmPosition position) {
        m_moveToPosition = position;
        m_motorLeader.setPosition(position.getSetpoint());
        updatePosition();

        // Now that the arm has been rer-commanded to a position, turn off flag
        m_wasRecentlyDisabled = false;
    }

    public void updatePosition() {
        m_actualEncoderPosition = m_motorLeader.getPosition().getValueAsDouble();
    }

    public double getArmPos() {
        return m_actualEncoderPosition;
    }

    // Use Current Position as Setpoint
    public void holdMagically (boolean reportStats) {

        // If robot was recently disabled and hasn't been re-commanded to a position, use actual encoder position
        if (m_wasRecentlyDisabled == true) {
            m_motorLeader.setPosition(m_actualEncoderPosition);
        } else {
            m_motorLeader.setPosition(m_moveToPosition.getSetpoint());
        }
    }


    public void simulationInit() {

            // Setup simulated leader Falcon
            PhysicsSim.getInstance().addTalonFX(m_armLeader, 0.001);

            // Put Mechanism 2d to SmartDashboard
            SmartDashboard.putData("Arm Sim", m_mech2d);
            m_armTower.setColor(new Color8Bit(Color.kBlue));

            // Simulation classes help us simulate what's going on, including gravity.
        // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
        // to 255 degrees (rotated down in the back).
        private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
                m_armGearbox,
                Constants.kArmReduction,
                SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
                Constants.kArmLength,
                Constants.kMinAngleRads,
                Constants.kMaxAngleRads,
                true,
                0,
                VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
        );
        private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

        // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
        private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
        private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
        private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
        private final MechanismLigament2d m_arm = m_armPivot.append(
                new MechanismLigament2d(
                        "Arm",
                        30,
                        Units.radiansToDegrees(m_armSim.getAngleRads()),
                        6,
                        new Color8Bit(Color.kYellow)));
    }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

}
