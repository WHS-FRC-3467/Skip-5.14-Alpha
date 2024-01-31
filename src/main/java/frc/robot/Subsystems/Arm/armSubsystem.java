package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.Utils;
//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Motion Magic
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.TunableNumber;
import frc.robot.sim.PhysicsSim;

public class armSubsystem extends SubsystemBase{
    /* Hardware */
    TalonFX m_motorLeader = new TalonFX(CanConstants.ID_ArmLeader);
    TalonFX m_motorFollower = new TalonFX(CanConstants.ID_ArmFollower);
    /* Hardware */

    /* Motion Magic Configure */
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    /* Motion Magic Configure */

    /* Modify Gains */
    /*
    // The Acceleration gain, in volt seconds^2 per radian
    private static TunableNumber m_WPIkA = new TunableNumber("WPI FeedForward Arm kA", 0.00);
    // The gravity gain, in volts
    private static TunableNumber m_WPIkG = new TunableNumber("WPI FeedForward Arm kG", 0.00);
    // The static gain, in volts
    private static TunableNumber m_WPIkS = new TunableNumber("WPI FeedForward Arm kS", 0.00);
    // The velocity gain, in volt seconds per radian
    private static TunableNumber m_WPIkV = new TunableNumber("WPI FeedForward Arm kV", 0.00);
    */

    // 
    private static TunableNumber m_kP = new TunableNumber("Arm kP", 60.00);
    // 
    private static TunableNumber m_kI = new TunableNumber("Arm kI", 0.00);
    // 
    private static TunableNumber m_kD = new TunableNumber("Arm kD", 0.10);
    // 
    private static TunableNumber m_kV = new TunableNumber("Arm kV", 0.12);
    //
    private static TunableNumber m_kS = new TunableNumber("Arm kS", 0.25);

    // double calculate(double positionRadians, double velocity) Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be zero)
    // double calculate (double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) Calcualtes the feedforward from the gains and setpoints

    // Shooter Velocity setpoints (in RPS)
    private static TunableNumber m_armSetpoint = new TunableNumber("m_armSetpoint", 0.0);
    /* Modify Gains */

    /* Arm Setpoints Enumeration */
    public enum eArmPosition {
        HOME(0, "HOME"),
        AMP(1000, "AMP"),
        CLIMB(900, "CLIMB");

        private final int setpoint;
        private final String name;

        private eArmPosition(int position, String name) {
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
    /* Arm Setpoints Enumeration */

    // Track the Enumerations
    public eArmPosition m_moveToPosition;

    // Actual encoder position - updated everytime the lift is moved
    // Save this because we want the default command to run MotionMagic at the current position,
    // which will avoid sudden movements upon re-enabling the robot
    private double m_actualEncoderPosition;
    private boolean m_wasRecentlyDisabled;




    public armSubsystem() {
        /* If running in Simulation, setup simulated Falcons */
        if (Utils.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(m_motorLeader, 0.001);
            PhysicsSim.getInstance().addTalonFX(m_motorFollower, 0.001);
        }

        /* Configure the devices */
        var leadConfiguration = new TalonFXConfiguration();

        /* Configure Motion Magic */
        MotionMagicConfigs armMagic = leadConfiguration.MotionMagic;
        armMagic.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        armMagic.MotionMagicAcceleration = 10; // Takes approximately 0.6 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        armMagic.MotionMagicJerk = 50;

        /* set motors to Brake */
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Config the peak outputs */
        leadConfiguration.Voltage.PeakForwardVoltage = 12.0;
        leadConfiguration.Voltage.PeakReverseVoltage = 12.0;

        /* Update Arm Gains from TunableNumbers */
        leadConfiguration.Slot0.kP = m_kP.get();
        leadConfiguration.Slot0.kI = m_kI.get();
        leadConfiguration.Slot0.kD = m_kD.get();
        leadConfiguration.Slot0.kV = m_kV.get();
        leadConfiguration.Slot0.kS = m_kS.get();

        FeedbackConfigs fdb = leadConfiguration.Feedback;
        fdb.SensorToMechanismRatio = 192;  // This is the ratio of the arm gearbox
    
        /* Apply Configs */
        leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorLeader.getConfigurator().apply(leadConfiguration);
        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motorFollower.getConfigurator().apply(leadConfiguration);

    }

    @Override
    public void periodic() {
        /* Inside of this function we are constantly checking to see what our position currently is */
        SmartDashboard.putNumber("Arm Lead Motor Position", m_motorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Lead Motor Velocity", m_motorLeader.getVelocity().getValueAsDouble());
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

}
