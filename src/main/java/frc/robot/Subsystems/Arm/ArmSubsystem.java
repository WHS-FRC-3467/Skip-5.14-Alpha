package frc.robot.Subsystems.Arm;

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
import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;

//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
//import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.PHConstants;
import frc.robot.Subsystems.Arm.Mechanisms;
import frc.robot.Util.TunableNumber;

public class ArmSubsystem extends SubsystemBase {
    
    // Armstate enum
    public enum armState {
        armIntake(0, "INTAKE - DEFAULT POSITION"),
        armAmp(60, "SCORE IN AMP"),
        armSubwoofer(120, "SHOOT FROM SUBWOOFER"),
        armPodium(120, "SHOOT FROM PODIUM"),
        armClimb(150, "CLIMB"),
        armHalfCourt(120,  "SHOOT FROM HALF COURT"),
        armNoneOfTheAbove(130, "Not in any of the postition");

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
    private final TalonFX m_armLeader = new TalonFX(CanConstants.ArmLeft);
    private final TalonFX m_armFollower = new TalonFX(CanConstants.ArmRight);
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    private final XboxController m_joystick = new XboxController(0);

    // Declare external arm encoder - this is the start of the degree measuring
    private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DIOConstants.ENCODER_ARM);


    // Declare Enumerator
    public armState m_currentPos;
    public armState m_desiredPos;

    private FeedbackConfigs FeedbackSensorSource;
    private FeedbackConfigs FeedbackRotorOffset;
    private final FeedbackConfigs sensorConfiguration = new FeedbackConfigs();

    private TrapezoidProfile.Constraints armConstraints;

    private ProfiledPIDController m_controllerArm;

    private JointConfig joint_Arm = new JointConfig(ArmConstants.UPPER_MASS, ArmConstants.UPPER_LENGTH,
      ArmConstants.UPPER_MOI, ArmConstants.UPPER_CGRADIUS, ArmConstants.UPPER_MOTOR);
      
    private final ArmFeedforward m_feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    private double m_upperSetpoint;

    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    private final NeutralOut m_brake = new NeutralOut();

  public ArmSubsystem() {
    
    // From the Motion Magic example -- Configuration
    var leadConfiguration = new TalonFXConfiguration();
    var followConfiguration = new TalonFXConfiguration();

    /* set motors to Brake */
    leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Make the right motor follow the other */
    m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));

    /* Configure current limits */
    MotionMagicConfigs mm = leadConfiguration.MotionMagic;
    mm.MotionMagicCruiseVelocity = 10; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = leadConfiguration.Slot0;
    slot0.kP = 60;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = leadConfiguration.Feedback;
    fdb.SensorToMechanismRatio = 144.0;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = m_armLeader.getConfigurator().apply(leadConfiguration);
      if (status.isOK()) break;
    }
    
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());}
    }
    private int m_printCount = 0;
    private final Mechanisms m_mechanisms = new Mechanisms();

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
    leadConfiguration.CurrentLimits = m_currentLimits; */


    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_armLeader.setSelectedSensorPosition(dutyCycleToCTREUnits(getArmPos()), 0, ArmConstants.TIMEOUT);

    SmartDashboard.putBoolean("Arm at Setpoint", getUpperAtSetpoint());
    SmartDashboard.putNumber("Arm Angle", getArmJointDegrees());

    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getArmPos()));
      SmartDashboard.putNumber("Arm Error", getUpperError());
      SmartDashboard.putNumber("Arm Setpoint", m_upperSetpoint);
    } 
      // From the Motion Magic example
    if (m_printCount++ > 10) {
        m_printCount = 0;
        System.out.println("Pos: " + m_armLeader.getPosition());
        System.out.println("Vel: " + m_armLeader.getVelocity());
        System.out.println();
      }
      m_mechanisms.update(m_armLeader.getPosition(), m_armLeader.getVelocity());

    /* Deadband the joystick */
    double leftY = m_joystick.getLeftY();
    if(leftY > -0.1 && leftY < 0.1) leftY = 0;

    m_armLeader.setControl(m_mmReq.withPosition(leftY * 10).withSlot(0));
    if(m_joystick.getBButton()) {
      armDown();
    }
  }

  /*public void reset() {
    m_upperSetpoint = getArmJointDegrees();
    m_controllerArm.reset(getArmJointDegrees());
    m_setpoint = new Setpoint(m_lowerSetpoint, m_upperSetpoint, false, ClawState.IN, 
                              m_lowerSetpoint, m_upperSetpoint, false, ClawState.OUT, ArmState.OTHER);
  }*/

  /*public void updateUpperSetpoint(double setpoint) {
    if (m_upperSetpoint != setpoint) {
      if (setpoint < 360 && setpoint > 0) {
        m_upperSetpoint = setpoint;
      }
    }
  }*/

    // System.out.println("Upper PID" + pidOutput);
    // if(Math.abs(pidOutput) > 0.01 && Math.abs(pidOutput)<0.045){
    //   pidOutput = Math.copySign(0.045, pidOutput);
    // }
    //m_armLeader.set(pidOutput + ff); // may need to negate ff voltage to get desired output }

    // if(Math.abs(pidOutput) > 0.01 && Math.abs(pidOutput)<0.04){
    //   pidOutput = Math.copySign(0.04, pidOutput);
    // }
    // System.out.println("Lower PID" + pidOutput);
  
  
  public double getUpperError(){
    return Math.abs(m_upperSetpoint - getArmJointDegrees());
  }

  public boolean getUpperAtSetpoint() {
    return getUpperError() < ArmConstants.TOLERANCE_POS;
  }

  
  /* public Setpoint getSetpoint() {
    if(m_setpoint.equals(null)){
      reset();
      return m_setpoint;
    }
    else{
      return m_setpoint;
    }
  }*/

  public void setPercentOutputUpper(double speed) {
    m_armLeader.set(speed);
  }


  public void neutralUpper() {
    //m_armLeader.neutralOutput();
  }

  public double getArmPos() {
    return m_armEncoder.getAbsolutePosition();
  }

  public double getArmJointDegrees() {
    return dutyCycleToDegrees(getArmPos()) + ArmConstants.ANGLE_OFFSET;
  }

  public double dutyCycleToCTREUnits(double dutyCyclePos) {
    // 4096 units per rotation = raw sensor units for Pulse width encoder
    return dutyCyclePos * 4096;
  }

  public double dutyCycleToDegrees(double dutyCyclePos) {
    return dutyCyclePos * 360;
  }

  // Command methods, will get their own files
  public void armDown() {
      // From the Motion Magic example
    m_armLeader.setPosition(0);
    armState m_armState = armState.armIntake;
  }

    public void SubwooferPos() {
      // From the Motion Magic example
    m_armLeader.setPosition(0.25);
    armState m_armState = armState.armSubwoofer;
  }
    public void armAmpPos() {
      // From the Motion Magic example
    m_armLeader.setPosition(0.4);
    armState m_armState = armState.armAmp;

  }

  public void armPodiumPos() {
      // From the Motion Magic example
    m_armLeader.setPosition(0.3);
    armState m_armState = armState.armPodium;

  }

  public void armHalfCourtPos() {
      // From the Motion Magic example
    m_armLeader.setPosition(0.25);
    armState m_armState = armState.armHalfCourt;

  }
  public armState returnArmstate() {
    
    return m_armState.getSetpoint();
    
  }


}