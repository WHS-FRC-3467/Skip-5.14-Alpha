// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

/* Phoenix6 */
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/* WPI */
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/* Local */
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Subsystems.Arm.Setpoint.ArmState;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    /* Creates a new ArmSubsystem */
	
    private TalonFX m_armLeader = new TalonFX(CanConstants.ID_ArmLeader);
    private TalonFX m_armFollower = new TalonFX(CanConstants.ID_ArmFollower);

    private DutyCycleEncoder m_encoder = new DutyCycleEncoder(DIOConstants.armAbsEncoder);
    private DutyCycleOut m_PercentOutput = new DutyCycleOut(0.0);

    private TrapezoidProfile.Constraints armConstraints;

    private ProfiledPIDController m_controllerarmLeader;

    private ArmFeedforward feedforward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG, Constants.ArmConstants.kV, Constants.ArmConstants.kA); // double kS, double kG, double kV, double kA

    private Setpoint m_setpoint;
    private double armDegrees;
    
    public ArmSubsystem() {
        
        // Config Duty Cycle Range for the encoders
        m_encoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);

        // Default Motors
        var leadConfiguration = new TalonFXConfiguration();
        var followerConfiguration = new TalonFXConfiguration();

        // Set the mode to brake
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leadConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.NEUTRAL_DEADBAND;
        followerConfiguration.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.NEUTRAL_DEADBAND;

        //leadConfiguration.CurrentLimits.SupplyCurrentLimit();

        leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //leadConfiguration.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);

        //leadConfiguration.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);

        //leadConfiguration.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
        //leadConfiguration.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
        

        Timer.delay(1.5); //TODO: See if needs to be removed
        armConstraints = new TrapezoidProfile.Constraints(ArmConstants.ARM_CRUISE,
                                                          ArmConstants.ARM_ACCELERATION);

        m_controllerarmLeader = new ProfiledPIDController(0.02, 0, 0, armConstraints);
        m_armLeader.getConfigurator().apply(leadConfiguration);

        m_armFollower.getConfigurator().apply(followerConfiguration);
        m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), false));

        reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if(getArmJointDegrees() <= 0.0 || getArmJointDegrees() >= 360.0) {
            m_armLeader.stopMotor();
        }
        SmartDashboard.putData("Arm Contoller",m_controllerarmLeader);
        //SmartDashboard.putBoolean("Upper at Setpoint", getUpperAtSetpoint());
        SmartDashboard.putBoolean("Arm Joint at Setpoint", getArmJointAtSetpoint());
        //SmartDashboard.putNumber("Upper Angle", getUpperJointDegrees());
        SmartDashboard.putNumber("Arm Joint Angle", getArmJointDegrees());

        SmartDashboard.putNumber("Arm Encoder", getArmJointPos());

        if (Constants.RobotConstants.kIsTuningMode) {
            SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getArmJointPos()));
            SmartDashboard.putNumber("Arm Joint Error", getArmJointError());
            SmartDashboard.putNumber("Arm Joint Setpoint", armDegrees);
        } 
    }

    public void reset() {
        armDegrees = getArmJointDegrees();
        m_controllerarmLeader.reset(getArmJointDegrees());
        m_setpoint = new Setpoint(armDegrees, ArmState.OTHER);
        
    }

    public void updateArmSetpoint(double setpoint) {
        if (armDegrees != setpoint) {                   
            if (setpoint < 360 && setpoint > 0) {          
                armDegrees = setpoint;                      
            }
        }
    }

    public void runArmProfiled() {
        m_controllerarmLeader.setConstraints(armConstraints);
        //m_controllerarmLeader.setGoal(new TrapezoidProfile.State(armDegrees, 0.0));
        double pidOutput = -m_controllerarmLeader.calculate(getArmJointDegrees(), new TrapezoidProfile.State(armDegrees, 0.0));
        double ff = feedforward.calculate(pidOutput, 0);
        if(Constants.RobotConstants.kIsTuningMode){
            SmartDashboard.putNumber("Arm PID output", pidOutput);
        }
        System.out.println("Upper PID" + pidOutput);
        if(Math.abs(pidOutput) > 0.01 && Math.abs(pidOutput)<0.045){
            pidOutput = Math.copySign(0.045, pidOutput);
        }
        m_armLeader.setControl(m_PercentOutput.withOutput(pidOutput - ff));
    }

    public double getArmJointError(){
        return Math.abs(armDegrees - getArmJointDegrees());
    }

    public boolean getArmJointAtSetpoint() {
        return getArmJointError() < ArmConstants.TOLERANCE_POS;
    }

    public Setpoint getSetpoint() {
        if(m_setpoint.equals(null)){
            reset();
            return m_setpoint;
        } else {
            return m_setpoint;
        }
    }

    public void setPercentOutputUpper(double speed) {
        m_armLeader.setControl(m_PercentOutput.withOutput(speed));
    }

    public Command runArmMotorSpeedCommand(double speed) {
        return new RunCommand(() -> this.setPercentOutputUpper(speed),this);
    }

/*     public void neutralUpper() {
        m_armLeader.neutralOutput();
    } */

    public double getArmJointPos() {
        return m_encoder.getAbsolutePosition();
    }

    public double getArmJointDegrees() {
        return dutyCycleToDegrees(getArmJointPos()) + ArmConstants.ARM_OFFSET;
    }

    public double dutyCycleToDegrees(double dutyCyclePos) {
        return dutyCyclePos * 360;
    }


}
