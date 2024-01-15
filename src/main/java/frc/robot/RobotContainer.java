// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

    /*
     * Shuffleboard Chooser widgets
     */
    private SendableChooser<Command> autoChooser;
    private SendableChooser<String> controlChooser = new SendableChooser<>();
    private SendableChooser<Double> speedChooser = new SendableChooser<>();

    /*
     * Speed adjustments
     */
    // Initial max is true top speed
    private double m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    // Reduction in speed from Max Speed, 0.1 = 10%
    private final double m_TurtleSpeed = 0.1;
    // .75 rotation per second max angular velocity. Adjust for max turning rate speed.
    private final double m_MaxAngularRate = Math.PI * 1.5;
    // .75 rotation per second max angular velocity. Adjust for max turning rate speed.
    private final double m_TurtleAngularRate = Math.PI * 0.5;
    // This will be updated when turtle and reset to MaxAngularRate
    private double m_AngularRate = m_MaxAngularRate;
    // Save last Speed Limit so we know if it needs updating
    private Double m_lastSpeed = 0.65;

    /*
     * Driver/Operator controllers
     */
    CommandXboxPS5Controller m_driverCtrl = new CommandXboxPS5Controller(0);
    CommandXboxPS5Controller m_operatorCtrl = new CommandXboxPS5Controller(1);

    // Drive Control style settings
    private Supplier<SwerveRequest> m_controlStyle;
    private String m_lastControl = "2 Joysticks";

    /*
     * Swerve Drive Configuration
     */
    // Tuner Constants is a static class that defines the drivetrain constants
    // It is configured by the Phoenix Tuner X Swerve Project Generator
    CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

    // Field-centric driving in Open Loop, can change to closed loop after characterization
    SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(m_MaxSpeed * 0.1).withRotationalDeadband(m_AngularRate * 0.1);

    // Field-centric driving in Closed Loop. Comment above and uncomment below.
    // SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
    // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(AngularRate * 0.1);

    // Swerve Drive functional requests
    SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();

    // Set up Drivetrain Telemetry
    Telemetry m_logger = new Telemetry(m_MaxSpeed);
    // Pose2d m_odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

    // Instantiate other Subsystems
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    // Setup Limelight periodic query (defaults to disabled)
    Limelight m_vision = new Limelight(m_drivetrain);

    public RobotContainer() {

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Change this to specify Limelight is in use
        m_vision.useLimelight(false);

        // Register NamedCommands for use in PathPlanner autos
        registerNamedCommands();
        
        // Configure Shuffleboard Chooser widgets
        configureChooserBindings();

        // Configure Driver and Operator controller buttons
        configureButtonBindings();

        // Configure button bindings for SysID robot profiling commands
        configureSysIDProfiling();

        // Set up the Telemetry function
        m_drivetrain.registerTelemetry(m_logger::telemeterize);

        // If running in Simulation, initialize the current pose
        if (Utils.isSimulation()) {
            m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        }

    }

    private void registerNamedCommands() {
        
        // Register Named Commands for use in PathPlanner autos
        NamedCommands.registerCommand("Deploy Intake", m_intakeSubsystem.deployIntakeCommand());
        NamedCommands.registerCommand("Retract Intake", m_intakeSubsystem.retractIntakeCommand());
        //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
        //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    }
    
    
    private void configureChooserBindings() {

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Build a drive control style chooser
        controlChooser.setDefaultOption("2 Joysticks", "2 Joysticks");
        controlChooser.addOption("1 Joystick Rotation Triggers", "1 Joystick Rotation Triggers");
        controlChooser.addOption("Split Joysticks Rotation Triggers", "Split Joysticks Rotation Triggers");
        controlChooser.addOption("2 Joysticks with Gas Pedal", "2 Joysticks with Gas Pedal");
        SmartDashboard.putData("Control Style Chooser", controlChooser);

        // Configure a Trigger to change the Control Style when a selection is made on the Control Style Chooser
        Trigger controlPick = new Trigger(() -> m_lastControl != controlChooser.getSelected());
        controlPick.onTrue(runOnce(() -> newControlStyle()));

        // Set the initial Drive Control Style
        newControlStyle();

        // Build a speed limit chooser
        speedChooser.addOption("100%", 1.0);
        speedChooser.addOption("95%", 0.95);
        speedChooser.addOption("90%", 0.9);
        speedChooser.addOption("85%", 0.85);
        speedChooser.addOption("80%", 0.8);
        speedChooser.addOption("75%", 0.75);
        speedChooser.addOption("70%", 0.7);
        speedChooser.setDefaultOption("65%", 0.65);
        speedChooser.addOption("60%", 0.6);
        speedChooser.addOption("55%", 0.55);
        speedChooser.addOption("50%", 0.5);
        speedChooser.addOption("35%", 0.35);
        SmartDashboard.putData("Speed Limit", speedChooser);

        // Configure a Trigger to change the speed limit when a selection is made on the Speed Limit Chooser
        Trigger speedPick = new Trigger(() -> m_lastSpeed != speedChooser.getSelected());
        speedPick.onTrue(runOnce(() -> newSpeed()));

        // Set the initial Speed Limit
        newSpeed();

    }

    private void configureButtonBindings() {

        // Driver: While A button is held, put swerve modules in Brake mode (modules make an 'X')
        m_driverCtrl.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

        // Driver: While B button is held, point drivetrain in the direction of the Left Joystick
        m_driverCtrl.b().whileTrue(m_drivetrain.applyRequest(
                () -> m_point.withModuleDirection(new Rotation2d(-m_driverCtrl.getLeftY(), -m_driverCtrl.getLeftX()))
            )
        );

        // Driver: On Start button press, reset the field-centric heading
        m_driverCtrl.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

        // Driver: While Left Bumper is held, drive in Turtle Mode
        m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_TurtleSpeed)
                .andThen(() -> m_AngularRate = m_TurtleAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * speedChooser.getSelected())
                .andThen(() -> m_AngularRate = m_MaxAngularRate));

        // Driver: Use Left and Right Triggers to run Intake at variable speed (left = in, right = out)
        m_driverCtrl.leftTrigger(0.1).onTrue(m_intakeSubsystem.runIntakeCommand(m_driverCtrl.getLeftTriggerAxis()));   
        m_driverCtrl.rightTrigger(0.1).onTrue(m_intakeSubsystem.runIntakeCommand((-1.0) * m_driverCtrl.getRightTriggerAxis()));   

        // Driver: Use Right Bumper to toggle intake position
        m_driverCtrl.rightBumper().onTrue(m_intakeSubsystem.toggleIntakeCommand());


        /*
         * Put Commands on Shuffleboard
         */
        SmartDashboard.putData("Deploy Intake", m_intakeSubsystem.deployIntakeCommand());
        SmartDashboard.putData("Retract Intake", m_intakeSubsystem.retractIntakeCommand());
        SmartDashboard.putData("Toggle Intake", m_intakeSubsystem.toggleIntakeCommand());
        SmartDashboard.putData("Update Shooter Gains", m_shooterSubsystem.updateShooterGainsCommand());
        SmartDashboard.putData("Run Shooter", m_shooterSubsystem.runShooterCommand());

    }

    private void configureSysIDProfiling() {

        /*
         *  These bindings will only be used when characterizing the Drivetrain. They can eventually be commented out.
         */
        m_driverCtrl.x().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveQuasiTest(Direction.kForward));
        m_driverCtrl.x().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runDriveQuasiTest(Direction.kReverse));

        m_driverCtrl.y().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveDynamTest(Direction.kForward));
        m_driverCtrl.y().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runDriveDynamTest(Direction.kReverse));

        m_driverCtrl.a().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runSteerQuasiTest(Direction.kForward));
        m_driverCtrl.a().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runSteerQuasiTest(Direction.kReverse));

        m_driverCtrl.b().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runSteerDynamTest(Direction.kForward));
        m_driverCtrl.b().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runSteerDynamTest(Direction.kReverse));

        // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon wheel slip
        m_driverCtrl.back().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveSlipTest());
    }

    public Command getAutonomousCommand() {

        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    private void newControlStyle() {

        m_lastControl = controlChooser.getSelected();
        switch (controlChooser.getSelected()) {
        case "2 Joysticks":
            m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed) // Drive forward -Y
                    .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverCtrl.getRightX() * m_AngularRate); // Drive counterclockwise with negative X (left)
            break;
        case "1 Joystick Rotation Triggers":
            m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed) // Drive forward -Y
                    .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate((m_driverCtrl.getLeftTriggerAxis() - m_driverCtrl.getRightTriggerAxis()) * m_AngularRate);
            // Left trigger turns left, right trigger turns right
            break;
        case "Split Joysticks Rotation Triggers":
            m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed) // Left stick forward/back
                    .withVelocityY(-m_driverCtrl.getRightX() * m_MaxSpeed) // Right stick strafe
                    .withRotationalRate((m_driverCtrl.getLeftTriggerAxis() - m_driverCtrl.getRightTriggerAxis()) * m_AngularRate);
            // Left trigger turns left, right trigger turns right
            break;
        case "2 Joysticks with Gas Pedal":
            m_controlStyle = () -> {
                var stickX = -m_driverCtrl.getLeftX();
                var stickY = -m_driverCtrl.getLeftY();
                var angle = Math.atan2(stickX, stickY);
                // RightTriggerAxis = "Gas pedal"
                return m_drive.withVelocityX(Math.cos(angle) * m_driverCtrl.getRightTriggerAxis() * m_MaxSpeed) // left x * gas
                        .withVelocityY(Math.sin(angle) * m_driverCtrl.getRightTriggerAxis() * m_MaxSpeed) // Angle of left stick Y * gas
                        .withRotationalRate(-m_driverCtrl.getRightX() * m_AngularRate); // Drive counterclockwise with negative X (left)
            };
            break;
        }
        try {
            m_drivetrain.getDefaultCommand().cancel();
        } catch (Exception e) {
        }

        // Specify the desired Control Style as the Drivetrain's default command
        // Drivetrain will execute this command periodically
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(m_controlStyle).ignoringDisable(true));
    }

    private void newSpeed() {

        m_lastSpeed = speedChooser.getSelected();
        m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_lastSpeed;
    }
}
