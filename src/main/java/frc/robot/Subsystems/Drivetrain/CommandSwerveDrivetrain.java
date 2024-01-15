package frc.robot.Subsystems.Drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Util.ModifiedSignalLogger;
import frc.robot.Util.SwerveVoltageRequest;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configNeutralMode(NeutralModeValue.Coast);
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configNeutralMode(NeutralModeValue.Coast);
        configurePathPlanner();
    }

    private void configurePathPlanner() {

        /*
         * Calculate drivebase radius (in meters). For swerve drive, this is the distance from the center of the robot to the furthest
         * module.
         */
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        /*
         * Configure the PathPlanner AutoBuilder
         */ 
        AutoBuilder.configureHolonomic(

            // Supplier of current robot pose
            () -> this.getState().Pose,

            // Consumer for seeding pose against auto
            this::seedFieldRelative,

            // A supplier for the robot's current robot relative chassis speeds
            this::getCurrentRobotChassisSpeeds,

            // A consumer for setting the robot's robot-relative chassis speeds
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),

            // Method for configuring the path following commands
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0), new PIDConstants(10, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
                    driveBaseRadius, new ReplanningConfig()),

            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field during auto only.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red & !DriverStation.isTeleop();
                } else
                    return false;
            },
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /*
     * SysID robot drive characterization routines
     */    
    private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))), null,
                    this));

    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))), null,
                    this));

    private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))), null,
                    this));

    public Command runDriveQuasiTest(Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public Command runDriveSlipTest() {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
}
