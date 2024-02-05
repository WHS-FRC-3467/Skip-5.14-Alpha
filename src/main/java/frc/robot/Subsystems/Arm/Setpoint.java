package frc.robot.Subsystems.Arm;

public class Setpoint {
    public double arm;
    public ArmState state;

    public Setpoint(double arm, ArmState state) {
        this.arm = arm;
        this.state = state;
    }

    public enum ArmState {
        HOME, AMP, CLIMB, OTHER
    }
}
