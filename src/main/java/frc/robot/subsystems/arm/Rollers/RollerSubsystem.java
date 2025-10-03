package frc.robot.subsystems.arm.Rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private double desiredSpeedRollers;
  private double desiredOutput;

  private DesiredState desiredState = DesiredState.DEFAULT;
  private SubsystemState subsystemState = SubsystemState.DEFAULT;

  public enum DesiredState {
    REVERSE,
    FORWARD,
    DEFAULT
  }

  public enum SubsystemState {
    REVERSING,
    FORWARDING,
    DEFAULT,
  }

  public RollerSubsystem(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);

    Logger.recordOutput("Rollers/Current Spped", desiredSpeedRollers);
    Logger.recordOutput("Rollers/Current State", subsystemState);
    Logger.recordOutput("Rollers/Desired State", desiredState);

    subsystemState = transitions();
    applyStates();
  }

  public void setRollerSpeed(double speed) {
    MathUtil.clamp(speed, 0, 1);
    io.setRollerSpeed(speed);
    desiredSpeedRollers = speed;
  }

  public void stop() {
    io.stopRoller();
  }

  public SubsystemState transitions() {
    return switch (desiredState) {
      case DEFAULT -> SubsystemState.DEFAULT;
      case REVERSE -> SubsystemState.REVERSING;
      case FORWARD -> SubsystemState.FORWARDING;
      default -> SubsystemState.DEFAULT;
    };
  }

  public void applyStates() {
    switch (subsystemState) {
      case DEFAULT:
        stop();
        break;
      case FORWARDING:
        setRollerSpeed(desiredOutput);
        break;
      case REVERSING:
        setRollerSpeed(-desiredOutput);
        break;
    }
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredState(DesiredState desiredState, double speedRollers) {
    this.desiredState = desiredState;
    this.desiredSpeedRollers = speedRollers;
  }
}
