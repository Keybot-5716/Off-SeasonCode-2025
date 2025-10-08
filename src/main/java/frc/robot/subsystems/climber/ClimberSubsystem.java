package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert motorDisconnected =
      new Alert("Climber Motor Disconnected! D:", AlertType.kWarning);

  private double desiredOutput;

  private DesiredState desiredState = DesiredState.STOPPED;
  private DesiredState lastDesiredState = DesiredState.STOPPED;
  private SubsystemState subsystemState = SubsystemState.STOPPING;

  public enum DesiredState {
    STOPPED,
    RETRIEVED,
    CLIMBED
  }

  public enum SubsystemState {
    STOPPING,
    RETRIEVING,
    CLIMBING
  }

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    motorDisconnected.set(
        !motorConnectedDebouncer.calculate(inputs.data.motorConnected() && !Robot.isJITing()));

    Logger.recordOutput("Climber/IndividualState", lastDesiredState);
    Logger.recordOutput("Climber/CurrentState", subsystemState);

    lastDesiredState = this.desiredState;
    subsystemState = setStateTransitions();
    applyStates();
  }

  public void runOpenLoop(double output) {
    io.runOpenLoop(output);
  }

  public void stop() {
    io.stopClimber();
  }

  public SubsystemState setStateTransitions() {
    return switch (desiredState) {
      case STOPPED -> SubsystemState.STOPPING;
      case RETRIEVED -> SubsystemState.RETRIEVING;
      case CLIMBED -> SubsystemState.CLIMBING;
      default -> SubsystemState.STOPPING;
    };
  }

  public void applyStates() {
    switch (subsystemState) {
      case STOPPING:
        stop();
        break;
      case RETRIEVING:
        runOpenLoop(-desiredOutput);
        break;
      case CLIMBING:
        runOpenLoop(desiredOutput);
    }
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredState(DesiredState desiredState, double desiredOutput) {
    this.desiredState = desiredState;
    this.desiredOutput = desiredOutput;
  }
}
