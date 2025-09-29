package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsys extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert motorDisconnected =
      new Alert("Intake Motor Disconnected! D:", AlertType.kWarning);

  private Angle lastDesiredAngle = Units.Rotations.of(0);
  private double desiredElevatorPosition;
  private double desiredOutput;

  private DesiredState desiredState = DesiredState.STOPPED;
  private DesiredState lastDesiredState = DesiredState.STOPPED;
  private SubsystemState subsystemState = SubsystemState.STOPPING;

  public enum DesiredState {
    STOPPED,
    HOME,
    DEPLOYED,
    MANUAL
  }

  public enum SubsystemState {
    STOPPING,
    HOMING,
    DEPLOYING,
    MANUAL
  }

  public IntakeSubsys(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    motorDisconnected.set(
        !motorConnectedDebouncer.calculate(inputs.data.motorConnected() && !Robot.isJITing()));

    // Logger.recordOutput("Intake/Current Position", getElevatorPosInRotations());
    Logger.recordOutput("Intake/Desired Position", lastDesiredAngle.in(Units.Rotations));
    // Logger.recordOutput("Intake/IsAtDesiredPosition", isAtDesiredPos());
    Logger.recordOutput("Intake/ElevatorIndividualState", lastDesiredState);

    lastDesiredState = this.desiredState;
  }

  public void updateInputs(IntakeIOInputs inputs) {}

  public void setRollerSpeed(double speed) {}

  public void stopRoller() {}

  public void runOpenLoop(double output) {}

  public SubsystemState setStateTransitions() {
    return switch (desiredState) {
      case HOME -> SubsystemState.HOMING;
      case STOPPED -> SubsystemState.STOPPING;
      case DEPLOYED -> SubsystemState.DEPLOYING;
      case MANUAL -> SubsystemState.MANUAL;
      default -> SubsystemState.STOPPING;
    };
  }

  public void applyStates() {
    switch (subsystemState) {
      case HOMING:
        // io.setPosition(IntakeConstants.NONE.in(Units.Rotations));
        break;
      case STOPPING:
        // io.stop();
        break;
      case MANUAL:
        io.runOpenLoop(desiredOutput);
        break;
    }
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }
  /*
    public void setDesiredState(DesiredState desiredState, double desiredIntakePosition) {
      this.desiredState = desiredState;
      this.desiredIntakePosition = desiredIntakePosition;
    }
  */
  public void setDesiredStateWithOutput(DesiredState desiredState, double desiredOutput) {
    this.desiredState = desiredState;
    this.desiredOutput = desiredOutput;
  }
}
