package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert motorDisconnected =
      new Alert("Arm Motor Disconnected! D:", AlertType.kWarning);

  private double lastDesiredAngle = 0;
  private double desiredElevatorPosition;
  private double desiredOutput;

  private DesiredState desiredState = DesiredState.STOPPED;
  private DesiredState lastDesiredState = DesiredState.STOPPED;
  private SubsystemState subsystemState = SubsystemState.STOPPING;

  public enum DesiredState {
    STOPPED,
    HOME,
    PREP_LVL,
    MANUAL
  }

  public enum SubsystemState {
    STOPPING,
    HOMING,
    PREPARING_LVL,
    MANUAL
  }

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    motorDisconnected.set(
        !motorConnectedDebouncer.calculate(inputs.data.motorConnected() && !Robot.isJITing()));

    Logger.recordOutput("Arm/Current Position", getArmPosInRotations());
    Logger.recordOutput("Arm/Desired Position", lastDesiredAngle);
    Logger.recordOutput("Arm/IsAtDesiredPosition", isAtDesiredPos());
    Logger.recordOutput("Arm/IndividualState", lastDesiredState);
    Logger.recordOutput("Arm/CurrentState", subsystemState);

    lastDesiredState = this.desiredState;
    subsystemState = setStateTransitions();
    applyStates();
  }

  public void setPosition(double position) {
    io.setPosition(position);
    lastDesiredAngle = position;
  }

  public void runOpenLoop(double output) {
    io.runOpenLoop(output);
  }

  public void setVoltage(Voltage voltage) {
    io.setVoltage(voltage.in(Units.Volts));
  }

  public double getArmPosInRotations() {
    return inputs.data.positionRotations();
  }

  public double getLastDesiredArmPosInRotations() {
    return lastDesiredAngle;
  }

  public void resetEncoder() {
    io.setPosition(0);
  }

  public boolean isPositioned(double position, double offset) {
    return MathUtil.isNear(position, getArmPosInRotations(), offset);
  }

  public boolean isPositionedRotations(double position, double offset) {
    return MathUtil.isNear(position, getArmPosInRotations(), offset);
  }

  public boolean isAtDesiredPos() {
    return isPositionedRotations(getLastDesiredArmPosInRotations(), ArmConstants.MIN_OFFSET);
  }

  public void stop() {
    io.stop();
  }

  public SubsystemState setStateTransitions() {
    return switch (desiredState) {
      case HOME -> SubsystemState.HOMING;
      case STOPPED -> SubsystemState.STOPPING;
      case PREP_LVL -> SubsystemState.PREPARING_LVL;
      case MANUAL -> SubsystemState.MANUAL;
      default -> SubsystemState.STOPPING;
    };
  }

  public void applyStates() {
    switch (subsystemState) {
      case HOMING:
        setPosition(ArmConstants.NONE);
        ;
        break;
      case STOPPING:
        stop();
        break;
      case PREPARING_LVL:
        setPosition(desiredElevatorPosition);
        break;
      case MANUAL:
        runOpenLoop(desiredOutput);
        break;
    }
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredState(DesiredState desiredState, double desiredElevatorPosition) {
    this.desiredState = desiredState;
    this.desiredElevatorPosition = desiredElevatorPosition;
  }

  public void setDesiredStateWithOutput(DesiredState desiredState, double desiredOutput) {
    this.desiredState = desiredState;
    this.desiredOutput = desiredOutput;
  }
}
