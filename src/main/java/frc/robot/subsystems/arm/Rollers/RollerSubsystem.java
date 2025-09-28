package frc.robot.subsystems.arm.Rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.DesiredState;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private Angle desiredSpeedRollers;
  private double desiredOutput;

  private DesiredState desiredState = DesiredState.DEFAULT;
  private DesiredState lastdesiredState = DesiredState.DEFAULT;
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

  /*
   * (Se debe poner esto?)
   * @Override
  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
  }
  */

  public void setRollerSpeed(double speed) {
    MathUtil.clamp(speed, 0, 1);
    io.setRollerSpeed(speed);
    // dudas
    desiredSpeedRollers = Units.Rotations.of(speed);
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
        io.stopRoller();
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
    this.desiredSpeedRollers = desiredSpeedRollers;
  }
}
