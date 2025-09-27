package frc.robot.subsystems.arm.Rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.DesiredState;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private double desiredRollersPosition;
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
    SCORING,
    TAKING_CORAL,
    DEFAULT,
  }

  public void setRollerSpeed(double speed) {}
}
