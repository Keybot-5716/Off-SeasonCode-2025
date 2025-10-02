package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public IntakeIOData data = new IntakeIOData(false, false, 0.0, 0.0, 0.0, 0.0);
  }

  record IntakeIOData(
      boolean motorConnected,
      boolean intaking,
      double velocityRPS,
      double RollerappliedVoltage,
      double RollercurrentAmps,
      double RollertemperatureCelsius) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerSpeed(double speed) {}

  public default void stopRoller() {}

  public default void runOpenLoop(double output) {}
}
