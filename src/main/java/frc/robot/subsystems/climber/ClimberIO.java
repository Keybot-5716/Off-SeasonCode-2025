package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    ClimberIOData data = new ClimberIOData(false, 0.0, 0.0, 0.0, 0.0);
  }

  record ClimberIOData(
      boolean motorConnected,
      double velocityRPS,
      double ClimberappliedVoltage,
      double ClimbercurrentAmps,
      double ClimbertemperatureCelsius) {}

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setClimberSpeed(double speed) {}

  public default void stopClimber() {}

  public default void runOpenLoop(double output) {}
}
