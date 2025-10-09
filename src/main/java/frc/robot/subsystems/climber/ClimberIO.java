package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    ClimberIOData data = new ClimberIOData(false, 0, 0, 0, 0, 0);
  }

  record ClimberIOData(
      boolean motorConnected,
      double positionRotations,
      double velocityRotationsPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double tempCelsius) {}

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void stopClimber() {}

  public default void runOpenLoop(double output) {}
}
