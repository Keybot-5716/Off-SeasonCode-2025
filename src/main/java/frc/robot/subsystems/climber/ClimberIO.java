package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  // Contiene todo el input que recibe del hardware
  @AutoLog
  public static class ClimberIOInputs {
    public double sparkAppliedVolts = 0.0;
    public double sparkTemp = 0.0;
    public double pidOutput = 0.0;
  }

  // Updatea los inputs
  public default void updateInputs(ClimberIOInputs inputs) {}

  // Pone el voltaje
  public default void setSparkMaxVoltage(double voltage) {}
}
