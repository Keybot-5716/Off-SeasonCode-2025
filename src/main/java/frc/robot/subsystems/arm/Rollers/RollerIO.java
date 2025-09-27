package frc.robot.subsystems.arm.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  
  @AutoLog
  public static class RollerIOInputs {
    RollerIOData data = new RollerIOData(false, 0.0, 0.0, 0.0, 0.0);
  }

  record RollerIOData(
      boolean motorConnected,
      double velocityRPS,
      double RollerappliedVoltage,
      double RollercurrentAmps,
      double RollertemperatureCelsius) {}

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerSpeed(double speed) {}

  public default void stopRoller() {}

  public default void runOpenLoop(double output) {}
}
