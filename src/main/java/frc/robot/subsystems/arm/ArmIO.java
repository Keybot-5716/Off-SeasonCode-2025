package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    ArmIOData data = new ArmIOData(false, 0, 0, 0, 0, 0);
  }

  record ArmIOData(
      boolean motorConnected,
      double positionRotations,
      double velocityRotationsPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double tempCelsius) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void stop() {}

  public default void runOpenLoop(double output) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double position) {}

  public default void setNeutralModeBreak(boolean enable) {}

  public default void resetEncoder() {}

  public default void setPID(double kP, double kI, double kD) {}

  /** Se utiliza para protocolos SysID */
  default void optimizeForSysID() {}
}
