package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    ElevatorIOData data = new ElevatorIOData(false, 0, 0, 0, 0, 0);
  }

  record ElevatorIOData(
      boolean motorConnected,
      double positionRotations,
      double velocityRotationsPS,
      double appliedVolts,
      double supplyCurrentAmps,
      double tempCelsius) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void stop() {}

  public default void runOpenLoop(double output) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double position) {}

  public default void setPositionV(double position) {}

  public default void setNeutralModeBreak(boolean enable) {}

  public default void resetEncoder() {}

  public default void setPID(double kP, double kI, double kD) {}

  /** Se utiliza para protocolos SysID */
  default void optimizeForSysID() {}
}
