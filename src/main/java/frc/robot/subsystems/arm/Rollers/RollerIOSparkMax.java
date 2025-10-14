package frc.robot.subsystems.arm.Rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RollerIOSparkMax implements RollerIO {
  private SparkMax motor;
  private SparkMaxConfig config = new SparkMaxConfig();

  public RollerIOSparkMax() {
    motor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.data =
        new RollerIOData(
            true,
            motor.getAppliedOutput(),
            motor.getBusVoltage(),
            motor.getOutputCurrent(),
            motor.getMotorTemperature());
  }

  @Override
  public void setRollerSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void stopRoller() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.set(output);
  }
}
