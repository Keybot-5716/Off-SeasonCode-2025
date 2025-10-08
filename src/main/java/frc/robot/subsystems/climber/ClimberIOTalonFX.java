package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ClimberIOTalonFX() {
    motor = new TalonFX(ClimberConstants.CLIMBER_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 3;

    config.Feedback.SensorToMechanismRatio = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 160;
    config.MotionMagic.MotionMagicJerk = 1600;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = 3;
    config.Slot0.kD = 0.1;
    config.Slot0.kG = 0.2;
    config.Slot0.kS = 0.3;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.data =
        new ClimberIOData(
            BaseStatusSignal.isAllGood(
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getDeviceTemp()),
            motor.getPosition().getValueAsDouble(),
            motor.getVelocity().getValueAsDouble(),
            motor.getMotorVoltage().getValueAsDouble(),
            motor.getSupplyCurrent().getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void stopClimber() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.setControl(new DutyCycleOut(output));
  }
}
