package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageOutRequest = new VoltageOut(0);
  private final MotionMagicExpoVoltage motionMagicEVRequest = new MotionMagicExpoVoltage(0);

  public ArmIOTalonFX() {
    motor = new TalonFX(ArmConstants.ARM_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.Feedback.SensorToMechanismRatio = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.MotionMagic.MotionMagicCruiseVelocity = 0; // Valor original: 80
    config.MotionMagic.MotionMagicAcceleration = 0; // Valor original: 160
    config.MotionMagic.MotionMagicExpo_kA = 0.005;
    config.MotionMagic.MotionMagicExpo_kV = 0.01;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = 1.5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.Slot0.kG = 0.2;
    config.Slot0.kS = 0.3;
    config.Slot0.kV = 0.001;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.data =
        new ArmIOData(
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
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.setControl(new DutyCycleOut(output));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageOutRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(motionMagicEVRequest.withPosition(position));
  }

  @Override
  public void setNeutralModeBreak(boolean enable) {
    config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    motor.getConfigurator().apply(config);
  }
}
