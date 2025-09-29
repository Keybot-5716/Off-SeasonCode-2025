package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeTalonFX implements IntakeIO {

  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageOutRequest = new VoltageOut(0);
  private final MotionMagicExpoVoltage motionMagicEVRequest = new MotionMagicExpoVoltage(0);

  public IntakeTalonFX() {
    motor = new TalonFX(IntakeConstants.INTAKE_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.Feedback.SensorToMechanismRatio = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 160;
    config.MotionMagic.MotionMagicJerk = 1600;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = 5;
    config.Slot0.kG = 0.5;
    config.Slot0.kS = 0.3;

    motor.setPosition(0);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.data =
        new IntakeIOData(
            BaseStatusSignal.isAllGood(
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getDeviceTemp()),
            false,
            motor.getVelocity().getValueAsDouble(),
            motor.getMotorVoltage().getValueAsDouble(),
            motor.getSupplyCurrent().getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble());
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.setControl(motionMagicEVRequest.withFeedForward(output));
  }

  public void setVoltage(double volts) {
    motor.setControl(voltageOutRequest.withOutput(volts));
  }

  public void setPosition(double position) {
    motor.setControl(motionMagicEVRequest.withPosition(position));
  }

  public void setNeutralModeBreak(boolean enable) {
    config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  public void resetEncoder() {
    motor.setPosition(0);
  }

  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    motor.getConfigurator().apply(config);
  }
}
