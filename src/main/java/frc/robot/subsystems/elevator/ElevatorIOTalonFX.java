package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageOutRequest = new VoltageOut(0);
  private final MotionMagicExpoVoltage motionMagicEVRequest = new MotionMagicExpoVoltage(0);
  private final DutyCycleOut openloop = new DutyCycleOut(0);

  public ElevatorIOTalonFX() {
    motor = new TalonFX(ElevatorConstants.ELEVATOR_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.FORWARD_THRESHOLD;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.REVERSE_THRESHOLD;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEARBOX_REDUCTION;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 160;
    config.MotionMagic.MotionMagicJerk = 1600;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = 3;
    config.Slot0.kG = 0.5;
    config.Slot0.kS = 0.3;

    motor.setPosition(0);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.data =
        new ElevatorIOData(
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
    motor.setControl(openloop.withOutput(output));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageOutRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(new PositionVoltage(position));
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
