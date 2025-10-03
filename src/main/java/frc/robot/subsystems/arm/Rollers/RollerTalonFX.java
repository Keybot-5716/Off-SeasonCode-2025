package frc.robot.subsystems.arm.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RollerTalonFX implements RollerIO {
  private TalonFX RollerMotor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltage = new VoltageOut(0);
  MotionMagicExpoVoltage positionVoltage = new MotionMagicExpoVoltage(0).withSlot(0);

  public RollerTalonFX() {
    RollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Ponemos la relación del sensor suponiendo que mide 2048 unidades por RPM
    config.Feedback.SensorToMechanismRatio = 5;
    // Ajustamos los valores de PID
    config.Slot0.kP = 30;
    config.Slot0.kD = 0.1;
    config.Slot0.kS = 1.0;

    RollerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.data =
        new RollerIOData(
            BaseStatusSignal.isAllGood(
                RollerMotor.getPosition(),
                RollerMotor.getVelocity(),
                RollerMotor.getMotorVoltage(),
                RollerMotor.getSupplyCurrent(),
                RollerMotor.getDeviceTemp()),
            RollerMotor.getVelocity().getValueAsDouble(),
            RollerMotor.getMotorVoltage().getValueAsDouble(),
            RollerMotor.getSupplyCurrent().getValueAsDouble(),
            RollerMotor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void setRollerSpeed(double speed) {
    RollerMotor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void stopRoller() {
    RollerMotor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    RollerMotor.setControl(new DutyCycleOut(output));
  }

  /*
  public void moveContinuously(double output) {
      RollerMotor.setControl(voltage.withOutput(output));
      checkForStall();
  }

  private void checkForStall() {
      double current = RollerMotor.getSupplyCurrent().getValueAsDouble();
      double velocity = RollerMotor.getVelocity().getValueAsDouble();

      if (current > STALL_CURRENT_THRESHOLD && Math.abs(velocity) < MIN_VELOCITY_THRESHOLD) {

          RollerMotor.stopMotor();
          setNeutralMode(true);
      }
  }*/
}
