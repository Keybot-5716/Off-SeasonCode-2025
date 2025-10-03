package frc.robot.subsystems.arm.Rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;

public class RollerTalonFX implements RollerIO {
  private TalonFX RollerMotor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltage = new VoltageOut(0);
  MotionMagicExpoVoltage positionVoltage = new MotionMagicExpoVoltage(0).withSlot(0);

  public RollerTalonFX() {

    // Aplicamos toda la configuración en un valor estático
    // Modo neutral del motor cuando se desabilita (el brake es un freno)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Inversión del motor(CW- o sentido del reloj)
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Habilitamos un límite hacia enfrente y ponemos ese límite
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.Rotations.of(0.28).in(Units.Rotations);
    // Habilitamos un límite hacia atrás y ponemos ese límite
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.Rotations.of(-0.01).in(Units.Rotations);
    // Ponemos la relación del sensor suponiendo que mide 2048 unidades por RPM
    config.Feedback.SensorToMechanismRatio = 64;
    // Ajustamos los valores de PID
    config.Slot0.kP = 30;
    config.Slot0.kD = 0.1;
    config.Slot0.kS = 1.0;

    // Configuración con Motion Magic
    var motionMagicConfigs = config.MotionMagic;

    // Ponemos el máximo de rotaciones que va a poder dar por segundo
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;

    // Ponemos el control de la aceleración o desaceleración
    motionMagicConfigs.MotionMagicAcceleration = 160;

    // Ponemos el control del cambio de la aceleración
    motionMagicConfigs.MotionMagicJerk = 1600;

    RollerMotor.setPosition(0);
    RollerMotor.getConfigurator().apply(config);
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