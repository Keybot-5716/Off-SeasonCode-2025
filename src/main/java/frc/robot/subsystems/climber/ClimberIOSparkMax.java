package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.CLIMBERSPARKMAXID;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax climberSparkMax = new SparkMax(CLIMBERSPARKMAXID, MotorType.kBrushless);
  private final SparkMaxConfig climberSparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController pidController = climberSparkMax.getClosedLoopController();

  public ClimberIOSparkMax() {
    // Poner el límite
    climberSparkMaxConfig.smartCurrentLimit(40);

    climberSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0).i(0).d(0);

    climberSparkMaxConfig.idleMode(IdleMode.kBrake);

    climberSparkMaxConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(-2);

    climberSparkMax.configure(
        climberSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSparkMaxVoltage(double voltage) {
    climberSparkMax.set(voltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.sparkAppliedVolts = climberSparkMax.getAppliedOutput();
    inputs.sparkTemp = climberSparkMax.getMotorTemperature();
  }
}
