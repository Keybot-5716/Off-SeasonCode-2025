package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorIOSim implements ElevatorIO {

  private static final double carriageMass = 10.0;
  private static final double stagesMass = 5.0;
  private static final double drumRadiusMeters = 0.0254;
  private static final double travelLimitMeters = 1.35;
  private static final double g = 9.81;
  private static final double loopPeriodSec = 0.02;

  private static final DCMotor motorGB =
      DCMotor.getKrakenX60(1).withReduction(ElevatorConstants.GEARBOX_REDUCTION);

  private double position = 0.0;
  private double velocityMPS = 0.0;
  private double appliedVolts = 0.0;
  private double supplyCurrent = 0.0;
  private double tempCelsius = 25;

  private boolean closedLoop = false;
  private double setpoint = 0.0;
  private double kP = 7;
  private double kI, kD = 0;
  private double integral = 0.0;
  private double prevError = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double totalMass = carriageMass + stagesMass;

    if (closedLoop) {
      double error = setpoint - position;
      integral += error * loopPeriodSec;
      double derivative = (error - prevError) / loopPeriodSec;
      prevError = error;

      double output = kP * error + kI * integral + kD * derivative;
      appliedVolts = MathUtil.clamp(output, -12.0, 12.0);
    }

    double motorSpeedRPS = velocityMPS / drumRadiusMeters;
    double motorTorqueNm = motorGB.KtNMPerAmp * motorGB.getCurrent(motorSpeedRPS, appliedVolts);

    double forceN = motorTorqueNm / drumRadiusMeters;

    double gravityForce = totalMass * g;
    double netForce = forceN - gravityForce;

    double accelMps2 = (totalMass > 0) ? netForce / totalMass : 0;

    velocityMPS += accelMps2 * loopPeriodSec;
    position += velocityMPS * loopPeriodSec;

    if (position < 0) {
      position = 0;
      velocityMPS = 0;
    }
    if (position > travelLimitMeters) {
      position = travelLimitMeters;
      velocityMPS = 0;
    }

    supplyCurrent = motorGB.getCurrent(motorSpeedRPS, appliedVolts);

    inputs.data =
        new ElevatorIOData(
            true,
            position / (2 * Math.PI * drumRadiusMeters),
            velocityMPS / (2 * Math.PI * drumRadiusMeters),
            appliedVolts,
            supplyCurrent,
            tempCelsius);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    velocityMPS = 0.0;
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(output * 12, -12, 12);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12, 12);
  }

  @Override
  public void setPosition(double position) {
    closedLoop = true;
    setpoint = position * (2 * Math.PI * drumRadiusMeters);
  }

  @Override
  public void setNeutralModeBreak(boolean x) {}

  @Override
  public void resetEncoder() {
    position = 0;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }
}
