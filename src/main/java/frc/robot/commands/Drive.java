package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends Command {
  private static final double DEADBAND = 0.1;

  private final SwerveSubsystem swerveSub;
  private final DoubleSupplier xAxis, yAxis, rAxis;
  private final BooleanSupplier slowMode;

  private double RED_ALLIANCE_MULTIPLIER = 1;

  public Drive(
      SwerveSubsystem swerveSub,
      DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rAxis,
      BooleanSupplier slowMode) {
    this.swerveSub = swerveSub;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rAxis = rAxis;
    this.slowMode = slowMode;

    addRequirements(swerveSub);
  }

  @Override
  public void initialize() {
    RED_ALLIANCE_MULTIPLIER =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? -1
            : 1;
  }

  @Override
  public void execute() {
    double xMagnitude = MathUtil.applyDeadband(xAxis.getAsDouble(), DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(yAxis.getAsDouble(), DEADBAND);
    double omega = MathUtil.applyDeadband(rAxis.getAsDouble(), DEADBAND);

    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(xAxis.getAsDouble(), yAxis.getAsDouble()), DEADBAND);
    Rotation2d linearDirection =
        new Rotation2d(Math.atan2(yAxis.getAsDouble(), xAxis.getAsDouble()));

    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    double SLOW_MODE_MULTIPLIER_TRANSLATIONAL = slowMode.getAsBoolean() ? 0.5 : 1;
    double SLOW_MODE_MULTIPLIER_ROTATIONAL = slowMode.getAsBoolean() ? 0.5 : 1;

    xMagnitude =
        Math.copySign(linearVelocity.getX() * linearVelocity.getX(), linearVelocity.getX());
    yMagnitude =
        Math.copySign(linearVelocity.getY() * linearVelocity.getY(), linearVelocity.getY());
    omega = Math.copySign(omega * omega, omega);

    LinearVelocity xVelocity =
        Units.MetersPerSecond.of(
            xMagnitude
                * swerveSub.getMaxLinearSpeedMetersPerSec()
                * SLOW_MODE_MULTIPLIER_TRANSLATIONAL
                * RED_ALLIANCE_MULTIPLIER);
    LinearVelocity yVelocity =
        Units.MetersPerSecond.of(
            yMagnitude
                * swerveSub.getMaxLinearSpeedMetersPerSec()
                * SLOW_MODE_MULTIPLIER_TRANSLATIONAL
                * RED_ALLIANCE_MULTIPLIER);
    AngularVelocity rVelocity =
        Units.RadiansPerSecond.of(
            omega * swerveSub.getMaxAngularSpeedRadPerSec() * SLOW_MODE_MULTIPLIER_ROTATIONAL);

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, rVelocity);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    swerveSub.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? swerveSub.getRotation().plus(new Rotation2d(Math.PI))
                : swerveSub.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerveSub.stop();
  }
}
