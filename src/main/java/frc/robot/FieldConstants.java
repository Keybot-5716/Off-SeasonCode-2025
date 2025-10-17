package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.function.Supplier;

public class FieldConstants {
  public static final Distance FIELD_LENGHT = Units.Meters.of(17.548);
  public static final Distance FIELD_WIDTH = Units.Meters.of(8.052);

  public static boolean isRedAlliance =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

  // AUTO_ALLIGNING POSES
  public static final Pose2d NONE = new Pose2d(0, 0, new Rotation2d());
  public static final Pose2d NONE1 = new Pose2d(3.169, 4.015, new Rotation2d());

  public static final Pose2d REEF_1A = new Pose2d(3.2, 4.19, Rotation2d.fromDegrees(0));
  public static final Pose2d REEF_1B = new Pose2d(3.2, 3.85, Rotation2d.fromDegrees(0));

  public static final Pose2d REEF_2A = new Pose2d(3.695, 3, Rotation2d.fromDegrees(60));
  public static final Pose2d REEF_2B = new Pose2d(3.978, 2.816, Rotation2d.fromDegrees(60));

  public static final Pose2d REEF_3A = new Pose2d(5.018, 2.806, Rotation2d.fromDegrees(120)); //
  public static final Pose2d REEF_3B = new Pose2d(5.291, 2.969, Rotation2d.fromDegrees(120)); //

  public static final Pose2d REEF_4A = new Pose2d(5.8, 4.19, Rotation2d.fromDegrees(180)); //
  public static final Pose2d REEF_4B = new Pose2d(5.8, 3.85, Rotation2d.fromDegrees(180)); //

  public static final Pose2d REEF_5A = new Pose2d(5.291, 5.081, Rotation2d.fromDegrees(-120)); //
  public static final Pose2d REEF_5B = new Pose2d(5.022, 5.241, Rotation2d.fromDegrees(-120)); //

  public static final Pose2d REEF_6A = new Pose2d(3.991, 5.241, Rotation2d.fromDegrees(-60));
  public static final Pose2d REEF_6B = new Pose2d(3.691, 5.079, Rotation2d.fromDegrees(-60));

  public static final Pose2d LEFT_FEEDER = new Pose2d(0, 8, Rotation2d.fromDegrees(-55));
  public static final Pose2d RIGHT_FEEDER = new Pose2d(0, 0, Rotation2d.fromDegrees(55));

  // AUTO_ALLIGNING POSES RETRIEVE (AFTER SCORE)
  public static final Pose2d REEF_1A_BACK = new Pose2d();
  public static final Pose2d REEF_1B_BACK = new Pose2d();

  public static final Pose2d REEF_2A_BACK = new Pose2d();
  public static final Pose2d REEF_2B_BACK = new Pose2d();

  public static final Pose2d REEF_3A_BACK = new Pose2d();
  public static final Pose2d REEF_3B_BACK = new Pose2d();

  public static final Pose2d REEF_4A_BACK = new Pose2d();
  public static final Pose2d REEF_4B_BACK = new Pose2d();

  public static final Pose2d REEF_5A_BACK = new Pose2d();
  public static final Pose2d REEF_5B_BACK = new Pose2d();

  public static final Pose2d REEF_6A_BACK = new Pose2d();
  public static final Pose2d REEF_6B_BACK = new Pose2d();

  private static final List<Pose2d> BLUE_REEF =
      List.of(
          REEF_1A, REEF_1B, REEF_2A, REEF_2B, REEF_3A, REEF_3B, REEF_4A, REEF_4B, REEF_5A, REEF_5B,
          REEF_6A, REEF_6B);
  private static final List<Pose2d> BLUE_FEEDER = List.of(LEFT_FEEDER, RIGHT_FEEDER);
  private static final Pose2d[] BLUE_POSES =
      new Pose2d[] {
        NONE,
        NONE1,
        REEF_1A,
        REEF_1B,
        REEF_2A,
        REEF_2B,
        REEF_3A,
        REEF_3B,
        REEF_4A,
        REEF_4B,
        REEF_5A,
        REEF_5B,
        REEF_6A,
        REEF_6B,
        LEFT_FEEDER,
        RIGHT_FEEDER
      };

  private static final List<Pose2d> RED_REEF = getRedReef();
  private static final List<Pose2d> RED_FEEDER = getRedFeeder();
  private static final Pose2d[] RED_POSES = getRedPoses();

  // En esta función regresamos una pose posibles del equipo azul convertida al rojo
  public static Pose2d getRedPose(Pose2d blue) {
    return new Pose2d(
        // Restamos la x/y y sumamos 180 grados para invertir
        FieldConstants.FIELD_LENGHT.in(Units.Meters) - blue.getX(),
        FieldConstants.FIELD_WIDTH.in(Units.Meters) - blue.getY(),
        blue.getRotation().plus(Rotation2d.fromDegrees(180)));
  }
  // Conseguimos las poses convertidas al equipo rojo
  private static Pose2d[] getRedPoses() {
    Pose2d[] poses = new Pose2d[BLUE_POSES.length];

    for (int num = 0; num < BLUE_POSES.length; num++) {
      poses[num] = getRedPose(BLUE_POSES[num]);
    }

    return poses;
  }
  // Conseguimos las poses del reef para el equipo rojo
  private static List<Pose2d> getRedReef() {
    Pose2d[] poses = new Pose2d[BLUE_REEF.size()];

    for (int num = 0; num < BLUE_REEF.size(); num++) {
      poses[num] = getRedPose(BLUE_REEF.get(num));
    }

    return List.of(
        poses[0], poses[1], poses[2], poses[3], poses[4], poses[5], poses[6], poses[7], poses[8],
        poses[9], poses[10], poses[11]);
  }
  // Conseguimos las poses del feeder para equipo rojo
  private static List<Pose2d> getRedFeeder() {
    Pose2d[] poses = new Pose2d[BLUE_FEEDER.size()];
    poses[0] = getRedPose(LEFT_FEEDER);
    poses[1] = getRedPose(RIGHT_FEEDER);

    return List.of(poses[0], poses[1]);
  }
  // Conseguimos las posiciones del campo, ya sean del equipo azul o rojo
  public static Supplier<Pose2d[]> getFieldPos() {
    // if(FieldConstants.ALLIANCE.isPresent() &&
    // FieldConstants.ALLIANCE.get().equals(Alliance.Red)){
    if (isRedAlliance) {
      return () -> RED_POSES;
    }

    return () -> BLUE_POSES;
  }
  // Conseguimos las poses del reef de los equipos azul y rojo
  public static final Supplier<List<Pose2d>> getReefPos() {
    if (isRedAlliance) {
      // if(FieldConstants.ALLIANCE.isPresent() &&
      // FieldConstants.ALLIANCE.get().equals(Alliance.Red)){
      return () -> RED_REEF;
    }

    return () -> BLUE_REEF;
  }
  // Conseguimos las poses del feeder de los equipos azul y rojo
  public static final Supplier<List<Pose2d>> getFeederPos() {
    if (isRedAlliance) {
      // if(FieldConstants.ALLIANCE.isPresent() &&
      // FieldConstants.ALLIANCE.get().equals(Alliance.Red)){
      return () -> RED_FEEDER;
    }

    return () -> BLUE_FEEDER;
  }
}
