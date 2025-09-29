package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static AprilTagFieldLayout aprilTagFieldLayoutAndyMark =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public static AprilTagFieldLayout aprilTagFieldLayoutWelded =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static String cameraName = "limelight";

  // Not used in code, only limelight interface
  public static Transform3d robotToCamera = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

  // Basic thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 // Camera
      };

  // Multipliers to apply for MT2 observations
  public static double linearStdDevMegatag2Factor = 0.5;
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
