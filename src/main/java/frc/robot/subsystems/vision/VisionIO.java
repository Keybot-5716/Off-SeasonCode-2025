package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTON_VISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
