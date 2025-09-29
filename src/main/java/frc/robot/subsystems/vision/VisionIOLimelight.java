package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  public final DoubleSubscriber latencySubscriber;
  public final DoubleSubscriber txSubscriber;
  public final DoubleSubscriber tySubscriber;

  public final DoubleArraySubscriber megatag1Subscriber;
  public final DoubleArraySubscriber megatag2Subscriber;

  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Updates in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

    NetworkTableInstance.getDefault().flush(); // Increases netwrok traffic but Limelight

    // Read a new pose from NT
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSamples : megatag1Subscriber.readQueue()) {
      if (rawSamples.value.length == 0) continue;
      for (int i = 11; i < rawSamples.value.length; i += 7) {
        tagIds.add((int) rawSamples.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSamples.timestamp * 1.0e-6 - rawSamples.value[6] * 1.0e-3,
              // 3D pose estimation
              parsePose(rawSamples.value),
              // Ambiguity, only applicable fotr the first tag
              rawSamples.value.length >= 18 ? rawSamples.value[17] : 0.0,
              // Tag count
              (int) rawSamples.value[7],
              // Average tag distance
              rawSamples.value[9],
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSamples : megatag2Subscriber.readQueue()) {
      if (rawSamples.value.length == 0) continue;
      for (int i = 11; i < rawSamples.value.length; i += 7) {
        tagIds.add((int) rawSamples.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSamples.timestamp * 1.0e-6 - rawSamples.value[6] * 1.0e-3,
              // 3D pose estimation
              parsePose(rawSamples.value),
              // Ambiguity equals to zero, because the pose is already disambiguiated
              0.0,
              // Tag count
              (int) rawSamples.value[7],
              // Average tag distance
              rawSamples.value[9],
              PoseObservationType.MEGATAG_2));
    }

    // Save the pose to the inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save the tags IDs to the inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  // Parses the 3D limelight pose into a botpose array
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
