package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.SuperstructureConstants.BranchType;
import frc.robot.subsystems.SuperstructureConstants.ReefLevel;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final SwerveSubsystem swerveSub;

  private boolean isAutonomous = DriverStation.isAutonomous();

  public enum DesiredState {
    STOPPED,
    HOME,
    DEFAULT,
    SCORE_LEFT_L1,
    SCORE_LEFT_L2,
    SCORE_LEFT_L3,
    SCORE_LEFT_L4,
    SCORE_RIGHT_L1,
    SCORE_RIGHT_L2,
    SCORE_RIGHT_L3,
    SCORE_RIGHT_L4,
    TO_FEEDER
  }

  public enum CurrentState {
    STOPPED,
    HOME,
    SCORE_LEFT_TELEOP_L1,
    SCORE_LEFT_TELEOP_L2,
    SCORE_LEFT_TELEOP_L3,
    SCORE_LEFT_TELEOP_L4,
    SCORE_RIGHT_TELEOP_L1,
    SCORE_RIGHT_TELEOP_L2,
    SCORE_RIGHT_TELEOP_L3,
    SCORE_RIGHT_TELEOP_L4,
    SCORE_LEFT_AUTO_L1,
    SCORE_LEFT_AUTO_L2,
    SCORE_LEFT_AUTO_L3,
    SCORE_LEFT_AUTO_L4,
    SCORE_RIGHT_AUTO_L1,
    SCORE_RIGHT_AUTO_L2,
    SCORE_RIGHT_AUTO_L3,
    SCORE_RIGHT_AUTO_L4,
    TO_FEEDER
  }

  private DesiredState desiredState = DesiredState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState lastCurrentState;

  private ReefLevel reefLevel;
  private ReefLevel desiredReefLevel;

  private BranchType branchType;
  private BranchType desiredBranch;

  public Superstructure(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;

    desiredBranch = BranchType.LEFT;
    desiredReefLevel = ReefLevel.L1;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Superstructure/DesiredState", desiredState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/LastCurrentState", lastCurrentState);

    Logger.recordOutput("Superstructure/Current Reef Level", reefLevel);
    Logger.recordOutput("Superstructure/Current Branch", branchType);

    currentState = setStateTransitions();
    reefLevel = desiredReefLevel;
    branchType = desiredBranch;

    tryStates();
  }

  private CurrentState setStateTransitions() {
    lastCurrentState = currentState;
    switch (desiredState) {
      default:
        currentState = CurrentState.STOPPED;
        break;
      case HOME:
        currentState = CurrentState.HOME;
        break;
      case DEFAULT:
        currentState = CurrentState.STOPPED;
        break;
      case SCORE_LEFT_L1:
        currentState =
            isAutonomous ? CurrentState.SCORE_LEFT_AUTO_L1 : CurrentState.SCORE_LEFT_TELEOP_L1;
        break;
      case SCORE_RIGHT_L1:
        currentState =
            isAutonomous ? CurrentState.SCORE_RIGHT_AUTO_L1 : CurrentState.SCORE_RIGHT_TELEOP_L1;
        break;
      case SCORE_LEFT_L2:
        currentState =
            isAutonomous ? CurrentState.SCORE_LEFT_AUTO_L2 : CurrentState.SCORE_LEFT_TELEOP_L2;
        break;
      case SCORE_RIGHT_L2:
        currentState =
            isAutonomous ? CurrentState.SCORE_RIGHT_AUTO_L2 : CurrentState.SCORE_RIGHT_TELEOP_L2;
        break;
      case SCORE_LEFT_L3:
        currentState =
            isAutonomous ? CurrentState.SCORE_LEFT_AUTO_L3 : CurrentState.SCORE_LEFT_TELEOP_L3;
        break;
      case SCORE_RIGHT_L3:
        currentState =
            isAutonomous ? CurrentState.SCORE_RIGHT_AUTO_L3 : CurrentState.SCORE_RIGHT_TELEOP_L3;
        break;
      case SCORE_LEFT_L4:
        currentState =
            isAutonomous ? CurrentState.SCORE_LEFT_AUTO_L4 : CurrentState.SCORE_LEFT_TELEOP_L4;
        break;
      case SCORE_RIGHT_L4:
        currentState =
            isAutonomous ? CurrentState.SCORE_RIGHT_AUTO_L4 : CurrentState.SCORE_RIGHT_TELEOP_L4;
        break;
      case TO_FEEDER:
        currentState = CurrentState.TO_FEEDER;
        break;
    }

    return currentState;
  }

  private void tryStates() {
    switch (currentState) {
        // ---- Default Cases
      case STOPPED:
        stopped();
        break;
      case HOME:
        home();
        break;
        // ---- Autonomous Score Cases
      case SCORE_LEFT_AUTO_L1:
        scoreAutoL1(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L1:
        scoreAutoL1(BranchType.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L2:
        scoreAutoL2(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L2:
        scoreAutoL2(BranchType.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L3:
        scoreAutoL3(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L3:
        scoreAutoL3(BranchType.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L4:
        scoreAutoL4(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L4:
        scoreAutoL4(BranchType.RIGHT);
        break;
        // ---- Teleop Scoring Cases
      case SCORE_LEFT_TELEOP_L1:
        scoreL1Teleop(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L1:
        scoreL1Teleop(BranchType.RIGHT);
        break;
      case SCORE_LEFT_TELEOP_L2:
        scoreL2Teleop(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L2:
        scoreL2Teleop(BranchType.RIGHT);
        break;
      case SCORE_LEFT_TELEOP_L3:
        scoreL3Teleop(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L3:
        scoreL3Teleop(BranchType.RIGHT);
        break;
      case SCORE_LEFT_TELEOP_L4:
        scoreL4Teleop(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L4:
        scoreL4Teleop(BranchType.RIGHT);
        break;
      case TO_FEEDER:
        rotateToFeeder();
        break;
    }
  }

  private void stopped() {
    swerveSub.setDesiredState(SwerveSubsystem.DesiredState.MANUAL_DRIVE);
  }

  private void home() {
    swerveSub.setDesiredState(SwerveSubsystem.DesiredState.ROBOT_RELATIVE);
  }

  // ==== Base Autonomous States
  private void scoreAutoL1(BranchType branchType) {}

  private void scoreAutoL2(BranchType branchType) {}

  private void scoreAutoL3(BranchType branchType) {}

  private void scoreAutoL4(BranchType branchType) {}

  // ==== Teleop States
  // -- Score States
  private void scoreL1Teleop(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void scoreL2Teleop(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void scoreL3Teleop(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void scoreL4Teleop(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  // -- Feeder States
  private void rotateToFeeder() {
    swerveSub.setDesiredPoseToRotate(getDesiredFeeder());
  }

  public void setDesiredState(DesiredState state) {
    this.desiredState = state;
  }

  public Command superstructureCommand(DesiredState state) {
    Command cmd = new InstantCommand(() -> setDesiredState(state));
    return cmd;
  }

  public ReefLevel getCurrentReefLevel() {
    return reefLevel;
  }

  public void setDesiredReefLevel(ReefLevel level) {
    desiredReefLevel = level;
  }

  public void selectBranchType() {
    if (branchType == BranchType.LEFT) this.getDesiredReef(true);
    else if (branchType == BranchType.RIGHT) this.getDesiredReef(false);
  }

  public BranchType getBranchType() {
    return branchType;
  }

  public Pose2d getDesiredReef(boolean leftRequest) {
    List<Pose2d> reef = FieldConstants.getReefPos().get();
    Pose2d currentPose = swerveSub.getPose();
    Pose2d desiredReef = currentPose.nearest(reef);
    int closestIndex = reef.indexOf(desiredReef);
    /*
    if(closestIndex > 3 && closestIndex < 10) {
    leftRequest = !leftRequest;
    }

    */

    if (leftRequest && (closestIndex % 2 == 1)) {
      desiredReef = reef.get(closestIndex - 1);
    } else if (!leftRequest && (closestIndex % 2 == 0)) {
      desiredReef = reef.get(closestIndex + 1);
    }

    return desiredReef;
  }

  public Pose2d getDesiredFeeder() {
    List<Pose2d> feeder = FieldConstants.getFeederPos().get();
    Pose2d currentPose = swerveSub.getPose();
    Pose2d desiredFeeder = currentPose.nearest(feeder);
    int closestIndex = feeder.indexOf(desiredFeeder);

    desiredFeeder = feeder.get(closestIndex);

    return desiredFeeder;
  }
}
