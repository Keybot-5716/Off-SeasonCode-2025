package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.SuperstructureConstants.AlgaeIntake;
import frc.robot.subsystems.SuperstructureConstants.AlgaeLevel;
import frc.robot.subsystems.SuperstructureConstants.BranchType;
import frc.robot.subsystems.SuperstructureConstants.ReefLevel;
import frc.robot.subsystems.SuperstructureConstants.RobotMode;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.Rollers.RollerConstants;
import frc.robot.subsystems.arm.Rollers.RollerSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final SwerveSubsystem swerveSub;
  private final ElevatorSubsystem elevatorSub;
  private final ArmSubsystem armSub;
  private final RollerSubsystem rollersSub;

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
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4,
    RESET,
    OUTTAKE_CORAL,
    RETRIEVE_SCORE,
    TO_FEEDER,
    INTAKE_CORAL,
    TAKE_CORAL,
    INTAKE_ALGAE,
    ALGAE_LOW_INTAKE,
    ALGAE_HIGH_INTAKE,
    GO_PROCESSOR,
    SCORE_PROCESSOR,
    GO_NET,
    SCORE_NET,
    OVERRIDE_CORAL,
    PREP_CLIMB,
    CLIMB
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
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4,
    PREP_AUTO_L1,
    PREP_AUTO_L2,
    PREP_AUTO_L3,
    PREP_AUTO_L4,
    RESET,
    OUTTAKE_CORAL,
    RETRIEVE_SCORE,
    OUTTAKE_AUTO_CORAL,
    TO_FEEDER,
    INTAKE_CORAL,
    TAKE_CORAL,
    INTAKE_ALGAE,
    ALGAE_LOW_INTAKE,
    ALGAE_HIGH_INTAKE,
    GO_PROCESSOR,
    SCORE_PROCESSOR,
    GO_NET,
    SCORE_NET,
    OVERRIDE_CORAL,
    PREP_CLIMB,
    CLIMB
  }

  private DesiredState desiredState = DesiredState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState lastCurrentState;

  // ROBOT MODE
  private RobotMode robotMode = RobotMode.CORAL;
  private RobotMode desiredRobotMode;

  // CORAL
  private ReefLevel reefLevel;
  private ReefLevel desiredReefLevel;

  private BranchType branchType;
  private BranchType desiredBranch;

  // ALGAE
  private AlgaeLevel algaeLevel;
  private AlgaeLevel desiredAlgaeLevel;

  private AlgaeIntake algaeIntake;
  private AlgaeIntake desiredAlgaeIntake;

  // CORAL INTAKE OVERRIDE
  boolean isIntakeOverride = false;

  public Superstructure(
      SwerveSubsystem swerveSub,
      ElevatorSubsystem elevatorSub,
      ArmSubsystem armSub,
      RollerSubsystem rollersSub) {
    this.swerveSub = swerveSub;
    this.elevatorSub = elevatorSub;
    this.armSub = armSub;
    this.rollersSub = rollersSub;

    desiredRobotMode = RobotMode.CORAL;

    // CORAL
    desiredBranch = BranchType.LEFT;
    desiredReefLevel = ReefLevel.L1;

    // ALGAE
    desiredAlgaeLevel = AlgaeLevel.NET;
    desiredAlgaeIntake = AlgaeIntake.LOW_ALGAE;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Superstructure/DesiredState", desiredState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/LastCurrentState", lastCurrentState);

    Logger.recordOutput("Superstructure/Current Mode", robotMode);
    // CORAL
    Logger.recordOutput("Superstructure/Coral/Current Reef Level", reefLevel);
    Logger.recordOutput("Superstructure/Coral/Current Branch", branchType);
    // ALGAE
    Logger.recordOutput("Superstructure/Algae/Current Intake", desiredAlgaeIntake);
    Logger.recordOutput("Superstructure/Algae/Current Level", desiredAlgaeLevel);

    currentState = setStateTransitions();
    robotMode = desiredRobotMode;
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
      case PREP_L1:
        currentState = isAutonomous ? CurrentState.PREP_AUTO_L1 : CurrentState.PREP_L1;
        break;
      case PREP_L2:
        currentState = isAutonomous ? CurrentState.PREP_AUTO_L2 : CurrentState.PREP_L2;
        break;
      case PREP_L3:
        currentState = isAutonomous ? CurrentState.PREP_AUTO_L3 : CurrentState.PREP_L3;
        break;
      case PREP_L4:
        currentState = isAutonomous ? CurrentState.PREP_AUTO_L4 : CurrentState.PREP_L4;
        break;
      case RESET:
        currentState = CurrentState.RESET;
        break;
      case OUTTAKE_CORAL:
        currentState = isAutonomous ? CurrentState.OUTTAKE_AUTO_CORAL : CurrentState.OUTTAKE_CORAL;
        break;
      case RETRIEVE_SCORE:
        currentState = CurrentState.RETRIEVE_SCORE;
        break;
      case TO_FEEDER:
        currentState = CurrentState.TO_FEEDER;
        break;
      case INTAKE_CORAL:
        currentState = CurrentState.INTAKE_CORAL;
        break;
      case TAKE_CORAL:
        currentState = CurrentState.TAKE_CORAL;
        break;
      case INTAKE_ALGAE:
        currentState = CurrentState.INTAKE_ALGAE;
        break;
      case ALGAE_LOW_INTAKE:
        currentState = CurrentState.ALGAE_LOW_INTAKE;
        break;
      case ALGAE_HIGH_INTAKE:
        currentState = CurrentState.ALGAE_HIGH_INTAKE;
        break;
      case GO_NET:
        currentState = CurrentState.GO_NET;
        break;
      case SCORE_NET:
        currentState = CurrentState.SCORE_NET;
        break;
      case GO_PROCESSOR:
        currentState = CurrentState.GO_PROCESSOR;
        break;
      case SCORE_PROCESSOR:
        currentState = CurrentState.SCORE_PROCESSOR;
        break;
      case OVERRIDE_CORAL:
        currentState = CurrentState.OVERRIDE_CORAL;
        break;
      case PREP_CLIMB:
        currentState = CurrentState.PREP_CLIMB;
        break;
      case CLIMB:
        currentState = CurrentState.CLIMB;
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
        autoAllighAUTOL1(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L1:
        autoAllighAUTOL1(BranchType.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L2:
        autoAllighAUTOL2(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L2:
        autoAllighAUTOL2(BranchType.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L3:
        autoAllighAUTOL3(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L3:
        autoAllighAUTOL3(BranchType.RIGHT);
        break;
      case SCORE_LEFT_AUTO_L4:
        autoAllighAUTOL4(BranchType.LEFT);
        break;
      case SCORE_RIGHT_AUTO_L4:
        autoAllighAUTOL4(BranchType.RIGHT);
        break;
        // ---- Teleop Scoring Cases
      case SCORE_LEFT_TELEOP_L1:
        autoAllignL1(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L1:
        autoAllignL1(BranchType.RIGHT);
        break;
      case SCORE_LEFT_TELEOP_L2:
        autoAllignL2(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L2:
        autoAllignL2(BranchType.RIGHT);
        break;
      case SCORE_LEFT_TELEOP_L3:
        autoAllignL3(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L3:
        autoAllignL3(BranchType.RIGHT);
        break;
      case SCORE_LEFT_TELEOP_L4:
        autoAllignL4(BranchType.LEFT);
        break;
      case SCORE_RIGHT_TELEOP_L4:
        autoAllignL4(BranchType.RIGHT);
        break;
      case PREP_L1:
        goToL1();
        break;
      case PREP_L2:
        goToL2();
        break;
      case PREP_L3:
        goToL3();
        break;
      case PREP_L4:
        goToL4();
        break;
      case PREP_AUTO_L1:
        break;
      case PREP_AUTO_L2:
        break;
      case PREP_AUTO_L3:
        break;
      case PREP_AUTO_L4:
        break;
      case RESET:
        break;
      case OUTTAKE_CORAL:
        score(desiredReefLevel);
        break;
      case RETRIEVE_SCORE:
        retrieve();
      case OUTTAKE_AUTO_CORAL:
        scoreAuto(desiredReefLevel);
        break;
      case TO_FEEDER:
        rotateToFeeder();
        break;
      case INTAKE_CORAL:
        intakeCoral();
        break;
      case TAKE_CORAL:
        takeCoral();
        break;
      case INTAKE_ALGAE:
        intakeAlgae();
        break;
      case ALGAE_LOW_INTAKE:
        intakeLowALgae();
        break;
      case ALGAE_HIGH_INTAKE:
        intakeHighAlgae();
        break;
      case GO_PROCESSOR:
        prepProcessor();
        break;
      case SCORE_PROCESSOR:
        scoreProcessor();
        break;
      case GO_NET:
        prepNet();
        break;
      case SCORE_NET:
        scoreNet();
        break;
      case OVERRIDE_CORAL:
        overrideIntakeCoral();
        break;
      case PREP_CLIMB:
        prepClimb();
        break;
      case CLIMB:
        climb();
        break;
    }
  }

  private void stopped() {
    swerveSub.setDesiredState(SwerveSubsystem.DesiredState.MANUAL_DRIVE);
    elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.STOPPED);
    armSub.setDesiredState(ArmSubsystem.DesiredState.STOPPED);
    rollersSub.setDesiredState(RollerSubsystem.DesiredState.DEFAULT);
  }

  private void home() {
    swerveSub.setDesiredState(SwerveSubsystem.DesiredState.MANUAL_DRIVE);
    elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.HOME);
    armSub.setDesiredState(ArmSubsystem.DesiredState.HOME);
    rollersSub.setDesiredState(RollerSubsystem.DesiredState.DEFAULT);
  }

  // ==== Base Autonomous States
  private void autoAllighAUTOL1(BranchType branchType) {}

  private void autoAllighAUTOL2(BranchType branchType) {}

  private void autoAllighAUTOL3(BranchType branchType) {}

  private void autoAllighAUTOL4(BranchType branchType) {}

  // ==== Teleop States
  // -- Score States
  private void autoAllignL1(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void autoAllignL2(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void autoAllignL3(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void autoAllignL4(BranchType branchType) {
    swerveSub.setDesiredPose(
        getDesiredReef(branchType == BranchType.LEFT), SwerveSubsystem.maxMetersToReef);
  }

  private void goToL1() {
    elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L1);
    armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.L1);
  }

  private void goToL2() {
    elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L2);
    armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.L2);
  }

  private void goToL3() {
    elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L3);
    if (elevatorSub.isPositioned(ElevatorConstants.L3, 0.1)) {
      armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.L3);
    }
  }

  private void goToL4() {
    elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L4);
    if (elevatorSub.isPositioned(ElevatorConstants.L4, 0.03)) {
      armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.L4);
    }
  }

  private void score(ReefLevel level) {
    switch (level) {
      case L1:
        break;
      case L2:
        armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.SCORE_L2);
        elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L2);
        rollersSub.setDesiredState(RollerSubsystem.DesiredState.REVERSE, 0.3);

        break;
      case L3:
        armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.SCORE_L3);
        elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L3);
        rollersSub.setDesiredState(RollerSubsystem.DesiredState.REVERSE, 0.3);

        break;
      case L4:
        armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.SCORE_L4);
        elevatorSub.setDesiredState(ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.L4);
        rollersSub.setDesiredState(RollerSubsystem.DesiredState.REVERSE, 0.3);

        break;
    }
  }

  private void retrieve() {}

  private void scoreAuto(ReefLevel level) {
    switch (level) {
      case L1:
        break;
      case L2:
        break;
      case L3:
        break;
      case L4:
        break;
    }
  }

  // -- Feeder States
  private void rotateToFeeder() {
    swerveSub.setDesiredPoseToRotate(getDesiredFeeder());
  }

  // -- Coral States

  private void intakeCoral() {
    elevatorSub.setDesiredState(
        ElevatorSubsystem.DesiredState.PREP_LVL, ElevatorConstants.INTAKE_CORAL);
    armSub.setDesiredState(ArmSubsystem.DesiredState.PREP_LVL, ArmConstants.NONE);
    rollersSub.setDesiredState(RollerSubsystem.DesiredState.DEFAULT);
  }

  private void takeCoral() {
    rollersSub.setDesiredState(RollerSubsystem.DesiredState.FORWARD, RollerConstants.TAKE_CORAL);
  }

  private void overrideIntakeCoral() {}

  // -- Algae States

  private void intakeAlgae() {}

  private void intakeLowALgae() {}

  private void intakeHighAlgae() {}

  private void prepNet() {}

  private void scoreNet() {}

  private void prepProcessor() {}

  private void scoreProcessor() {}

  private void prepClimb() {}

  private void climb() {}

  public void setDesiredState(DesiredState state) {
    this.desiredState = state;
  }

  public Command superstructureCommand(DesiredState state) {
    return new InstantCommand(() -> setDesiredState(state));
  }

  public RobotMode getCurrentRobotMode() {
    return robotMode;
  }

  public void setDesiredRobotMode(RobotMode mode) {
    desiredRobotMode = mode;
  }

  // -- CORAL --
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

  // -- ALGAE --
  public AlgaeIntake getCurrentAlgaeIntake() {
    return algaeIntake;
  }

  public void setDesiredAlgaeIntake(AlgaeIntake type) {
    desiredAlgaeIntake = type;
  }

  public AlgaeLevel getCurrentAlgaeLevel() {
    return algaeLevel;
  }

  public void setDesiredAlgaeLevel(AlgaeLevel type) {
    desiredAlgaeLevel = type;
  }

  public Pose2d getDesiredReef(boolean leftRequest) {
    List<Pose2d> reef = FieldConstants.getReefPos().get();
    Pose2d currentPose = swerveSub.getPose();
    Pose2d desiredReef = currentPose.nearest(reef);
    int closestIndex = reef.indexOf(desiredReef);

    if (closestIndex > 3 && closestIndex < 10) {
      leftRequest = !leftRequest;
    }

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

  // -- OPERATOR COMMANDS
  public Command setMode1OperatorSystem() {
    return Commands.either(
        Commands.runOnce(() -> setDesiredReefLevel(ReefLevel.L1)),
        Commands.runOnce(() -> setDesiredAlgaeLevel(AlgaeLevel.PROCESSOR)),
        () -> robotMode == RobotMode.CORAL);
  }

  public Command setMode2OperatorSystem() {
    return Commands.either(
        Commands.runOnce(() -> setDesiredReefLevel(ReefLevel.L2)),
        Commands.runOnce(() -> setDesiredAlgaeIntake(AlgaeIntake.LOW_ALGAE)),
        () -> robotMode == RobotMode.CORAL);
  }

  public Command setMode3OperatorSystem() {
    return Commands.either(
        Commands.runOnce(() -> setDesiredReefLevel(ReefLevel.L3)),
        Commands.runOnce(() -> setDesiredAlgaeIntake(AlgaeIntake.HIGH_ALGAE)),
        () -> robotMode == RobotMode.CORAL);
  }

  public Command setMode4OperatorSystem() {
    return Commands.either(
        Commands.runOnce(() -> setDesiredReefLevel(ReefLevel.L4)),
        Commands.runOnce(() -> setDesiredAlgaeLevel(AlgaeLevel.NET)),
        () -> robotMode == RobotMode.CORAL);
  }

  // SCORE COMMAND
  public Command scoreCommand(BranchType type) {
    return Commands.sequence(
        superstructureCommand(DesiredState.SCORE_LEFT_L1),
        superstructureCommand(DesiredState.PREP_L1),
        Commands.waitUntil(() -> elevatorSub.isAtDesiredPos()));
  }

  // Sketch Intake Command
}
