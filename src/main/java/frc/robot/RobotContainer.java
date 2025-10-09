// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.SuperstructureConstants.ReefLevel;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.Rollers.RollerIO;
import frc.robot.subsystems.arm.Rollers.RollerSubsystem;
import frc.robot.subsystems.arm.Rollers.RollerTalonFX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drive;
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final RollerSubsystem rollers;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driver_controller = new CommandXboxController(1);
  private final CommandXboxController operator_controller = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new SwerveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                driver_controller);
        elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());
        arm = new ArmSubsystem(new ArmIOTalonFX());
        rollers = new RollerSubsystem(new RollerTalonFX());
        superstructure = new Superstructure(drive, elevator, arm, rollers);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                driver_controller);
        elevator = new ElevatorSubsystem(new ElevatorIO() {});
        arm = new ArmSubsystem(new ArmIO() {});
        rollers = new RollerSubsystem(new RollerIO() {});
        superstructure = new Superstructure(drive, elevator, arm, rollers);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driver_controller);
        elevator = new ElevatorSubsystem(new ElevatorIO() {});
        arm = new ArmSubsystem(new ArmIO() {});
        rollers = new RollerSubsystem(new RollerIO() {});
        superstructure = new Superstructure(drive, elevator, arm, rollers);

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureDriver(driver_controller);
    configureOperatorBindings(operator_controller);
  }

  private void configureDriver(CommandXboxController controller) {
    /*
    controller
        .leftTrigger()
        .onTrue(
            Commands.either(
                superstructure.superstructureCommand(DesiredState.SCORE_LEFT_L1),
                Commands.either(
                    superstructure.superstructureCommand(DesiredState.SCORE_LEFT_L2),
                    Commands.either(
                        superstructure.superstructureCommand(DesiredState.SCORE_LEFT_L3),
                        Commands.either(
                            superstructure.superstructureCommand(DesiredState.SCORE_LEFT_L4),
                            superstructure.superstructureCommand(DesiredState.DEFAULT),
                            () -> superstructure.getCurrentReefLevel() == ReefLevel.L4),
                        () -> superstructure.getCurrentReefLevel() == ReefLevel.L3),
                    () -> superstructure.getCurrentReefLevel() == ReefLevel.L2),
                () -> superstructure.getCurrentReefLevel() == ReefLevel.L1))
        .onFalse(superstructure.superstructureCommand(DesiredState.DEFAULT));
    controller
        .rightTrigger()
        .onTrue(
            Commands.either(
                superstructure.superstructureCommand(DesiredState.SCORE_RIGHT_L1),
                Commands.either(
                    superstructure.superstructureCommand(DesiredState.SCORE_RIGHT_L2),
                    Commands.either(
                        superstructure.superstructureCommand(DesiredState.SCORE_RIGHT_L3),
                        Commands.either(
                            superstructure.superstructureCommand(DesiredState.SCORE_RIGHT_L4),
                            superstructure.superstructureCommand(DesiredState.DEFAULT),
                            () -> superstructure.getCurrentReefLevel() == ReefLevel.L4),
                        () -> superstructure.getCurrentReefLevel() == ReefLevel.L3),
                    () -> superstructure.getCurrentReefLevel() == ReefLevel.L2),
                () -> superstructure.getCurrentReefLevel() == ReefLevel.L1))
        .onFalse(superstructure.superstructureCommand(DesiredState.DEFAULT)); */
    controller
        .leftBumper()
        .onTrue(superstructure.superstructureCommand(DesiredState.TO_FEEDER))
        .onFalse(superstructure.superstructureCommand(DesiredState.DEFAULT));

    controller
        .start()
        .onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(0, 0, new Rotation2d()))));

    controller
        .x()
        .onTrue(
            superstructure
                .superstructureCommand(DesiredState.PREP_L3)
                .alongWith(
                    Commands.runOnce(() -> superstructure.setDesiredReefLevel(ReefLevel.L3))));
    controller.a().onTrue(superstructure.changeButtons(DesiredState.HOME_CORAL, DesiredState.HOME_ALGAE));

    controller
        .y()
        .onTrue(
            superstructure
                .changeButtons(DesiredState.PREP_L4, DesiredState.ALGAE_HIGH_INTAKE)
                .alongWith(
                    Commands.runOnce(() -> superstructure.setDesiredReefLevel(ReefLevel.L4))));
    controller
        .rightTrigger()
        .onTrue(superstructure.changeButtons(DesiredState.OUTTAKE_CORAL, DesiredState.OUTTAKE_ALGAE));
    controller
        .b()
        .onTrue(
            superstructure
                .superstructureCommand(DesiredState.PREP_L2)
                .alongWith(
                    Commands.runOnce(() -> superstructure.setDesiredReefLevel(ReefLevel.L2))));
    controller
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> superstructure.changeButtons(DesiredState.INTAKE_CORAL, DesiredState.INTAKE_ALGAE)))
        .onFalse(
            Commands.runOnce(() -> superstructure.setDesiredState(DesiredState.STOPPED))
                .andThen(
                    Commands.waitSeconds(1.5)
                        .andThen(
                            Commands.runOnce(
                                    () -> superstructure.changeButtons(DesiredState.TAKE_CORAL, DesiredState.TAKE_ALGAE)))
                                .andThen(Commands.waitSeconds(0.5))
                                .andThen(
                                    Commands.runOnce(
                                        () ->
                                            superstructure.changeButtons(DesiredState.HOME_CORAL, DesiredState.HOME_ALGAE)))));
  }

  private void configureOperatorBindings(CommandXboxController controller) {
    controller
        .a()
        .whileTrue(
            Commands.run(
                () ->
                    elevator.setDesiredStateWithOutput(ElevatorSubsystem.DesiredState.MANUAL, 0.1)))
        .onFalse(
            Commands.runOnce(
                () -> elevator.setDesiredState(ElevatorSubsystem.DesiredState.STOPPED)));
    controller
        .b()
        .whileTrue(
            Commands.run(
                () ->
                    elevator.setDesiredStateWithOutput(
                        ElevatorSubsystem.DesiredState.MANUAL, -0.1)))
        .onFalse(
            Commands.runOnce(
                () -> elevator.setDesiredState(ElevatorSubsystem.DesiredState.STOPPED)));
    controller
        .x()
        .whileTrue(
            Commands.run(
                () -> arm.setDesiredStateWithOutput(ArmSubsystem.DesiredState.MANUAL, 0.3)))
        .onFalse(Commands.runOnce(() -> arm.setDesiredState(ArmSubsystem.DesiredState.STOPPED)));
    controller
        .y()
        .whileTrue(
            Commands.run(
                () -> arm.setDesiredStateWithOutput(ArmSubsystem.DesiredState.MANUAL, -0.1)))
        .onFalse(Commands.runOnce(() -> arm.setDesiredState(ArmSubsystem.DesiredState.STOPPED)));
    controller
        .rightBumper()
        .whileTrue(
            Commands.run(() -> rollers.setDesiredState(RollerSubsystem.DesiredState.FORWARD, 0.5)))
        .onFalse(
            Commands.runOnce(() -> rollers.setDesiredState(RollerSubsystem.DesiredState.DEFAULT)));
    controller
        .leftBumper()
        .whileTrue(
            Commands.run(() -> rollers.setDesiredState(RollerSubsystem.DesiredState.REVERSE, 0.3)))
        .onFalse(
            Commands.runOnce(() -> rollers.setDesiredState(RollerSubsystem.DesiredState.DEFAULT)));
  }
  /*
  private void configureOperatorBindings(CommandXboxController controller) {
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setDesiredRobotMode(RobotMode.CORAL)));
    controller
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setDesiredRobotMode(RobotMode.ALGAE)));
    controller.povUp().onTrue(superstructure.setMode1OperatorSystem());
    controller.povRight().onTrue(superstructure.setMode2OperatorSystem());
    controller.povDown().onTrue(superstructure.setMode3OperatorSystem());
    controller.povLeft().onTrue(superstructure.setMode4OperatorSystem());
  } */

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
