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
import frc.robot.subsystems.Intake.IntakeSubsys;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.SuperstructureConstants.ReefLevel;
import frc.robot.subsystems.SuperstructureConstants.RobotMode;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
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
  private final IntakeSubsys intake;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driver_controller = new CommandXboxController(0);
  private final CommandXboxController operator_controller = new CommandXboxController(1);

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
        intake = new IntakeSubsys();
        superstructure = new Superstructure(drive, elevator, intake);

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
        elevator = new ElevatorSubsystem(new ElevatorIOSim());
        intake = new IntakeSubsys();
        superstructure = new Superstructure(drive, elevator, intake);

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
        intake = new IntakeSubsys();
        superstructure = new Superstructure(drive, elevator, intake);

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
        .onFalse(superstructure.superstructureCommand(DesiredState.DEFAULT));
    controller
        .leftBumper()
        .onTrue(superstructure.superstructureCommand(DesiredState.TO_FEEDER))
        .onFalse(superstructure.superstructureCommand(DesiredState.DEFAULT));

    controller
        .a()
        .onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(0, 0, new Rotation2d()))));
  }

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
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
