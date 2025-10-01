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
// GNU General Public License for more det+ails.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final SwerveSubsystem drive;
  private final ElevatorSubsystem elevator;

  // Controller
  private final CommandXboxController operator_controller = new CommandXboxController(1);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        /*
        // Real robot, instantiate hardware IO implementations
        drive =
            new SwerveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                driver_controller);
        */
        elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        /*
        drive =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                driver_controller);*/
        elevator = new ElevatorSubsystem(new ElevatorIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        /*
        drive =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driver_controller);*/
        elevator = new ElevatorSubsystem(new ElevatorIO() {});

        break;
    }

    /*
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
    */
    // Configure the button bindings
    // configureDriver(driver_controller);
    configureOperatorBindings(operator_controller);
  }

  /*
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
  }*/

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
        .povRight()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setDesiredState(
                        ElevatorSubsystem.DesiredState.PREP_LVL,
                        ElevatorConstants.L2.in(Units.Rotations))));
    controller
        .y()
        .onTrue(
            Commands.runOnce(() -> elevator.setDesiredState(ElevatorSubsystem.DesiredState.HOME)));

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setDesiredState(
                        ElevatorSubsystem.DesiredState.PREP_LVL,
                        ElevatorConstants.L1.in(Units.Rotations))));
  }

  public Command getAutonomousCommand() {
    return null;
    // return autoChooser.get();
  }
}
