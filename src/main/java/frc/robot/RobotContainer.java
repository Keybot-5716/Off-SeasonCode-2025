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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import frc.robot.subsystems.arm.Rollers.RollerIOSparkMax;
import frc.robot.subsystems.arm.Rollers.RollerSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
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
  private final VisionSubsystem vision;
  private final ClimberSubsystem climber;

  // Controller
  private final CommandXboxController driver_controller = new CommandXboxController(1);
  private final CommandXboxController Emergency_Operator_controller = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Método para cargar archivos .auto desde un directorio
  private void loadAutoFiles(String directory) {
    Path autoDirectory = Paths.get(Filesystem.getDeployDirectory().toString(), directory);
    System.out.println("Buscando archivos en: " + autoDirectory.toString());

    if (!Files.exists(autoDirectory)) {
      System.err.println("El directorio no existe: " + autoDirectory.toString());
      return;
    }

    try (Stream<Path> paths = Files.list(autoDirectory)) {
      paths
          .filter(Files::isRegularFile)
          .filter(path -> path.toString().endsWith(".auto"))
          .forEach(
              path -> {
                try {
                  // Leer el contenido del archivo .auto
                  String content = Files.readString(path);
                  JSONParser parser = new JSONParser();
                  JSONObject autoData = (JSONObject) parser.parse(content);

                  // Obtener el nombre del archivo sin la extensión
                  String autoName = path.getFileName().toString().replace(".auto", "");

                  System.out.println("Cargando archivo: " + autoName);

                  // Crear un comando de ejemplo (puedes personalizarlo según tus necesidades)
                  Command autoCommand =
                      new InstantCommand(
                          () -> {
                            System.out.println("Ejecutando auto: " + autoName);
                          });

                  // Agregar el comando al autoChooser
                  autoChooser.addOption(autoName, autoCommand);
                } catch (Exception e) {
                  DriverStation.reportError(
                      "Error al cargar el archivo .auto: " + path, e.getStackTrace());
                }
              });
    } catch (IOException e) {
      DriverStation.reportError(
          "Error al listar archivos en el directorio: " + autoDirectory, e.getStackTrace());
    }
  }
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
        rollers = new RollerSubsystem(new RollerIOSparkMax() {});
        climber = new ClimberSubsystem(new ClimberIOTalonFX());
        vision =
            new VisionSubsystem(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-pepelin", drive::getRotation));
        superstructure = new Superstructure(drive, elevator, arm, rollers, climber);

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
        rollers = new RollerSubsystem(new RollerIOSparkMax());
        climber = new ClimberSubsystem(new ClimberIO() {});
        vision =
            new VisionSubsystem(
                drive::addVisionMeasurement,
                new VisionIOSim(
                    VisionConstants.cameraName, VisionConstants.robotToCamera, drive::getPose));
        superstructure = new Superstructure(drive, elevator, arm, rollers, climber);

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
        climber = new ClimberSubsystem(new ClimberIO() {});
        vision = new VisionSubsystem(drive::addVisionMeasurement, new VisionIO() {});
        superstructure = new Superstructure(drive, elevator, arm, rollers, climber);

        break;
    }

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>("Autonomous papupro", AutoBuilder.buildAutoChooser());

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

    configNamedCommands();
    configureEmergencyOperatorBindings(Emergency_Operator_controller);
  }

  private void configureDriver(CommandXboxController controller) {

    controller
        .leftBumper()
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
                () -> superstructure.getCurrentReefLevel() == ReefLevel.L1));
    controller
        .rightBumper()
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
                () -> superstructure.getCurrentReefLevel() == ReefLevel.L1));

    controller
        .start()
        .onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(0, 0, new Rotation2d()))));
    controller.a().onTrue(superstructure.superstructureCommand(DesiredState.HOME));
    controller.back().onTrue(superstructure.superstructureCommand(DesiredState.PREP_L1));
    controller.b().onTrue(superstructure.superstructureCommand(DesiredState.PREP_L2));
    controller.x().onTrue(superstructure.superstructureCommand(DesiredState.PREP_L3));
    controller.y().onTrue(superstructure.superstructureCommand(DesiredState.PREP_L4));
    controller
        .rightTrigger()
        .onTrue(superstructure.superstructureCommand(DesiredState.OUTTAKE_CORAL));
    controller
        .leftTrigger()
        .whileTrue(superstructure.superstructureCommand(DesiredState.INTAKE_CORAL))
        .onFalse(
            Commands.runOnce(() -> superstructure.setDesiredState(DesiredState.TAKE_CORAL))
                .andThen(Commands.waitSeconds(0.7))
                .andThen(
                    Commands.runOnce(() -> superstructure.setDesiredState(DesiredState.HOME))));
    controller.povUp().onTrue(superstructure.setRobotStateCmd());
    controller
        .povLeft()
        .whileTrue(
            Commands.run(
                () -> rollers.setDesiredState(RollerSubsystem.DesiredState.FORWARD, 0.85)));

    controller
        .povRight()
        .whileTrue(
            Commands.run(() -> rollers.setDesiredState(RollerSubsystem.DesiredState.REVERSE, 0.4)));
    controller
        .povDown()
        .onTrue(superstructure.superstructureCommand(DesiredState.CLIMB))
        .onFalse(superstructure.superstructureCommand(DesiredState.STOPPED));
  }

  public void configNamedCommands() {
    NamedCommands.registerCommand(
        "Giroooos",
        Commands.runOnce(
            () -> {
              System.out.println("yeahhhh");
            }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureEmergencyOperatorBindings(CommandXboxController controller) {
    // Configure operator button bindings here

    controller.a().onTrue(new InstantCommand(() -> arm.setTargetPosition(50))); // ejemplo
    controller.povUp().onTrue(new InstantCommand(() -> arm.adjustTargetPosition(+5)));
    controller.povDown().onTrue(new InstantCommand(() -> arm.adjustTargetPosition(-5)));
  }
}
