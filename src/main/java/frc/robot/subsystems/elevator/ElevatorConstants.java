package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class ElevatorConstants {

  public static final int ELEVATOR_ID = 0;

  public static final double GEARBOX_REDUCTION = 0;
  public static final Angle FORWARD_THRESHOLD = Units.Rotations.of(0);
  public static final Angle REVERSE_THRESHOLD = Units.Rotations.of(0);

  public static final double MIN_OFFSET = 0.1;

  public static final Angle NONE = Units.Rotations.of(0.1);
  public static final Angle REST = Units.Rotations.of(0.1);

  // CORAL
  public static final Angle L1 = Units.Rotations.of(0.1);
  public static final Angle L2 = Units.Rotations.of(0.1);
  public static final Angle L3 = Units.Rotations.of(0.1);
  public static final Angle L4 = Units.Rotations.of(0.1);
  public static final Angle TAKE_CORAL = Units.Rotations.of(0.1);

  // ALGAE
  public static final Angle GROUND = Units.Rotations.of(0.1);
  public static final Angle LOW_ALGAE = Units.Rotations.of(0.1);
  public static final Angle HIGH_ALGAE = Units.Rotations.of(0.1);
  public static final Angle NET = Units.Rotations.of(0.1);
  public static final Angle PROCESSOR = Units.Rotations.of(0.1);
}
