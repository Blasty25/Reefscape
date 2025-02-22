package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final double travel = Units.inchesToMeters(69.0);
  public static final double mass = Units.lbsToKilograms(25.1);

  public static final double drumRadius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
  public static final double gearing = (5.0 / 1.0);
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;
  public static final double tolerance = .01;

  public static final int current = 40;

  public static class Gains {
    public static final double[] kS = {0.0, 0.0, 0.0};
    public static final double[] kG = {0.7, 0.93, 1.475};
    public static final double kV = 3.41;
    public static final double[] kA = {0.07, 0.09, 0.0};

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final double maxVelocity = // 0.3;
      Units.inchesToMeters(124);
  public static final double maxAcceleration = // 0.3;
      Units.inchesToMeters(110.88) / 0.3;

  public static final double homingVolts = -1.0;
  public static final double homingTimeSecs = 0.2;
  public static final double homingVelocityThresh = 0.2;

  public static final double staticCharacterizationVelocityThresh = 0.1;

  public static final double visualizerOffset = 0.0;
}
