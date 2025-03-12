package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public ElevatorSetpoint setpoint = ElevatorSetpoint.ZERO;
    public double targetPositionMeters = 0.0;
    public double profiledTargetMeters = 0.0;
    public boolean limitSwitch = false;

    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorVelocityMetersPerSecond = 0.0;

    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerVelocityMetersPerSecond = 0.0;

    public boolean motorConnected = false;
    public double motorTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(final ElevatorIOInputsAutoLogged inputs) {}

  public default void setVoltage(final double voltage) {}

  public default void stop() {
    setVoltage(0);
  }

  public default void resetEncoder(final double position) {}

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
