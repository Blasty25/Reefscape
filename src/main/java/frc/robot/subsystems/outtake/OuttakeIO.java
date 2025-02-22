package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public static class OuttakeIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void processInputs(final OuttakeIOInputsAutoLogged inputs) {}

  public default void setVoltage(double voltage) {}
}
