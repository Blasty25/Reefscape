package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.gearing,
          ElevatorConstants.mass,
          ElevatorConstants.drumRadius,
          0,
          ElevatorConstants.travel,
          true,
          0);
  private double volts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    elevatorSim.update(0.02);

    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.motorVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.motorAppliedVolts = volts;
    inputs.motorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.motorTempCelsius = 20.0;
    inputs.followerTempCelsius = 20.0;
  }

  @Override
  public void setVoltage(double voltage) {
    volts = voltage;
    elevatorSim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void resetEncoder(double position) {}
}
