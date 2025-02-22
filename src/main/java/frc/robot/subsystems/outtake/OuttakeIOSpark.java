package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.RobotMap;

public class OuttakeIOSpark implements OuttakeIO {
  private final SparkMax spark;

  public OuttakeIOSpark() {
    spark = new SparkMax(RobotMap.Outtake.motor, MotorType.kBrushless);
  }

  @Override
  public void processInputs(OuttakeIOInputsAutoLogged inputs) {
    inputs.appliedVolts = spark.getAppliedOutput();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.tempCelsius = spark.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    spark.setVoltage(voltage);
  }
}
