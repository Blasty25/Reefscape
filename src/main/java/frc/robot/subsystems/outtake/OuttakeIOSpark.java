package frc.robot.subsystems.outtake;

import static frc.robot.util.SparkUtil.*;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.RobotMap;

public class OuttakeIOSpark implements OuttakeIO {
  private final SparkMax spark;
  private final Canandcolor canandcolor;

  public OuttakeIOSpark() {
    var sparkConfig = new SparkMaxConfig();
    sparkConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12.0);

    spark = new SparkMax(RobotMap.Outtake.motor, MotorType.kBrushless);

    CanandEventLoop.getInstance();
    canandcolor = new Canandcolor(RobotMap.Outtake.canandcolor);
    canandcolor.resetFactoryDefaults();

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void processInputs(OuttakeIOInputsAutoLogged inputs) {
    inputs.motorAppliedVolts = spark.getAppliedOutput();
    inputs.motorCurrentAmps = spark.getOutputCurrent();
    inputs.motorTempCelsius = spark.getMotorTemperature();
    inputs.sensorConnected = canandcolor.isConnected();
    inputs.sensorRaw = canandcolor.getProximity();
    inputs.sensorDetected = (canandcolor.getProximity() < 0.1);
    inputs.sensorTempCelsius = canandcolor.getTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    spark.setVoltage(voltage);
  }
}
