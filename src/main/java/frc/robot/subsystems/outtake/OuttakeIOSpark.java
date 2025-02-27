package frc.robot.subsystems.outtake;

import static frc.robot.util.SparkUtil.*;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.RobotMap;

public class OuttakeIOSpark implements OuttakeIO {
  private final SparkMax spark = new SparkMax(RobotMap.Outtake.motor, MotorType.kBrushless);
  private final RelativeEncoder encoder = spark.getEncoder();

  private final Canandcolor canandcolor;

  private final Debouncer sparkConnectedDebounce = new Debouncer(0.5);
  private final Debouncer currentDebounce = new Debouncer(0.2);

  public OuttakeIOSpark() {
    var sparkConfig = new SparkMaxConfig();
    sparkConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);

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
    sparkStickyFault = false;
    ifOk(spark, spark::getAppliedOutput, (value) -> inputs.motorAppliedVolts = value);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.motorCurrentAmps = value);
    ifOk(spark, spark::getMotorTemperature, (value) -> inputs.motorTempCelsius = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.motorVelocityRPM = value);
    inputs.motorConnected = sparkConnectedDebounce.calculate(!sparkStickyFault);
    inputs.motorStalled =
        currentDebounce.calculate(
            (Math.abs(spark.getOutputCurrent() - 40.0) < 5) && (encoder.getVelocity() < 100.0));

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
