package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax spark = new SparkMax(ElevatorConstants.spark, MotorType.kBrushless);
  private final SparkMax followerSpark =
      new SparkMax(ElevatorConstants.followerSpark, MotorType.kBrushless);
  private final RelativeEncoder encoder = spark.getEncoder();

  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOSpark() {
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.current)
        .voltageCompensation(12.0)
        .inverted(true);
    sparkConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    sparkConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.current)
        .voltageCompensation(12.0)
        .follow(ElevatorConstants.spark, true)
        .inverted(true);
    followerConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    followerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    sparkStickyFault = false;
    ifOk(spark, encoder::getPosition, (value) -> inputs.positionMeters = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.motorVelocity = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.motorAppliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.motorCurrentAmps = value);
    inputs.motorConnected = motorConnectedDebounce.calculate(!sparkStickyFault);
    ifOk(spark, spark::getMotorTemperature, (value) -> inputs.motorTempCelsius = value);

    sparkStickyFault = false;
    inputs.followerConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
    ifOk(
        followerSpark,
        followerSpark::getMotorTemperature,
        (value) -> inputs.followerTempCelsius = value);
    ifOk(
        spark,
        new DoubleSupplier[] {followerSpark::getAppliedOutput, followerSpark::getBusVoltage},
        (values) -> inputs.followerAppliedVolts = values[0] * values[1]);
    ifOk(followerSpark, encoder::getVelocity, (value) -> inputs.followerVelocity = value);
    ifOk(
        followerSpark,
        followerSpark::getOutputCurrent,
        (value) -> inputs.followerCurrentAmps = value);
  }

  @Override
  public void setVoltage(double voltage) {
    spark.setVoltage(voltage);
  }

  @Override
  public void resetEncoder(double position) {
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }
}
