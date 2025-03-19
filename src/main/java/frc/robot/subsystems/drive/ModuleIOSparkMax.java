// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ModuleIOSparkMax implements ModuleIO {
  private SparkMax driveSparky;
  private SparkMax turnSparky;

  private AnalogEncoder encoder;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private SparkClosedLoopController driveController;
  private SparkClosedLoopController turnController;

  private SparkMaxConfig driveConfig = new SparkMaxConfig();
  private SparkMaxConfig turnConfig = new SparkMaxConfig();

  private double encoderOffset;

  private static final LoggedTunableNumber driveP = new LoggedTunableNumber("Spark/Drive/P", 0.0);
  private static final LoggedTunableNumber driveI = new LoggedTunableNumber("Spark/Drive/I", 0.0);
  private static final LoggedTunableNumber driveD = new LoggedTunableNumber("Spark/Drive/D", 0.0);

  private static final LoggedTunableNumber turnP = new LoggedTunableNumber("Spark/turn/P", 0.0);
  private static final LoggedTunableNumber turnI = new LoggedTunableNumber("Spark/turn/I", 0.0);
  private static final LoggedTunableNumber turnD = new LoggedTunableNumber("Spark/turn/D", 0.0);

  public ModuleIOSparkMax(Config config) {
    driveSparky = new SparkMax(config.driveMotorId, MotorType.kBrushless);
    turnSparky = new SparkMax(config.turnMotorId, MotorType.kBrushless);

    encoder = new AnalogEncoder(config.encoderChannel);

    encoderOffset = config.encoderOffset;

    driveConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(80);
    driveConfig
        .encoder
        .positionConversionFactor(DriveConstants.drivePositionConversionFactor)
        .velocityConversionFactor(DriveConstants.driveVelocityFactor);
    driveConfig.closedLoop.pid(driveP.getAsDouble(), driveI.getAsDouble(), driveD.getAsDouble());

    turnConfig.inverted(config.isTurnInverted).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    turnConfig
        .encoder
        .positionConversionFactor(DriveConstants.turnPositionConversionFactor)
        .velocityConversionFactor(DriveConstants.turnVelocityFactor);
    turnConfig.closedLoop.pid(turnP.getAsDouble(), turnI.getAsDouble(), turnD.getAsDouble());
    turnConfig
        .closedLoop
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .positionWrappingEnabled(true);

    driveSparky.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparky.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveSparky.getEncoder();
    turnEncoder = turnSparky.getEncoder();

    driveController = driveSparky.getClosedLoopController();
    turnController = turnSparky.getClosedLoopController();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.driveAppliedVolts = driveSparky.getBusVoltage();
    inputs.driveCurrentAmps = driveSparky.getOutputCurrent();
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();

    inputs.encoderOffset = encoder.get() - encoderOffset;

    inputs.turnPosition = new Rotation2d(turnEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnConnected = true;
    inputs.turnCurrentAmps = turnSparky.getOutputCurrent();
    inputs.turnAppliedVolts = turnSparky.getAppliedOutput();
    inputs.turnEncoderConnected = true;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveController.setReference(velocityRotPerSec, ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnController.setReference(rotation.getRotations(), ControlType.kPosition);
  }

  @Override
  public void setDriveOpenLoop(double output) {}

  @Override
  public void setTurnOpenLoop(double output) {}
}
