// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

public class Config {
  int driveMotorId;
  int turnMotorId;
  int encoderChannel;
  double encoderOffset;
  boolean isTurnInverted;

  public Config(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      double encoderOffset,
      boolean turnInverted) {
    this.driveMotorId = driveMotorId;
    this.turnMotorId = turnMotorId;
    this.encoderChannel = encoderChannel;
    this.encoderOffset = encoderOffset;
    this.isTurnInverted = turnInverted;
  }
}
