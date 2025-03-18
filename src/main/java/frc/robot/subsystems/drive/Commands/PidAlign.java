// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PidAlign extends Command {

  private Drive drive;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private double translSpeedLim = 4.0; // MPS
  private double speedTol = 0.10;
  private double speedToRot = Math.PI / 16;
  private boolean alliance;
  private PIDController xPID = new PIDController(0.5, 0.0, 0.0);
  private PIDController yPID = new PIDController(0.5, 0.0, 0.0);
  private PIDController headingPID = new PIDController(0.5, 0.0, 0.0);
  private Pose2d tolerancePose = new Pose2d(0.05, 0.05, new Rotation2d(3.0));

  public PidAlign(Drive drive, boolean alliance) {
    this.drive = drive;
    this.targetPose = drive.autoLeftPose(alliance);
    this.currentPose = drive.getPose();
    this.alliance = alliance;
  }

  @Override
  public void initialize() {
    headingPID.enableContinuousInput(-Math.PI, Math.PI);

    // set tolerance
    xPID.setTolerance(tolerancePose.getX());
    yPID.setTolerance(tolerancePose.getY());
    headingPID.setTolerance(tolerancePose.getRotation().getRadians());

    xPID.reset();
    yPID.reset();
    headingPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(calculate(currentPose, targetPose));
  }

  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d endPose) {
    xPID.setSetpoint(endPose.getX());
    yPID.setSetpoint(endPose.getY());
    headingPID.setSetpoint(endPose.getRotation().getRadians());

    Logger.recordOutput("Drive/PID/Pose", targetPose.getTranslation());
    Logger.recordOutput("Drive/ENDPID/XSetpoints", xPID.calculate(endPose.getX()));
    Logger.recordOutput("Drive/ENDPID/YSetpoint", yPID.calculate(endPose.getY()));

    Logger.recordOutput("Drive/STARTPID/XSetpoints", xPID.calculate(currentPose.getX()));
    Logger.recordOutput("Drive/STARTPID/YSetpoint", yPID.calculate(currentPose.getY()));

    // calcualte feedback
    double xFeedback =
        MathUtil.clamp(xPID.calculate(currentPose.getX()), -translSpeedLim, translSpeedLim);
    double yFeedback =
        MathUtil.clamp(yPID.calculate(currentPose.getY()), -translSpeedLim, translSpeedLim);
    double headingFeedback =
        MathUtil.clamp(
            headingPID.calculate(currentPose.getRotation().getRadians()),
            -speedToRot,
            speedToRot); // Replace current pose with Vision.getRoation

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedback, yFeedback, headingFeedback, currentPose.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint()
        && yPID.atSetpoint()
        && headingPID.atSetpoint()
        && Math.hypot(drive.getSpeed().vxMetersPerSecond, drive.getSpeed().vyMetersPerSecond)
            < speedTol
        && drive.getSpeed().omegaRadiansPerSecond < speedToRot;
  }
}
