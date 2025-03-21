// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Commands.AutoRightFind;
import frc.robot.subsystems.drive.Commands.DriveCommands;
import frc.robot.subsystems.drive.Commands.HumanPlayerRoute;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Commands.Autovator;
import frc.robot.subsystems.elevator.Commands.Reset;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.outtake.Commands.Shoot;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIOSim;
import frc.robot.subsystems.outtake.OuttakeIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  public Drive drive;
  public Elevator elevator;
  public Outtake outtake;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Command pathplannerChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.isReal()) {
      drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      elevator = new Elevator(new ElevatorIOSpark());
      outtake = new Outtake(new OuttakeIOSpark());
    } else {
      // Sim robot, instantiate physics sim IO implementations
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      elevator = new Elevator(new ElevatorIOSim());
      outtake = new Outtake(new OuttakeIOSim());
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Tuning Auto Chooser");
    autoChooser.addOption("None", Commands.print("No Auto Selected"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Elevator static", elevator.staticCharacterization(1.0));
    pathplannerChooser = new PathPlannerAuto("middle");
    autoChooser.addDefaultOption("Middle", pathplannerChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> OperatorConstants.deadband,
            () -> 1));
    driver
        .x()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> OperatorConstants.deadband,
                () -> OperatorConstants.precisionMode));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> new Rotation2d(),
                () -> OperatorConstants.deadband));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver.y().onTrue(Commands.runOnce(drive::logPose, drive));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driver
        .leftTrigger()
        .whileTrue(
            outtake
                .setVoltage(() -> -2)
                .until(() -> outtake.getDetected() && elevator.intaking())
                .andThen(() -> outtake.setVoltage(0)));

    driver.rightTrigger().onTrue(outtake.setVoltage(() -> 12)).onFalse(outtake.setVoltage(() -> 0));

    operator.y().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L4));
    operator.x().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L3));
    operator.b().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L2));
    operator.a().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L1));
    operator.povDown().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.ZERO));
    operator.povUp().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE));

    operator.leftTrigger().onTrue(elevator.homingSequence().andThen(elevator.reset()));

    // PARMS (Subsystem, Alliance) if True BLUE alliancce if False RED alliance
    // driver.leftBumper().whileTrue(new AutoLeftFind(drive, Constants.allianceMode));
    driver.rightBumper().whileTrue(new AutoRightFind(drive, Constants.allianceMode));
    driver.povDown().whileTrue(new HumanPlayerRoute(drive, Constants.allianceMode));
    driver.leftBumper().onTrue(drive.followTraj());

    operator
        .rightTrigger()
        .whileTrue(
            elevator.setVoltage(() -> 12.0 * MathUtil.applyDeadband(-operator.getRightY(), 0.05)))
        .onFalse(elevator.setVoltage(() -> 0));
    // on left, test and get Connor's feedback on
    // input squaring
  }

  public Command getAutonomousCommand() {
    NamedCommands.registerCommand("autovator", new Autovator(elevator));
    NamedCommands.registerCommand("reset", new Reset(elevator, outtake));
    NamedCommands.registerCommand("shoot", new Shoot(outtake)); // check voltage later
    return autoChooser.get();
  }
}
