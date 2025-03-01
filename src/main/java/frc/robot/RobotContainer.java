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

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOSim;
import frc.robot.subsystems.outtake.OuttakeIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.AllianceFlipUtil;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Elevator elevator;
  public final Outtake outtake;
  public final Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final AutoFactory autoFactory;
  private final LoggedDashboardChooser<Command> autoChooser;

  private final EnumMap<ElevatorSetpoint, Command> outtakeCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSpark());
        outtake = new Outtake(new OuttakeIOSpark());
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {}); // disabled

        break;

      case SIM:
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        outtake = new Outtake(new OuttakeIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // outtakeCommands =
    //     new EnumMap<ElevatorSetpoint, Command>(
    //         Map.ofEntries(
    //             Map.entry(ElevatorSetpoint.ZERO, outtake.setVoltage(() -> -2)),
    //             Map.entry(
    //                 ElevatorSetpoint.INTAKE,
    //                 outtake
    //                     .setVoltage(() -> -2)
    //                     .until(() -> outtake.getDetected())
    //                     .andThen(outtake.setVoltage(() -> 0))),
    //             Map.entry(ElevatorSetpoint.L1, outtake.setVoltage(() -> 12)),
    //             Map.entry(ElevatorSetpoint.L2, outtake.setVoltage(() -> -2)),
    //             Map.entry(ElevatorSetpoint.DEALGAE2, outtake.setVoltage(() -> 12)),
    //             Map.entry(ElevatorSetpoint.L3, outtake.setVoltage(() -> -2)),
    //             Map.entry(ElevatorSetpoint.L4, outtake.setVoltage(() -> -2))));

    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            (sample, isStart) -> {
              Logger.recordOutput(
                  "ActiveTrajectory",
                  Arrays.stream(sample.getPoses())
                      .map(AllianceFlipUtil::apply)
                      .toArray(Pose2d[]::new));
            });

    AutoRoutines autoRoutines = new AutoRoutines(autoFactory, elevator, outtake);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Choreo Auto Chooser");
    autoChooser.addDefaultOption("None", Commands.print("No Auto Selected"));

    autoChooser.addOption("Center to DR4", autoRoutines.centerToDR4Auto().cmd());
    autoChooser.addOption("Center to DL4", autoRoutines.centerToDL4Auto().cmd());
    autoChooser.addOption("Spit", autoRoutines.spitAuto().cmd());

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> OperatorConstants.deadband));
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY() * OperatorConstants.precisionMode,
                () -> -driver.getLeftX() * OperatorConstants.precisionMode,
                () -> -driver.getRightX() * OperatorConstants.precisionMode,
                () -> OperatorConstants.deadband));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> new Rotation2d(),
                () -> OperatorConstants.deadband));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    driver.leftTrigger().onTrue(Commands.select(outtakeCommands, elevator::getSetpoint));

    driver.rightTrigger().onTrue(outtake.setVoltage(() -> 12)).onFalse(outtake.setVoltage(() -> 0));

    operator.y().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L4));
    operator.x().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L3));
    operator.b().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L2));
    operator.a().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L1));
    operator.povDown().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.ZERO));
    operator.povUp().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE));

    operator.leftTrigger().onTrue(elevator.homingSequence());

    operator
        .rightTrigger()
        .whileTrue(
            elevator.setVoltage(() -> 12.0 * MathUtil.applyDeadband(-operator.getLeftY(), 0.05)))
        .onFalse(elevator.setVoltage(() -> 0)); // TODO: changed operator control scheme to home
    // on left, test and get Connor's feedback on
    // input squaring
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
