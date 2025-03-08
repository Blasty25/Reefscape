package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.outtake.Outtake;

public class AutoRoutines {
  private final AutoFactory autoFactory;

  private final Elevator elevator;
  private final Outtake outtake;

  public AutoRoutines(AutoFactory autoFactory, Elevator elevator, Outtake outtake) {
    this.autoFactory = autoFactory;

    this.elevator = elevator;
    this.outtake = outtake;
  }

  public AutoRoutine centerToDR4toFL4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerDR4FL4");

    AutoTrajectory centerToDR = routine.trajectory("centerToDR");
    AutoTrajectory DRToDEF = routine.trajectory("DRToDEF");
    AutoTrajectory DEFToFL = routine.trajectory("DEFToFL");

    routine.active().onTrue(Commands.sequence(centerToDR.resetOdometry(), centerToDR.cmd()));

    centerToDR
        .atTime(0.3)
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(5),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(3).andThen(outtake.setVoltage(() -> -4)))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE)));
    centerToDR.done().onTrue(Commands.waitSeconds(4).andThen(DRToDEF.cmd()));

    DRToDEF.done()
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(4),
                    outtake
                        .setVoltage(() -> -2)
                        .until(() -> outtake.getDetected())
                        .andThen(outtake.setVoltage(() -> 0)))
                .andThen(DEFToFL.cmd()));

    DEFToFL.done()
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(5),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(3).andThen(outtake.setVoltage(() -> -4)))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE)));
    return routine;
  }

  public AutoRoutine centerResetAuto() {
    AutoRoutine routine = autoFactory.newRoutine("centerReset");
    AutoTrajectory centerToDR = routine.trajectory("centerToDR");

    routine.active().onTrue(Commands.sequence(centerToDR.resetOdometry()));

    return routine;
  }

  public AutoRoutine centerToDR4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerToDR4");

    AutoTrajectory centerToDR = routine.trajectory("centerToDR");

    routine.active().onTrue(Commands.sequence(centerToDR.resetOdometry(), centerToDR.cmd()));

    centerToDR
        .atTime(0.3)
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(5),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(3).andThen(outtake.setVoltage(() -> -4)))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE)));

    return routine;
  }

  public AutoRoutine centerToDL4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerToDL4");

    AutoTrajectory centerToDL = routine.trajectory("mirrored_centerToDR");

    routine.active().onTrue(Commands.sequence(centerToDL.resetOdometry(), centerToDL.cmd()));

    centerToDL
        .atTime(0.3)
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(5),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(3).andThen(outtake.setVoltage(() -> -4)))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE)));

    return routine;
  }

  public AutoRoutine blueXToER4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("blueXToER4");

    AutoTrajectory blueXToER = routine.trajectory("blueXToER");

    routine.active().onTrue(Commands.sequence(blueXToER.resetOdometry(), blueXToER.cmd()));

    blueXToER
        .atTime(0.3)
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(5),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(3).andThen(outtake.setVoltage(() -> -4)))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE)));

    return routine;
  }

  public AutoRoutine redXToCL4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("redXToCL4");

    AutoTrajectory redXToCL = routine.trajectory("mirrored_blueXToER");
    AutoTrajectory CLToredX = routine.trajectory("mirrored_ERToblueX");

    routine.active().onTrue(Commands.sequence(redXToCL.resetOdometry(), redXToCL.cmd()));

    redXToCL
        .atTime(0.3)
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(5),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(3).andThen(outtake.setVoltage(() -> -4)))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE)));

    return routine;
  }

  public AutoRoutine centerDriveAuto() {
    AutoRoutine routine = autoFactory.newRoutine("centerDrive");

    AutoTrajectory drive = routine.trajectory("centerDrive");

    routine.active().onTrue(Commands.sequence(drive.resetOdometry(), drive.cmd()));

    return routine;
  }

  public AutoRoutine blueXDriveAuto() {
    AutoRoutine routine = autoFactory.newRoutine("blueXDrive");

    AutoTrajectory blueXDrive = routine.trajectory("blueXDrive");

    routine.active().onTrue(Commands.sequence(blueXDrive.resetOdometry(), blueXDrive.cmd()));

    return routine;
  }

  public AutoRoutine redXDriveAuto() {
    AutoRoutine routine = autoFactory.newRoutine("redXDrive");

    AutoTrajectory redXDrive = routine.trajectory("mirrored_blueXDrive");

    routine.active().onTrue(Commands.sequence(redXDrive.resetOdometry(), redXDrive.cmd()));

    return routine;
  }

  public AutoRoutine spitAuto() {
    AutoRoutine routine = autoFactory.newRoutine("spit");
    routine.active().onTrue(outtake.setVoltage(() -> -2));

    return routine;
  }
}
