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
    // AutoTrajectory DRToDEF = routine.trajectory("DRToDEF");
    // AutoTrajectory DEFToFL = routine.trajectory("DEFToFL");

    routine.active().onTrue(Commands.sequence(centerToDR.resetOdometry(), centerToDR.cmd()));

    centerToDR
        .atTime(0.3)
        .onTrue(
            Commands.deadline(
                    Commands.waitSeconds(6),
                    elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                    Commands.waitSeconds(4)
                        .andThen(outtake.setVoltage(() -> -2).andThen(Commands.waitSeconds(2))))
                .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.ZERO)))
    // .andThen(DRToDEF.cmd()))
    ;

    // DRToDEF.done()
    //     .onTrue(
    //         outtake
    //             .setVoltage(() -> -2)
    //             .until(() -> outtake.getDetected())
    //             .andThen(outtake.setVoltage(() -> 0))
    //             .andThen(DEFToFL.cmd()));

    // DEFToFL.atTime(0.3)
    //     .onTrue(
    //         Commands.deadline(
    //                 Commands.waitSeconds(6),
    //                 elevator.setSetpoint(() -> ElevatorSetpoint.L4),
    //                 Commands.waitSeconds(4)
    //                     .andThen(outtake.setVoltage(() -> -2).andThen(Commands.waitSeconds(2))))
    //             .andThen(elevator.setSetpoint(() -> ElevatorSetpoint.ZERO)));
    return routine;
  }

  public AutoRoutine centerToDR4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerToDR4");

    AutoTrajectory centerToDR = routine.trajectory("centerToDR");

    routine.active().onTrue(Commands.sequence(centerToDR.resetOdometry(), centerToDR.cmd()));

    centerToDR
        .atTime(0.3)
        .onTrue(
            Commands.parallel(
                elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                Commands.waitSeconds(6).andThen(outtake.setVoltage(() -> -2))));

    return routine;
  }

  public AutoRoutine centerToDL4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerToDL4");

    AutoTrajectory centerToDL = routine.trajectory("mirrored_centerToDR");

    routine.active().onTrue(Commands.sequence(centerToDL.resetOdometry(), centerToDL.cmd()));

    centerToDL
        .atTime(0.3)
        .onTrue(
            Commands.parallel(
                elevator.setSetpoint(() -> ElevatorSetpoint.L4),
                Commands.waitSeconds(6).andThen(outtake.setVoltage(() -> -2))));

    return routine;
  }

  public AutoRoutine spitAuto() {
    AutoRoutine routine = autoFactory.newRoutine("spit");
    routine.active().onTrue(outtake.setVoltage(() -> -2));

    return routine;
  }
}
