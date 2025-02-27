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

  public AutoRoutine centerToG4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerToG4");

    AutoTrajectory centerToG = routine.trajectory("centerToG");

    routine.active().onTrue(Commands.sequence(centerToG.resetOdometry(), centerToG.cmd()));

    centerToG.atTime(0.5).whileTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L4));
    centerToG.done().onTrue(outtake.setVoltage(() -> -2));

    return routine;
  }

  public AutoRoutine centerToH4Auto() {
    AutoRoutine routine = autoFactory.newRoutine("centerToH4");

    AutoTrajectory centerToH = routine.trajectory("mirrored_centerToG");

    routine.active().onTrue(Commands.sequence(centerToH.resetOdometry(), centerToH.cmd()));

    centerToH.atTime(0.5).whileTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L4));
    centerToH.done().onTrue(outtake.setVoltage(() -> -2));

    return routine;
  }
}
