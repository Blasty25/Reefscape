package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  public enum ElevatorSetpoint {
    ZERO,
    INTAKE,
    L1,
    L2,
    DEALGAE2,
    L3,
    L4
  }

  public static final EnumMap<ElevatorSetpoint, Double> elevatorHeights =
      new EnumMap<ElevatorSetpoint, Double>(
          Map.ofEntries(
              Map.entry(ElevatorSetpoint.ZERO, 0.0),
              Map.entry(ElevatorSetpoint.INTAKE, 0.057),
              Map.entry(ElevatorSetpoint.L1, 0.33),
              Map.entry(ElevatorSetpoint.L2, 0.63),
              Map.entry(ElevatorSetpoint.DEALGAE2, 0.81),
              Map.entry(ElevatorSetpoint.L3, 1.05),
              Map.entry(ElevatorSetpoint.L4, 1.76)));

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber[] kS = {
    new LoggedTunableNumber("Elevator/kS/Stage1"),
    new LoggedTunableNumber("Elevator/kS/Stage2"),
    new LoggedTunableNumber("Elevator/kS/Stage3")
  };
  private static final LoggedTunableNumber[] kG = {
    new LoggedTunableNumber("Elevator/kG/Stage1"),
    new LoggedTunableNumber("Elevator/kG/Stage2"),
    new LoggedTunableNumber("Elevator/kG/Stage3")
  };
  private static final LoggedTunableNumber[] kA = {
    new LoggedTunableNumber("Elevator/kA/Stage1"),
    new LoggedTunableNumber("Elevator/kA/Stage2"),
    new LoggedTunableNumber("Elevator/kA/Stage3")
  };
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", ElevatorConstants.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Elevator/MaxAccelerationMetersPerSec2", ElevatorConstants.maxAcceleration);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", ElevatorConstants.homingVolts);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", ElevatorConstants.homingTimeSecs);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber(
          "Elevator/HomingVelocityThresh", ElevatorConstants.homingVelocityThresh);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber(
          "Elevator/StaticCharacterizationVelocityThresh",
          ElevatorConstants.staticCharacterizationVelocityThresh);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/Tolerance", ElevatorConstants.tolerance);

  static {
    kP.initDefault(ElevatorConstants.Gains.kP);
    kD.initDefault(ElevatorConstants.Gains.kD);

    for (int stage = 0; stage < 3; stage++) {
      kS[stage].initDefault(ElevatorConstants.Gains.kS[stage]);
      kG[stage].initDefault(ElevatorConstants.Gains.kG[stage]);
      kA[stage].initDefault(ElevatorConstants.Gains.kA[stage]);
    }
  }

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);

  @AutoLogOutput private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  @AutoLogOutput(key = "Elevator/AtGoal")
  private boolean atGoal = false;

  private ElevatorFeedforward ff2, ff1, ff0;
  private ProfiledPIDController pid;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  public boolean hasZeroed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    ff2 =
        new ElevatorFeedforward(kS[2].get(), kG[2].get(), ElevatorConstants.Gains.kV, kA[2].get());
    ff1 =
        new ElevatorFeedforward(kS[1].get(), kG[1].get(), ElevatorConstants.Gains.kV, kA[1].get());
    ff0 =
        new ElevatorFeedforward(kS[0].get(), kG[0].get(), ElevatorConstants.Gains.kV, kA[0].get());

    pid =
        new ProfiledPIDController(
            kP.get(),
            ElevatorConstants.Gains.kI,
            kD.get(),
            new TrapezoidProfile.Constraints(
                maxVelocity.getAsDouble(), maxAcceleration.getAsDouble()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);

    if (kP.hasChanged(hashCode())) {
      pid.setP(kP.getAsDouble());
    }

    if (kD.hasChanged(hashCode())) {
      pid.setD(kD.getAsDouble());
    }

    if (kG[2].hasChanged(hashCode())) {
      ff2 =
          new ElevatorFeedforward(
              kS[2].getAsDouble(), kG[2].getAsDouble(), ElevatorConstants.Gains.kV);
    }

    atGoal =
        Math.abs(inputs.positionMeters - inputs.targetPositionMeters) < tolerance.getAsDouble();
    currentFilterValue = currentFilter.calculate(inputs.motorCurrentAmps);
  }

  public Command setSetpoint(Supplier<ElevatorSetpoint> setpoint) {
    return this.run(
        () -> {
          double meters = elevatorHeights.get(setpoint.get());
          double ffVolts =
              switch (getStages()) {
                case 0 -> ffVolts =
                    ff0.calculateWithVelocities(
                        inputs.motorVelocityMetersPerSecond, pid.getSetpoint().velocity);
                case 1 -> ffVolts =
                    ff1.calculateWithVelocities(
                        inputs.motorVelocityMetersPerSecond, pid.getSetpoint().velocity);
                default -> ffVolts =
                    ff2.calculateWithVelocities(
                        inputs.motorVelocityMetersPerSecond, pid.getSetpoint().velocity);
              };
          double pidVolts = pid.calculate(inputs.positionMeters, meters);

          double volts = pidVolts + ffVolts * Math.signum(pidVolts);
          Logger.recordOutput("Elevator/ffVolts", ffVolts * Math.signum(pidVolts));
          Logger.recordOutput("Elevator/pidVolts", pidVolts);
          io.setVoltage(volts);
          inputs.setpoint = setpoint.get();
          inputs.targetPositionMeters = meters;
          inputs.profiledTargetMeters = pid.getSetpoint().position;
        });
  }

  public Command setSetpoint(ElevatorSetpoint setpoint) {
    return this.setSetpoint(() -> setpoint);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
        });
  }

  public Command setVoltage(double voltage) {
    return this.setVoltage(() -> voltage);
  }

  public Command resetEncoder() {
    return this.run(() -> io.resetEncoder(0.0));
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(homingVolts.getAsDouble());
              Logger.recordOutput("Elevator/Setpoint", Double.NaN);
            })
        .until(() -> currentFilterValue > 20.0)
        .finallyDo(
            (interrupted) -> {
              if (!interrupted) {
                io.resetEncoder(0.0);
                hasZeroed = true;
              }
            });
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              io.setVoltage(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.motorVelocityMetersPerSecond) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(
            () -> {
              io.resetEncoder();
              homed = true;
            });
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public double getTargetMeters() {
    return inputs.targetPositionMeters;
  }

  public ElevatorSetpoint getSetpoint() {
    return inputs.setpoint;
  }

  public double getVelocity() {
    return inputs.motorVelocityMetersPerSecond;
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  @AutoLogOutput(key = "Elevator/Intaking")
  public boolean intaking() {
    return ((inputs.setpoint == ElevatorSetpoint.INTAKE)
        && (Math.abs(inputs.positionMeters - elevatorHeights.get(ElevatorSetpoint.INTAKE)) < 0.2));
  }

  public int getStages() {
    if (inputs.positionMeters
        > ElevatorConstants.stageOneTravel + ElevatorConstants.stageTwoTravel) {
      return 2;
    } else if (inputs.positionMeters > ElevatorConstants.stageOneTravel) {
      return 1;
    } else {
      return 0;
    }
  }

  public Command reset() {
    return this.run(
        () -> {
          inputs.setpoint = ElevatorSetpoint.ZERO;
        });
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.setVoltage(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () -> inputs.motorVelocityMetersPerSecond >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              timer.stop();
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
