package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.util.VisionResult;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private VisionIO io;

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public VisionResult[] getUnreadResults() {
    return io.getUnreadResults();
  }

  public void update(Pose2d pose) {
    io.setRobotPose(pose);
  }

  public Pose2d[] getRobotPose() {
    VisionResult[] results = getUnreadResults();
    Pose2d[] poses = new Pose2d[results.length];
    for (int i=0; i<poses.length; i++) {
      poses[i] = results[i].getPose2d();
    }
    return poses;
  }


  @Override
  public void periodic() {
    io.updateInputs();

    VisionResult[] measuredPoses = getUnreadResults();

    for (int i = 0; i < measuredPoses.length; i++) {
      Logger.recordOutput(
          "/Vision/Camera" + (i + 1) + "/Estimated Pose", measuredPoses[i].getPose2d());
    }
  }
}
