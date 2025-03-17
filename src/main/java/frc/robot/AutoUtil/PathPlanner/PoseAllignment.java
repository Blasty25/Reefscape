// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoUtil.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class PoseAllignment {

  public List<Pose2d> poseLeft = new ArrayList<>();
  public List<Pose2d> poseRight = new ArrayList<>();

  public PoseAllignment() {
    poseLeft.add(new Pose2d(3.057, 4.175, new Rotation2d()));
    poseLeft.add(new Pose2d(3.608, 2.820, new Rotation2d(59.744)));
    poseLeft.add(new Pose2d(5.083, 2.688, new Rotation2d(122.152)));
    poseLeft.add(new Pose2d(5.970, 3.851, new Rotation2d(-180.000)));
    poseLeft.add(new Pose2d(5.359, 5.242, new Rotation2d(-120.838)));
    poseLeft.add(new Pose2d(3.884, 5.374, new Rotation2d(-59.886)));

    poseRight.add(7, new Pose2d(3.021, 3.863, new Rotation2d()));
    poseRight.add(8, new Pose2d(3.848, 2.688, new Rotation2d(59.744)));
    poseRight.add(9, new Pose2d(5.347, 2.868, new Rotation2d(122.152)));
    poseRight.add(10, new Pose2d(5.958, 4.151, new Rotation2d(180)));
    poseRight.add(11, new Pose2d(5.095, 5.374, new Rotation2d(-120.838)));
    poseRight.add(12, new Pose2d(3.656, 5.242, new Rotation2d()));
  }
}
