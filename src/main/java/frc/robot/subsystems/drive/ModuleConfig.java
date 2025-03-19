// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

public class ModuleConfig {

  public Config configure(int index) {
    Config config;
    switch (index) {
      case 0:
        config = new Config(1, 2, 1, 0.74, true);
        return config;
      case 1:
        config = new Config(3, 4, 2, 0.35, true);
        return config;
      case 2:
        config = new Config(5, 6, 3, 0.51, true);
        return config;
      case 3:
        config = new Config(7, 8, 4, 0.25, true);
        return config;
      default:
        config = new Config(0, 0, 0, 0.0, false);
        return config;
    }
  }
}
