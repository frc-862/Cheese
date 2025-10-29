// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.util.LightningRobot;

public class Robot extends TimedRobot {

  @SuppressWarnings("resource")
  public Robot() {
    new LightningRobot(new RobotContainer());
  }

}
