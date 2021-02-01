/*
 * File: Autonomous.java
 * Project: Robot Programming 2021
 * File Created: Monday, 18th January 2021 5:32 pm
 * Author: Manuel Diaz and Obed Garcia
 * 
 * Copyright (c) 2021 WinT 3794. Under MIT License.
 */

package org.wint3794.frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import org.wint3794.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Autonomous extends CommandBase {

  private final Drivetrain drivetrain;

  public Autonomous(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    String trajectoryJSON = "paths/Main.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      System.out.println("LOLOOLOLOLO");
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
