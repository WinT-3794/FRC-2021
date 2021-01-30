/*
 * File: RobotContainer.java
 * Project: Robot Programming 2021
 * File Created: Sunday, 17th January 2021 8:14 pm
 * Author: Manuel Diaz and Obed Garcia
 * 
 * Copyright (c) 2021 WinT 3794. Under MIT License.
 */

package org.wint3794.frc.robot;

import org.wint3794.frc.robot.commands.Autonomous;
import org.wint3794.frc.robot.commands.TeleOp;
import org.wint3794.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Autonomous m_autoCommand = new Autonomous(m_drivetrain);
  private final TeleOp m_teleOp = new TeleOp(m_drivetrain);

  public RobotContainer() { }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public TeleOp getTeleOpCommand() {
    return m_teleOp;
  }
}
