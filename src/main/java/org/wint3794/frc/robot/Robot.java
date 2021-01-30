/*
 * File: Robot.java
 * Project: Robot Programming 2021
 * File Created: Monday, 18th January 2021 5:24 pm
 * Author: Manuel Diaz and Obed Garcia
 * 
 * Copyright (c) 2021 WinT 3794. Under MIT License.
 */

package org.wint3794.frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleOpCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_teleOpCommand = m_robotContainer.getTeleOpCommand();

    if (m_teleOpCommand != null) {
      m_teleOpCommand.schedule();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
