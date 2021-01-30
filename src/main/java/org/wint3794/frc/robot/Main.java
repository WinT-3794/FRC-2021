/*
 * File: Main.java
 * Project: Robot Programming 2021
 * File Created: Monday, 18th January 2021 5:21 pm
 * Author: Manuel Diaz and Obed Garcia
 * 
 * Copyright (c) 2021 WinT 3794. Under MIT License.
 */

package org.wint3794.frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
