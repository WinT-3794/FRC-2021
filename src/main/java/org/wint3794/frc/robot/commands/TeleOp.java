// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc.robot.commands;

import org.wint3794.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleOp extends CommandBase {
  
  private final XboxController m_driverController = new XboxController(0);

  private final Timer m_timer = new Timer();

  private final Drivetrain m_drivetrain;

  /** Creates a new TeleOp. */
  public TeleOp(Drivetrain m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(this.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.getDrive().arcadeDrive(
      m_driverController.getX(Hand.kRight),
      -m_driverController.getY(Hand.kLeft)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
