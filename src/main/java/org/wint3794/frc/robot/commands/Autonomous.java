package org.wint3794.frc.robot.commands;

import org.wint3794.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Autonomous extends CommandBase {

  private final Drivetrain drivetrain;

  public Autonomous(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
