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

  public TeleOp(Drivetrain m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(this.m_drivetrain);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(
      -m_driverController.getY(Hand.kLeft),
      m_driverController.getX(Hand.kLeft)
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
