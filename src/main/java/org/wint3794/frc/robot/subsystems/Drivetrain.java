// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc.robot.subsystems;

import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;

import org.wint3794.frc.robot.Robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrivetrainSim m_driveSim;

  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(2, MotorType.kBrushless);
   
  private CANSparkMax m_leftMotorSlave = new CANSparkMax(3,MotorType.kBrushless); 
  private CANSparkMax m_rightMotorSlave = new CANSparkMax(4, MotorType.kBrushless);

   /*
  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  */

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private Field2d m_field = new Field2d();

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(),
    new Pose2d(0, 0, new Rotation2d())
  );

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    
    m_leftMotorSlave.follow(m_leftMotor); 
    m_rightMotorSlave.follow(m_rightMotor);
     
    if (Robot.isReal()) {

    } else {
      m_driveSim = new DifferentialDrivetrainSim(DCMotor.getNeo550(2), // 2 NEO motors on each side of the drivetrain.
          7.29, // 7.29:1 gearing reduction.
          7.5, // MOI of 7.5 kg m^2 (from CAD model).
          60.0, // The mass of the robot is 60 kg.
          Units.inchesToMeters(3), // The robot uses 3" radius wheels.
          0.7112, // The track width is 0.7112 meters.

          // The standard deviations for measurement noise:
          // x and y: 0.001 m
          // heading: 0.001 rad
          // l and r velocity: 0.1 m/s
          // l and r position: 0.005 m
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    }

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * 10 / 4096);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * 10 / 4096);

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
     m_odometry.update(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
     );

     m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
        m_rightMotor.get() * RobotController.getInputVoltage());

    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }

  public DifferentialDrive getDrive() {
    return m_drive;
  }
}
