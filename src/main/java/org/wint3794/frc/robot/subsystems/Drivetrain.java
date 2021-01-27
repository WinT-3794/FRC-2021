// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import org.wint3794.frc.robot.hardware.CANSparkMaxController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.wint3794.frc.robot.Robot;
import org.wint3794.frc.robot.util.Constants;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrivetrainSim m_driveSim;

  private CANSparkMaxController m_leftMotor = new CANSparkMaxController(1, MotorType.kBrushless);
  private CANSparkMaxController m_rightMotor = new CANSparkMaxController(2, MotorType.kBrushless);
   
  private CANSparkMaxController m_leftMotorSlave = new CANSparkMaxController(3,MotorType.kBrushless); 
  private CANSparkMaxController m_rightMotorSlave = new CANSparkMaxController(4, MotorType.kBrushless);

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private AHRS m_ahrs = new AHRS();

  private Field2d m_field = new Field2d();

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_ahrs.getRotation2d(),
    new Pose2d(0, 0, new Rotation2d())
  );

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    
    m_leftMotorSlave.follow(m_leftMotor); 
    m_rightMotorSlave.follow(m_rightMotor);
     
    if (Robot.isReal()) {

    } else {
      m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
        7.29,                    // 7.29:1 gearing reduction.
        7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
        60.0,                    // The mass of the robot is 60 kg.
        Units.inchesToMeters(3), // The robot uses 3" radius wheels.
        0.7112,                  // The track width is 0.7112 meters.
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

      /*m_driveSim = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(Constants.KvLinear, Constants.KaLinear, Constants.KvAngular, Constants.KaAngular),
        DCMotor.getNEO(3),
        0.7112,
        7.29,
        Units.inchesToMeters(3),
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));*/
    }

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * 10 / 4096);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * 10 / 4096);

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
     m_odometry.update(
      m_ahrs.getRotation2d(),
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

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_driveSim.getHeading().getDegrees());
  }

  public DifferentialDrive getDrive() {
    return m_drive;
  }
}
