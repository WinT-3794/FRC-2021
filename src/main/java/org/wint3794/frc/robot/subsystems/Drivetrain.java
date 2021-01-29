// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wint3794.frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import org.wint3794.frc.robot.hardware.CANSparkMaxController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.wint3794.frc.robot.Robot;
import org.wint3794.frc.robot.util.Constants.DrivetrainConstants;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrivetrainSim m_driveSim;

  private CANSparkMaxController m_leftMotor =
    new CANSparkMaxController(DrivetrainConstants.kMotorIDs[0][0], MotorType.kBrushless);   
  private CANSparkMaxController m_leftMotorSlave1 =
    new CANSparkMaxController(DrivetrainConstants.kMotorIDs[0][1],MotorType.kBrushless); 
  private CANSparkMaxController m_leftMotorSlave2 =
    new CANSparkMaxController(DrivetrainConstants.kMotorIDs[0][2],MotorType.kBrushless); 

  private CANSparkMaxController m_rightMotor = 
    new CANSparkMaxController(DrivetrainConstants.kMotorIDs[1][0], MotorType.kBrushless);
  private CANSparkMaxController m_rightMotorSlave1 = 
    new CANSparkMaxController(DrivetrainConstants.kMotorIDs[1][1], MotorType.kBrushless);
  private CANSparkMaxController m_rightMotorSlave2 = 
    new CANSparkMaxController(DrivetrainConstants.kMotorIDs[1][2], MotorType.kBrushless);

  private Encoder m_leftEncoder = 
    new Encoder(DrivetrainConstants.kLeftEncoderPorts[0], DrivetrainConstants.kLeftEncoderPorts[1], false);
  private Encoder m_rightEncoder = 
    new Encoder(DrivetrainConstants.kRightEncoderPorts[0], DrivetrainConstants.kRightEncoderPorts[1], true);

  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private AHRS m_ahrs = new AHRS();

  private final PIDController m_leftPIDController = 
    new PIDController(DrivetrainConstants.kPID[0], DrivetrainConstants.kPID[1], DrivetrainConstants.kPID[2]);
  private final PIDController m_rightPIDController =
    new PIDController(DrivetrainConstants.kPID[0], DrivetrainConstants.kPID[1], DrivetrainConstants.kPID[2]);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private Field2d m_field = new Field2d();

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_ahrs.getRotation2d());

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  private SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(
    m_leftMotor, m_leftMotorSlave1, m_leftMotorSlave2);

  private SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(
    m_rightMotor, m_rightMotorSlave1, m_rightMotorSlave2);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    if (Robot.isSimulation())  {
      m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(3),
        DrivetrainConstants.kGearingReduction,
        DrivetrainConstants.kMOI,
        DrivetrainConstants.kMass,
        DrivetrainConstants.kWheelRadius,
        DrivetrainConstants.kTrackWidth,
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    }

    m_leftEncoder.setDistancePerPulse(DrivetrainConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DrivetrainConstants.kDistancePerPulse);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_rightMotors.setInverted(true);

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
        -m_rightMotor.get() * RobotController.getInputVoltage());

    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_driveSim.getHeading().getDegrees());
  }

  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftMotors.setVoltage(leftOutput + leftFeedforward);
    m_rightMotors.setVoltage(rightOutput + rightFeedforward);
  }

  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_driveSim.setPose(pose);
    m_odometry.resetPosition(pose, m_ahrs.getRotation2d());
  }

  public Pose2d getCurrentPosition(){
    return m_odometry.getPoseMeters();
  }

  public void drive(double speed, double rotation) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation)));
  }

  public void stop(){
    this.m_leftMotors.stopMotor();
    this.m_rightMotors.stopMotor();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void arcadeDrive(double speed, double rotation) {
    double xSpeed = m_speedLimiter.calculate(speed) * DrivetrainConstants.kMaxSpeed;
    double rot = -m_rotLimiter.calculate(rotation) * DrivetrainConstants.kMaxAngularSpeed;
    drive(xSpeed, rot);
  }

  public DifferentialDriveKinematics getKinematics(){
    return this.m_kinematics;
  }
}
