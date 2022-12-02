/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.util.MotorFactory;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  
  private final WPI_TalonFX m_rightMotor1;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.auto.S, Constants.auto.V, Constants.auto.A);
  private final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(0.62865);
  private final RamseteController m_ramseteController = new RamseteController(Constants.auto.kRamseteB, Constants.auto.kRamseteZeta);
  private final PIDController m_leftPID = new PIDController(Constants.auto.P, Constants.auto.I, Constants.auto.D);
  private final PIDController m_rightPID = new PIDController(Constants.auto.P, Constants.auto.I, Constants.auto.D);
  public Field2d m_field = new Field2d();
  private final DifferentialDriveOdometry m_odometry;
  private final AHRS m_navX;
  private final 

  public Drivetrain() {
    m_leftMotor1 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor1);
    m_rightMotor1 = MotorFactory.createTalonFX(Constants.drive.kRightMotor1);
    m_rightMotor1.setInverted(true);
    m_leftMotor1.set

    m_navX = new AHRS(SPI.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d());
  }

  /**
   * Drives the robot using tank drive controls
   * Tank drive is slightly easier to code but less intuitive to control, so this is here as an example for now
   * @param leftPower the commanded power to the left motors
   * @param rightPower the commanded power to the right motors
   */
  public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor1.set(ControlMode.PercentOutput, -leftPower);
    m_rightMotor1.set(ControlMode.PercentOutput, -rightPower);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn the commanded turn rotation
   */
  public void arcadeDrive(double throttle, double turn) {
    double speed = Driver.getLeftTrigger()>0.5?1:0.5;
    m_leftMotor1.set(ControlMode.PercentOutput, speed*(throttle - turn));
    m_rightMotor1.set(ControlMode.PercentOutput, speed*(throttle + turn));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putData("Drivetrain", m_dDrive);
    // Update the odometry in the periodic block
    m_field.setRobotPose(getPose());
    updateOdometry();
  }

  public void updateOdometry() {
    m_odometry.update(m_navX.getRotation2d(), getLeftEncoder(), getRightEncoder());
  }

  public double getLeftEncoder() {
    return m_leftMotor1.getSelectedSensorPosition();
  }
  public double getRightEncoder() {
    return m_rightMotor1.getSelectedSensorPosition();
  }
  public double getLeftEncoderSpeed() {
    return m_leftMotor1.getSelectedSensorVelocity();
  }
  public double getRightEncoderSpeed() {
    return m_rightMotor1.getSelectedSensorVelocity();
  }

  public void resetEncoders() {
    m_leftMotor1.setSelectedSensorPosition(0);
    m_rightMotor1.setSelectedSensorPosition(0);
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //zeroHeading();
    m_odometry.resetPosition(pose, m_navX.getRotation2d());
  }

  public void tankDriveVolts(double left, double right){
    m_leftMotor1.setVoltage(left);
    m_rightMotor1.setVoltage(right);
  }

  public RamseteController getRamseteController() {
    return m_ramseteController;
  }
  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }
  public DifferentialDriveKinematics getDriveKinematics() {
    return m_driveKinematics;
  }
  public PIDController getLeftPID(){
    return m_leftPID;
  }
  public PIDController getRightPID(){
    return m_rightPID;
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
  }
}
