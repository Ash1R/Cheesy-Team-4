/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;
import frc.robot.controls.Driver;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  
  private final WPI_TalonFX m_rightMotor1;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.auto.S, Constants.auto.V, Constants.auto.A);
  private final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(0.62865);
  private final RamseteController m_ramseteController = new RamseteController(Constants.auto.kRamseteB, Constants.auto.kRamseteZeta);
  private final PIDController m_leftPID = new PIDController(Constants.auto.P, Constants.auto.I, Constants.auto.D);
  private final PIDController m_rightPID = new PIDController(Constants.auto.P, Constants.auto.I, Constants.auto.D);

  public Drivetrain() {
    m_leftMotor1 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor1);
    m_rightMotor1 = MotorFactory.createTalonFX(Constants.drive.kRightMotor1);
    m_rightMotor1.setInverted(true);
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

  public double getEncoderPosition() {
    return m_leftMotor1.getSelectedSensorPosition();
  }

  public void setEncoderPosition() {
    m_leftMotor1.setSelectedSensorPosition(0);
  }
}
