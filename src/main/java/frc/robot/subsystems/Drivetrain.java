/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new DrivetrainSubsystem.
   */
  private final WPI_VictorSPX m_frontRightMotor = new WPI_VictorSPX(DrivetrainConstants.kFrontRightMotorPort);
  private final WPI_VictorSPX m_frontLeftMotor = new WPI_VictorSPX(DrivetrainConstants.kFrontLeftMotorPort);
  private final WPI_VictorSPX m_rearRightMotor = new WPI_VictorSPX(DrivetrainConstants.kRearRightMotorPort);
  private final WPI_VictorSPX m_rearLeftMotor = new WPI_VictorSPX(DrivetrainConstants.kRearLeftMotorPort);
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_rearLeftMotor, m_rearRightMotor);
  
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final Encoder leftWheelEncoder =  new Encoder(DrivetrainConstants.leftEncoderPort1,DrivetrainConstants.leftEncoderPort2);
  private final Encoder rightWheelEncoder =  new Encoder(DrivetrainConstants.rightEncoderPort1,DrivetrainConstants.rightEncoderPort2);

  public Drivetrain() {
    m_frontRightMotor.follow(m_rearRightMotor);
    m_frontLeftMotor.follow(m_rearLeftMotor);
    
    gyro.calibrate();
    leftWheelEncoder.setDistancePerPulse(15.24*Math.PI/2048);
    rightWheelEncoder.setDistancePerPulse(15.24*Math.PI/2048);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getDistance(){
    return (leftWheelEncoder.getDistance()+rightWheelEncoder.getDistance())/2;
  }

  public void joystickDrive(DoubleSupplier linearSpeed, DoubleSupplier angularSpeed){
    m_drive.arcadeDrive(linearSpeed.getAsDouble(), angularSpeed.getAsDouble());
  }
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
}
