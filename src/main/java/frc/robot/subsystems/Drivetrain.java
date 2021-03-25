/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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

  public Drivetrain() {
    m_frontRightMotor.follow(m_rearRightMotor);
    m_frontLeftMotor.follow(m_rearLeftMotor);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void joystickDrive(DoubleSupplier linearSpeed, DoubleSupplier angularSpeed){
    m_drive.arcadeDrive(linearSpeed.getAsDouble(), angularSpeed.getAsDouble());
  }
}
