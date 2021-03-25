/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_VictorSPX shooterMotor1 = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort1);
  private final WPI_VictorSPX shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort2);
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runShooter(double speed){
    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }
  public void stopShooter(){
    shooterMotor1.set(0);
    shooterMotor2.set(0);
  }
}
