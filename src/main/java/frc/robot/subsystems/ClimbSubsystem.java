/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimbSubsystem.
   */
  private final Compressor compressor = new Compressor(ClimbConstants.compressorPin);
  private final DoubleSolenoid climbSolenoid = new DoubleSolenoid(ClimbConstants.solenoidForwardPin,
      ClimbConstants.solenoidBackwardPin);
  public ClimbSubsystem() {
    compressorDisable();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void compressorEnable(){
    compressor.setClosedLoopControl(true);
  }

  public void compressorDisable(){
    compressor.setClosedLoopControl(false);
  }
  public void climbUp(){
    climbSolenoid.set(Value.kForward);
  }
  public void climbDown(){
    climbSolenoid.set(Value.kReverse);
  }
  public void stopCylinders(){
    climbSolenoid.set(Value.kOff);
  }
}
