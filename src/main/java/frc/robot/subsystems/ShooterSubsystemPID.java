/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystemPID extends PIDSubsystem {
  private final PWMVictorSPX m_shooterMotor = new PWMVictorSPX(ShooterConstants.kShooterMotorPort);
  private final PWMVictorSPX m_feederMotor = new PWMVictorSPX(ShooterConstants.kFeederMotorPort);
  private final Encoder m_shooterEncoder =
      new Encoder(ShooterConstants.kEncoderPorts[0], ShooterConstants.kEncoderPorts[1],
                  ShooterConstants.kEncoderReversed);
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kSVolts,
                                 ShooterConstants.kVVoltSecondsPerRotation);
  /**
   * Creates a new PIDDenemeSubsystem.
   */
  public ShooterSubsystemPID() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }
}
}