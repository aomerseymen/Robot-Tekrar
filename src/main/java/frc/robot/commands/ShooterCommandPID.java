/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterCommandPID extends PIDCommand {
  /**
   * Creates a new ShooterCommandPID.
   */
  public ShooterCommandPID(double targetAngleDegrees, DrivetrainSubsystem drive) {
    super(
        new PIDController(DrivetrainConstants.kTurnP, DrivetrainConstants.kTurnI, Drivetrainonstants.kTurnD),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0, output),
        // Require the drive
        drive);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().enableContinuousInput(-180, 180);
    getController()
        .setTolerance(DrivetrainConstants.kTurnToleranceDeg, DrivetrainConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
