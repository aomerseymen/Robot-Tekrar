/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final Drivetrain m_drive = new Drivetrain();

  private final Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drive.setDefaultCommand(
        (Command) new Joystick(m_drive, () -> -m_driverController.getRawAxis(1), () -> m_driverController.getRawAxis(0)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 1).whileHeld(new RunIntake(m_intake, 0.6));
    new JoystickButton(m_driverController, 1).whileHeld(new RunIntake(m_intake, -0.6));
    new JoystickButton(m_driverController, 2).whileHeld(new RunShooter(m_shooter, 0.6));
    new JoystickButton(m_driverController, 3).whileHeld(new RunHopper(m_hopper, 0.6));
    new JoystickButton(m_driverController, 3).whileHeld(new RunHopper(m_hopper, -0.6));
    

    // Turn off the shooter when the 'B' button is pressed   
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command trajectoryCommand()
  {
    // Create a voltage constraint to ensure we don't accelerate too fast
    final var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                       DrivetrainConstants.kvVoltSecondsPerMeter,
                                       DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
            10);
  
   final TrajectoryConfig config =
      new TrajectoryConfig(DrivetrainConstants.kMaxSpeedMetersPerSecond,
        DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DrivetrainConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    final TrajectoryConfig configReversed =
        new TrajectoryConfig(DrivetrainConstants.kMaxSpeedMetersPerSecond,
          DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DrivetrainConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

    configReversed.setReversed(true);

    final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, -1),
            new Translation2d(2, 0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, -1, new Rotation2d(0)),
        // Pass config
        config
    );

    final Object m_drive;
    final RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drive::getPose,
        new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                   DrivetrainConstants.kvVoltSecondsPerMeter,
                                   DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        DrivetrainConstants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(DrivetrainConstants.kPDriveVel, 0, 0, 0, null, null),
        new PIDController(DrivetrainConstants.kPDriveVel, 0, 0, 0, null, null),
        // RamseteCommand passes volts to the callback
        m_drive::arcadeDriveVolts,
        m_drive
    );

    return ramseteCommand.andThen( () -> ((Object) m_drive).arcadeDriveVolts(0, 0) );
  }
  public Command getAutonomousCommand() {
    final Command m_autoCommand;
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
