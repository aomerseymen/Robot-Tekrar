/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class IntakeConstants{
        public static final int kIntakeMotorPort = 0;
        
    }
    public final class ShooterConstants{
        public static final int kShooterMotorPort1 = 1;
        public static final int kShooterMotorPort2= 2;
        
    }

    public final class HopperConstants{
        public static final int frontRightMotorPort = 3;
        public static final int rearLeftMotorPort = 4;
        
    }
    public final class DrivetrainConstants{
        public static final int kFrontRightMotorPort = 5;
        public static final int kFrontLeftMotorPort = 6;
        public static final int kRearRightMotorPort = 7;
        public static final int kRearLeftMotorPort = 8;

        public static final int leftEncoderPort1 = 1;
        public static final int leftEncoderPort2 = 2;
        public static final int rightEncoderPort1 = 3;
        public static final int rightEncoderPort2 = 4;

        public static final double turnP = 1.0;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;

        public static final double kTurnToleranceDeg = 1.0;
        public static final double kTurnRateToleranceDegPerS = 1.0;

        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
            

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;


        
    }

    public final class ClimbConstants{
        public static final int solenoidForwardPin = 9;
        public static final int solenoidBackwardPin = 10;
        public static final int compressorPin = 11;

    }

    public final class JoystickConstants{
        public static final int kDriverControllerPort = 1;
    }
    
}
