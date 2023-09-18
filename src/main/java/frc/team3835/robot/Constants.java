// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OIConstants {
        public static final int BUTTON_XBOX_CONTROLLER_PORT = 0;
        public static final double DEADZONE_JOYSTICK = 0.1;
    }
    public static class SwerveConstants
    {
        public static final double pulseToMeterFalcon = 43310.195537448053576162384741371; // TODO: Fact check
        public static final double ticksPerDegree = 70.9; // TODO: Fact Check

        public static final double steerP = 0.1; // TODO: Find appropriate pid values, (Previous: 0.125)
        public static final double steerI = 0;
        public static final double steerD = 0;

        public static final double driveP = 0.02; // TODO: Find appropriate pid values, (Previous: 0.14)
        public static final double driveI = 0;
        public static final double driveD = 0;

        public static final double feedForwardKs = 0.4704 / 12;
        public static final double feedForwardKv = 2.3615 / 12;

        public static final double maxAcceleration = 1.5;

        public static final double steerMotorThreshold = 0.04; // TODO: Check minimal power required to move motor
    }
    public static class ChassisConstants {
        public static final int LEFT_FRONT_DRIVE = 12; // CAN ID
        public static final int RIGHT_FRONT_DRIVE = 10; // CAN ID
        public static final int LEFT_BACK_DRIVE = 13; // CAN ID
        public static final int RIGHT_BACK_DRIVE = 11; // CAN ID
        // Ports for angle motors
        public static final int LEFT_FRONT_STEER = 22; // CAN ID
        public static final int RIGHT_FRONT_STEER = 20; // CAN ID
        public static final int LEFT_BACK_STEER = 23; // CAN ID
        public static final int RIGHT_BACK_STEER = 21; // CAN ID
        // Ports for encoders
        public static final int LEFT_FRONT_ENC = 32; // CAN ID
        public static final int RIGHT_FRONT_ENC = 30; // CAN ID
        public static final int LEFT_BACK_ENC = 33; // CAN ID
        public static final int RIGHT_BACK_ENC = 31; // CAN ID
        // Offsets for absolute encoders:
        public static final double LEFT_FRONT_ZERO = 5.71;
        public static final double RIGHT_FRONT_ZERO = -63.63;
        public static final double LEFT_BACK_ZERO = -127.08;
        public static final double RIGHT_BACK_ZERO =  76.9;
        // Which motors are inverted                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        public static final boolean frontLeftDriveInverted = true;
        public static final boolean LEFT_FRONT_INVERTED = false;
        public static final boolean RIGHT_FRONT_INVERTED = false;
        public static final boolean LEFT_BACK_INVERTED = false;
        public static final boolean RIGHT_BACK_INVERTED = false;

        // Max speed in meters per second
        public static final double kMaxSpeedMetersPerSecond = 4;
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.35*2.54;
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.35*2.54;

        // Swerve Kinematics
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Left front
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //Right front
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Left back
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //Right back
    }
    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR = 40; // CAN ID
    }
}
