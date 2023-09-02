// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.Constants.SwerveConstants;
import frc.team3835.robot.OI;
import frc.team3835.robot.commands.TeleopDriveCommand;

public class ChassisSubsystem extends SubsystemBase { 
  // An enum with the names of the wheel modules
  public enum wheels {
    left_front, right_front, right_back, left_back
  }

  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // Gyro 
  private AHRS imu;

  public ChassisSubsystem() { 
    this.swerve_modules[wheels.left_front.ordinal()] = new SwerveModule(Constants.ChassisConstants.LEFT_FRONT_DRIVE, // Instancing Left Front Wheel
            Constants.ChassisConstants.LEFT_FRONT_STEER,
            Constants.ChassisConstants.LEFT_FRONT_ENC,
            Constants.ChassisConstants.LEFT_FRONT_ZERO,
            Constants.ChassisConstants.LEFT_FRONT_INVERTED);

    this.swerve_modules[wheels.right_front.ordinal()] = new SwerveModule(Constants.ChassisConstants.RIGHT_FRONT_DRIVE, // Instancing Right Front Wheel
            Constants.ChassisConstants.RIGHT_FRONT_STEER, 
            Constants.ChassisConstants.RIGHT_FRONT_ENC, 
            Constants.ChassisConstants.RIGHT_FRONT_ZERO, 
            Constants.ChassisConstants.RIGHT_FRONT_INVERTED);

    this.swerve_modules[wheels.left_back.ordinal()] = new SwerveModule(Constants.ChassisConstants.LEFT_BACK_DRIVE, // Instancing Left Back Wheel
            Constants.ChassisConstants.LEFT_BACK_STEER,
            Constants.ChassisConstants.LEFT_BACK_ENC,
            Constants.ChassisConstants.LEFT_BACK_ZERO,
            Constants.ChassisConstants.LEFT_BACK_INVERTED);

    this.swerve_modules[wheels.right_back.ordinal()] = new SwerveModule(Constants.ChassisConstants.RIGHT_BACK_DRIVE, // Instancing Right Back Wheel
            Constants.ChassisConstants.RIGHT_BACK_STEER,
            Constants.ChassisConstants.RIGHT_BACK_ENC,
            Constants.ChassisConstants.RIGHT_BACK_ZERO,
            Constants.ChassisConstants.RIGHT_BACK_INVERTED);

    setDefaultCommand(new TeleopDriveCommand(this));
  }

 /**
     * Sets the module's state to given one
     *
     * @param chassisSpeeds The desired chassisSpeeds object for module velocities
     * @param fieldRelative  Is field relative or not
     */
  public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
    // Makes a swerve module-state array from chassisSpeeds
    var swerveModuleStates = Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative
      ?ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.imu.getRotation2d().unaryMinus())
      :chassisSpeeds
    );
    setModuleStates(swerveModuleStates);
  }

  /**
     * Sets the module's state to given one
     *
     * @param xVelocity  The velocity on the x axis
     * @param yVelocity  The velocity on the y axis
     * @param rot  The rotational velocity
     * @param fieldRelative  Is field relative or not
     */
  public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative) {
    // Kinematics turns the Chassis speeds to desired swerveModule states depending on if field relative or not
    var swerveModuleStates =
    Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rot, this.imu.getRotation2d().unaryMinus())
                : new ChassisSpeeds(xVelocity, yVelocity, rot));
    setModuleStates(swerveModuleStates);
  }
  

  /**
     * Sets the desired states of the modules to given ones
     *
     * @param desiredStates  Desired array of 4 module states
     */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets max acceleration and velocity to wheels
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ChassisConstants.kMaxSpeedMetersPerSecond);
    
    // Uses the set method of the SwerveModule to declare desired state of the module
    this.swerve_modules[wheels.left_front.ordinal()].set(desiredStates[0]);
    this.swerve_modules[wheels.right_front.ordinal()].set(desiredStates[1]);
    this.swerve_modules[wheels.left_back.ordinal()].set(desiredStates[2]);
    this.swerve_modules[wheels.right_back.ordinal()].set(desiredStates[3]);
  }

  /**
     * Stops all modules
     *
     */
  public void stopModules() {
      this.swerve_modules[wheels.left_front.ordinal()].stopMotors();
      this.swerve_modules[wheels.right_front.ordinal()].stopMotors();
      this.swerve_modules[wheels.left_back.ordinal()].stopMotors();
      this.swerve_modules[wheels.right_back.ordinal()].stopMotors();
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Left Front",this.swerve_modules[wheels.left_front.ordinal()].GetTrueAngle());
      SmartDashboard.putNumber("Left Back",this.swerve_modules[wheels.left_back.ordinal()].GetTrueAngle());
      SmartDashboard.putNumber("Right Front",this.swerve_modules[wheels.right_front.ordinal()].GetTrueAngle());
      SmartDashboard.putNumber("Right Back",this.swerve_modules[wheels.right_back.ordinal()].GetTrueAngle());


      SmartDashboard.putNumber("Left Joystick X", OI.getLeftJoystickX());
      SmartDashboard.putNumber("Left Joystick Y", OI.getLeftJoystickY());
      SmartDashboard.putNumber("Right Joystick X", OI.getRightJoystickX());
      SmartDashboard.putNumber("Right Joystick Y", OI.getRightJoystickY());

      SmartDashboard.putNumber("Left Front Velocity",this.swerve_modules[wheels.left_front.ordinal()].GetVel());
      SmartDashboard.putNumber("Left Back Velocity",this.swerve_modules[wheels.left_back.ordinal()].GetVel());
      SmartDashboard.putNumber("Right Front Velocity",this.swerve_modules[wheels.right_front.ordinal()].GetVel());
      SmartDashboard.putNumber("Right Back Velocity",this.swerve_modules[wheels.right_back.ordinal()].GetVel());
  }
}
