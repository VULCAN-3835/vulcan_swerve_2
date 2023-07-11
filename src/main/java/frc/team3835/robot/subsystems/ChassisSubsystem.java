// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.Constants.SwerveConstants;

public class ChassisSubsystem extends SubsystemBase {
  // An enum with the names of the wheel modules
  public enum wheels {
    left_front, right_front,
    right_back, left_back
  }

  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // Gyro 
  private AHRS imu;

  public ChassisSubsystem() {
    this.swerve_modules[wheels.left_front.ordinal()] = new SwerveModule(Constants.ChassisConstants.LEFT_FRONT_DRIVE, Constants.ChassisConstants.LEFT_FRONT_STEER, Constants.ChassisConstants.LEFT_FRONT_ENC, Constants.ChassisConstants.LEFT_FRONT_ZERO, Constants.ChassisConstants.LEFT_FRONT_INVERTED);
    this.swerve_modules[wheels.right_front.ordinal()] = new SwerveModule(Constants.ChassisConstants.RIGHT_FRONT_DRIVE, Constants.ChassisConstants.RIGHT_FRONT_STEER, Constants.ChassisConstants.RIGHT_FRONT_ENC, Constants.ChassisConstants.RIGHT_FRONT_ZERO, Constants.ChassisConstants.RIGHT_FRONT_INVERTED);
    this.swerve_modules[wheels.left_back.ordinal()] = new SwerveModule(Constants.ChassisConstants.LEFT_BACK_DRIVE, Constants.ChassisConstants.LEFT_BACK_STEER, Constants.ChassisConstants.LEFT_BACK_ENC, Constants.ChassisConstants.LEFT_BACK_ZERO, Constants.ChassisConstants.LEFT_BACK_INVERTED);
    this.swerve_modules[wheels.right_back.ordinal()] = new SwerveModule(Constants.ChassisConstants.RIGHT_BACK_DRIVE, Constants.ChassisConstants.RIGHT_BACK_STEER, Constants.ChassisConstants.RIGHT_BACK_ENC, Constants.ChassisConstants.RIGHT_BACK_ZERO, Constants.ChassisConstants.RIGHT_BACK_INVERTED);

    setDefaultCommand(null); // TODO: Default Command
  }

  // Gets a chassisSpeeds and boolean to set the module states
  public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
    // makes a swerve module-state array from chassisSpeeds
    var swerveModuleStates = Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative
      ?ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.imu.getRotation2d().unaryMinus())
      :chassisSpeeds
    );
    setModuleStates(swerveModuleStates);
  }

  // Gets an xVelocity a yVelocity a rotation value and a boolean to set the module states
  public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative) {
    // Kinematics turns the Chassis speeds to desired swerveModule states depending on if field relative or not
    var swerveModuleStates =
    Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rot, this.imu.getRotation2d().unaryMinus())
                : new ChassisSpeeds(xVelocity, yVelocity, rot));
    setModuleStates(swerveModuleStates);
  }
  
  // Gets an array of 4 module states to set as the desired states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets max acceleration and velocity to wheels
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ChassisConstants.kMaxSpeedMetersPerSecond);
    
    this.swerve_modules[wheels.left_front.ordinal()].set(desiredStates[0]);
    this.swerve_modules[wheels.right_front.ordinal()].set(desiredStates[1]);
    this.swerve_modules[wheels.left_back.ordinal()].set(desiredStates[2]);
    this.swerve_modules[wheels.right_back.ordinal()].set(desiredStates[3]);
  }

  // Stop all motors of the modules
  public void stopModules() {
      this.swerve_modules[wheels.left_front.ordinal()].stopMotors();
      this.swerve_modules[wheels.right_front.ordinal()].stopMotors();
      this.swerve_modules[wheels.left_back.ordinal()].stopMotors();
      this.swerve_modules[wheels.right_back.ordinal()].stopMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
