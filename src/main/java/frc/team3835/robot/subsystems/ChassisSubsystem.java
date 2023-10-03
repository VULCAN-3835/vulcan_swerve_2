// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.Constants.SwerveConstants;
import frc.team3835.robot.commands.TeleopDriveCommand;
import frc.team3835.robot.OI;
import frc.team3835.robot.commands.TeleopDriveCommand;
import frc.team3835.robot.util.Conversions;

public class ChassisSubsystem extends SubsystemBase { 
  // An enum with the names of the wheel modules
  public enum wheels {
    left_front, right_front, right_back, left_back
  }
  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // Gyro 
  private AHRS imu;

  //
  private SwerveDriveOdometry odometry;

  private Pose2d robotPose;

  private Field2d field;

  private SwerveModulePosition[] swerve_module_positions = new SwerveModulePosition[4];

  public ChassisSubsystem(IntakeSubsystem intakeSubsystem) {

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

    this.field = new Field2d();

    this.imu = new AHRS();

    updateModuePositions();

    this.odometry = new SwerveDriveOdometry(Constants.ChassisConstants.kDriveKinematics, this.imu.getRotation2d(), swerve_module_positions);

    this.robotPose = this.odometry.getPoseMeters();

    setDefaultCommand(new TeleopDriveCommand(this, intakeSubsystem));
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rot, Rotation2d.fromDegrees(normalizeAngleDegrees(this.imu.getRotation2d().getDegrees())))
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
  public void resetImu() {
      this.imu.reset();
  }
  public double getPitch() {
      return this.imu.getPitch();
  }

  private Pose2d getPose2d() {
      return this.robotPose;
  }
  private void resetOdometry(Pose2d pose) {
      this.robotPose = pose;
  }

  private void updateModuePositions() {
      this.swerve_module_positions[wheels.left_front.ordinal()] = this.swerve_modules[wheels.left_front.ordinal()].GetModulePosition();
      this.swerve_module_positions[wheels.right_front.ordinal()] = this.swerve_modules[wheels.right_front.ordinal()].GetModulePosition();
      this.swerve_module_positions[wheels.right_back.ordinal()] = this.swerve_modules[wheels.right_back.ordinal()].GetModulePosition();
      this.swerve_module_positions[wheels.left_back.ordinal()] = this.swerve_modules[wheels.left_back.ordinal()].GetModulePosition();
  }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if(isFirstPath){
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose2d, // Pose supplier
                        Constants.ChassisConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        this // Requires this drive subsystem
                )
        );
    }
    public double normalizeAngleDegrees(double angleDegrees) {
        while (angleDegrees > 180) {
            angleDegrees -= 360;
        }
        while (angleDegrees < -180) {
            angleDegrees += 360;
        }
        return angleDegrees;
    }

  @Override
  public void periodic() {
      updateModuePositions();
      this.odometry.update(this.imu.getRotation2d(), this.swerve_module_positions);

      this.robotPose = this.odometry.getPoseMeters();
      field.setRobotPose(this.robotPose);

      SmartDashboard.putData(field);

      SmartDashboard.putNumber("Gyro Yaw", this.imu.getAngle());
      SmartDashboard.putNumber("GYRO", normalizeAngleDegrees(this.imu.getRotation2d().getDegrees()));

      SmartDashboard.putNumber("Left Front Distance Meters",this.swerve_modules[wheels.left_front.ordinal()].GetDistanceMeters());
      SmartDashboard.putNumber("Left Back Distance Meters",this.swerve_modules[wheels.left_back.ordinal()].GetDistanceMeters());
      SmartDashboard.putNumber("Right Front Distance Meters",this.swerve_modules[wheels.right_front.ordinal()].GetDistanceMeters());
      SmartDashboard.putNumber("Right Back Distance Meters",this.swerve_modules[wheels.right_back.ordinal()].GetDistanceMeters());

      SmartDashboard.putNumber("Left Front",this.swerve_modules[wheels.left_front.ordinal()].GetTrueAngle());
      SmartDashboard.putNumber("Left Back",this.swerve_modules[wheels.left_back.ordinal()].GetTrueAngle());
      SmartDashboard.putNumber("Right Front",this.swerve_modules[wheels.right_front.ordinal()].GetTrueAngle());
      SmartDashboard.putNumber("Right Back",this.swerve_modules[wheels.right_back.ordinal()].GetTrueAngle());

      SmartDashboard.putNumber("Left Front Error",this.swerve_modules[wheels.left_front.ordinal()].GetTargetError());
      SmartDashboard.putNumber("Left Back Error",this.swerve_modules[wheels.left_back.ordinal()].GetTargetError());
      SmartDashboard.putNumber("Right Front Error",this.swerve_modules[wheels.right_front.ordinal()].GetTargetError());
      SmartDashboard.putNumber("Right Back Error",this.swerve_modules[wheels.right_back.ordinal()].GetTargetError());

      SmartDashboard.putNumber("Left Joystick X", OI.getLeftJoystickX());
      SmartDashboard.putNumber("Left Joystick Y", OI.getLeftJoystickY());
      SmartDashboard.putNumber("Right Joystick X", OI.getRightJoystickX());
      SmartDashboard.putNumber("Right Joystick Y", OI.getRightJoystickY());

      SmartDashboard.putNumber("Pitch", this.imu.getPitch());

      SmartDashboard.putNumber("Left Front Velocity",this.swerve_modules[wheels.left_front.ordinal()].GetVel());
      SmartDashboard.putNumber("Left Back Velocity",this.swerve_modules[wheels.left_back.ordinal()].GetVel());
      SmartDashboard.putNumber("Right Front Velocity",this.swerve_modules[wheels.right_front.ordinal()].GetVel());
      SmartDashboard.putNumber("Right Back Velocity",this.swerve_modules[wheels.right_back.ordinal()].GetVel());
      SmartDashboard.putNumber("Velocity Average", Conversions.falconToMPS(
              (Math.abs(this.swerve_modules[wheels.left_front.ordinal()].GetVel())+
              Math.abs(this.swerve_modules[wheels.left_back.ordinal()].GetVel())+
              Math.abs(this.swerve_modules[wheels.right_front.ordinal()].GetVel())+
              Math.abs(this.swerve_modules[wheels.right_back.ordinal()].GetVel()))/4,
              Units.inchesToMeters(4*Math.PI),
              6.75
              ));
  }
}
