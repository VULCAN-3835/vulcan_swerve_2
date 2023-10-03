// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.OI;
import frc.team3835.robot.RobotContainer;
import frc.team3835.robot.subsystems.ChassisSubsystem;
import frc.team3835.robot.subsystems.IntakeSubsystem;

public class TeleopDriveCommand extends CommandBase {
  private ChassisSubsystem swerveDrive;
  private IntakeSubsystem intakeSubsystem;

  public TeleopDriveCommand(ChassisSubsystem swerveDrive, IntakeSubsystem intakeSubsystem) {
    this.swerveDrive = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double normal = this.intakeSubsystem.isClosed() ? Constants.SwerveConstants.maxDrivingVelocity : Constants.SwerveConstants.maxPlacingVelocity;
    this.swerveDrive.drive(OI.driveX()*normal,OI.driveY()*normal,OI.driveRot()*normal,true);
    SmartDashboard.putNumber("X Joystick", OI.driveX()*normal);
    SmartDashboard.putNumber("Y Joystick", OI.driveY()*normal);
    SmartDashboard.putNumber("Rot Joystick", OI.driveRot()*normal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
