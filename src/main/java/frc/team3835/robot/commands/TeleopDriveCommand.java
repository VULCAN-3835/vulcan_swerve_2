// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3835.robot.OI;
import frc.team3835.robot.subsystems.ChassisSubsystem;

public class TeleopDriveCommand extends CommandBase {
  private ChassisSubsystem swerveDrive;

  public TeleopDriveCommand(ChassisSubsystem swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.swerveDrive.drive(OI.driveX(),OI.driveY(),OI.driveRot(),true);
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
