// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.OI;
import frc.team3835.robot.RobotContainer;
import frc.team3835.robot.subsystems.ChassisSubsystem;
import frc.team3835.robot.subsystems.IntakeSubsystem;

public class ShootingDriveCommand extends CommandBase {
    private ChassisSubsystem swerveDrive;
    private IntakeSubsystem intakeSubsystem;
    private PIDController pidController;
    private double rotTolerance = 1.5;


    public ShootingDriveCommand(ChassisSubsystem swerveDrive, IntakeSubsystem intakeSubsystem) {
        this.swerveDrive = swerveDrive;
        this.intakeSubsystem = intakeSubsystem;

        pidController = new PIDController(0.005,0,0);
        pidController.setTolerance(rotTolerance);
        pidController.setSetpoint(180);
        pidController.enableContinuousInput(-180,180);

        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double normal = this.intakeSubsystem.isClosed() ? Constants.SwerveConstants.maxDrivingVelocity : Constants.SwerveConstants.maxPlacingVelocity;

        double rot = -pidController.calculate(this.swerveDrive.getHeading());

        this.swerveDrive.drive(OI.driveX()*normal,OI.driveY()*normal,rot,true);
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
