package frc.team3835.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3835.robot.subsystems.ChassisSubsystem;

public class StabilizeRamp extends CommandBase {
    ChassisSubsystem swerveChassisSubsystem;
    double SPEEDFORWARD = 0.18; //0.08
    double SPEEDBACKWARDS = -0.18; //-0.08
    double PITCH_DEADZONE = 5; //6
    boolean stabalized = false;
    int counter;
    public StabilizeRamp(ChassisSubsystem swerveChassisSubsystem) {
        this.swerveChassisSubsystem = swerveChassisSubsystem;

        addRequirements(swerveChassisSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pitch = (swerveChassisSubsystem.getPitch());
        SmartDashboard.putNumber("Pitch Degrees", pitch);

        if (pitch > PITCH_DEADZONE) {
            swerveChassisSubsystem.drive(SPEEDFORWARD,0,0, true);
            counter = 0;
        }
        else if (pitch < (-PITCH_DEADZONE)+1) {
            swerveChassisSubsystem.drive(SPEEDBACKWARDS,0,0, true);
            counter = 0;
        }
        else{
            swerveChassisSubsystem.drive(0,0,0.0001, true);
            counter++;
        }



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveChassisSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.counter>=30;
    }
}