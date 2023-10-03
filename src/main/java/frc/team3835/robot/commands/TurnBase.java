package frc.team3835.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3835.robot.subsystems.ChassisSubsystem;

public class TurnBase extends PIDCommand {
    private double rotTolerance = 1.5;
    public TurnBase(ChassisSubsystem swerveChassisSubsystem) {
        super(
                // The controller that the command will use
                new PIDController(0.005,0,0),
                // This should return the measurement
                () -> swerveChassisSubsystem.getHeading(),
                // This should return the setpoint (can also be a constant)
                () -> 180,
                // This uses the output
                output -> {
                    swerveChassisSubsystem.drive(0,0,-output,true);
                });
        getController().setTolerance(this.rotTolerance);
        addRequirements(swerveChassisSubsystem);
        // Configure additional PID options by calling `getController` here.
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
//        return getController().atSetpoint();
    }
}
