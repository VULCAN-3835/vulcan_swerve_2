package frc.team3835.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3835.robot.OI;
import frc.team3835.robot.subsystems.ElevatorSubsystem;


public class ElevatorDeafultCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public ElevatorDeafultCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
         this.elevatorSubsystem.setPower((OI.getRightTrigger()-OI.getLeftTrigger())*0.6);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
