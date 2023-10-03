package frc.team3835.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.*;
import frc.team3835.robot.Constants;
import frc.team3835.robot.commands.StabilizeRamp;
import frc.team3835.robot.subsystems.ChassisSubsystem;
import frc.team3835.robot.subsystems.ElevatorSubsystem;
import frc.team3835.robot.subsystems.IntakeSubsystem;

public class AutonomousCubeTaxiStabilize extends SequentialCommandGroup {
    ChassisSubsystem chassisSubsystem;
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;


    // Drives forward at a velocity of 0.6 for 3 second and then stops the modules. Then stabilizes the ramp and for 0.1
    // seconds rotates all the wheels 45 degrees
    public AutonomousCubeTaxiStabilize(ChassisSubsystem chassisSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.chassisSubsystem = chassisSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(this.chassisSubsystem);
        addCommands(new InstantCommand(() -> {
                    this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Score Mid")[1]);
                    this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Score Mid")[0]);
                }),
                new WaitCommand(3.5),
                new InstantCommand(() -> {
                    this.intakeSubsystem.setIntakePower(-Constants.ElevatorConstants.INTAKE_POWER);
                }),
                new WaitCommand(0.4),
                new InstantCommand(() -> {
                    this.intakeSubsystem.setIntakePower(0);
                    this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Default")[1]);
                    this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Default")[0]);
                }),
                new WaitCommand(0),
                new AutonomousMoveOutStabilize(this.chassisSubsystem)
        );
    }
}