package frc.team3835.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.*;
import frc.team3835.robot.commands.StabilizeRamp;
import frc.team3835.robot.subsystems.ChassisSubsystem;

public class AutonomousMoveOutStabilize extends SequentialCommandGroup {
    ChassisSubsystem chassisSubsystem;

    // Drives forward at a velocity of 0.6 for 3 second and then stops the modules. Then stabilizes the ramp and for 0.1
    // seconds rotates all the wheels 45 degrees
    public AutonomousMoveOutStabilize(ChassisSubsystem chassisSubsystem) {
        this.chassisSubsystem = chassisSubsystem;

        addRequirements(this.chassisSubsystem);
//        addCommands(new ParallelRaceGroup(new WaitCommand(2.65),
//                        new StartEndCommand(() -> this.chassisSubsystem.drive(-1.15  ,-0,-0, true),
//                                () -> this.chassisSubsystem.stopModules(), this.chassisSubsystem)),
//                new StabilizeRamp(chassisSubsystem),
//                new ParallelRaceGroup(new WaitCommand(0.1),
//                        new StartEndCommand(() -> this.chassisSubsystem.drive(0  ,0,0.05, true),
//                                () -> this.chassisSubsystem.stopModules(), this.chassisSubsystem)));
        addCommands(new InstantCommand(() -> this.chassisSubsystem.drive(1.2  ,-0,-0, true)),
                new WaitCommand(2.9),
                new InstantCommand(() -> this.chassisSubsystem.drive(0  ,-0,-0, true)),
                new InstantCommand(() -> this.chassisSubsystem.drive(0.4  ,-0,-0, true)),
                new WaitCommand(3.1),
                new InstantCommand(() -> this.chassisSubsystem.drive(0  ,-0,-0, true)),
                new WaitCommand(0.2),
                new InstantCommand(() -> this.chassisSubsystem.drive(-1.15  ,-0,-0, true)),
                new WaitCommand(2.35),
                new InstantCommand(() -> this.chassisSubsystem.drive(0  ,-0,-0, true)),

                new StabilizeRamp(chassisSubsystem),
                new ParallelRaceGroup(new WaitCommand(0.1),
                        new StartEndCommand(() -> this.chassisSubsystem.drive(0  ,0,0.005, true),
                                () -> this.chassisSubsystem.stopModules(), this.chassisSubsystem)));
    }
}