// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3835.robot.subsystems.ChassisSubsystem;
import frc.team3835.robot.subsystems.ElevatorSubsystem;
import frc.team3835.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private ChassisSubsystem chassisSubsystem = new ChassisSubsystem(intakeSubsystem);

    public RobotContainer()
    {
        Constants.ElevatorConstants.positionMap.put("Default", new double[] {0,155}); // Elevator Closed, Angle Closed
        Constants.ElevatorConstants.positionMap.put("Collect Cone", new double[] {0,28});
        Constants.ElevatorConstants.positionMap.put("Collect Cube", new double[] {25,50});
        Constants.ElevatorConstants.positionMap.put("Score Low", new double[] {0,0});
        Constants.ElevatorConstants.positionMap.put("Score Mid", new double[] {28.5,0});
        // Configure the trigger bindings
        configureBindings();
    }

    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        Trigger triggerAPressed = new Trigger(OI::getAButtonPressed);
        Trigger triggerAReleased = new Trigger(OI::getAButtonReleased);

        Trigger triggerBPressed = new Trigger(OI::getBButtonPressed);
        Trigger triggerBReleased = new Trigger(OI::getBButtonReleased);

        Trigger triggerYPressed = new Trigger(OI::getYButtonPressed);
        Trigger triggerYReleased = new Trigger(OI::getYButtonReleased);

        Trigger triggerXPressed = new Trigger(OI::getXButtonPressed);
        Trigger triggerXReleased = new Trigger(OI::getXButtonReleased);

        triggerAPressed.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Score Low")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Score Low")[0]);
        }));
        triggerAReleased.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Default")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Default")[0]);
        }));

        triggerBPressed.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Score Mid")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Score Mid")[0]);
        }));
        triggerBReleased.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Default")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Default")[0]);
        }));

        triggerYPressed.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Collect Cone")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Collect Cone")[0]);
            this.intakeSubsystem.setIntakePower(-Constants.ElevatorConstants.INTAKE_POWER);
        }));
        triggerYReleased.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Default")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Default")[0]);
            this.intakeSubsystem.setIntakePower(0);
        }));

        triggerXPressed.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Collect Cube")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Collect Cube")[0]);
            this.intakeSubsystem.setIntakePower(Constants.ElevatorConstants.INTAKE_POWER*0.4);
        }));
        triggerXReleased.onTrue(new InstantCommand(() -> {
            this.intakeSubsystem.setAxisPosition(Constants.ElevatorConstants.positionMap.get("Default")[1]);
            this.elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.positionMap.get("Default")[0]);
            this.intakeSubsystem.setIntakePower(0);
        }));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous

        return null;
//        return new ParallelRaceGroup(
//                new WaitCommand(3),
//                new StartEndCommand(
//                        () -> this.chassisSubsystem.drive(0.5,0,0,false),
//                        () -> this.chassisSubsystem.drive(0,0,0,false),
//                        this.chassisSubsystem)
//        );
    }
}
