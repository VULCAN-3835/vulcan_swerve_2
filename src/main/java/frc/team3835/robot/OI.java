// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class OI {
    private static XboxController xboxController = new XboxController(Constants.OIConstants.BUTTON_XBOX_CONTROLLER_PORT); // Creates the button xbox controller
    private static XboxController elevatorXboxController = new XboxController(Constants.OIConstants.ELEVATOR_XBOX_CONTROLLER_PORT); // Creates the button xbox controller

    // Returns if X button is pressed in button controller
    public static boolean getXButtonPressed() {
        return xboxController.getXButtonPressed();
    }
    // Returns if A button is pressed in button controller
    public static boolean getAButtonPressed() {
        return xboxController.getAButtonPressed();
    }
    // Returns if Y button is pressed in button controller
    public static boolean getYButtonPressed() {
        return xboxController.getYButtonPressed();
    }
    // Returns if B button is pressed in button controller
    public static boolean getButtonPressed() {
        return xboxController.getBButtonPressed();
    }
    // Returns if Right Bumper is pressed in button controller
    public static boolean getRightBumperPressed() {
        return xboxController.getRightBumperPressed();
    }
    public static boolean getRightBumperReleased() {
        return xboxController.getRightBumperReleased();
    }
    public static boolean getRightBumper() {return xboxController.getRightBumper();}

    // Returns if Left Bumper is pressed in button controller
    public static boolean getLeftBumperPressed(){
        return xboxController.getLeftBumperPressed();
    }
    public static boolean getLeftBumper() {return xboxController.getLeftBumper();}

    public static boolean getBackButton() {
        return xboxController.getBackButton();
    }
    // Returns the value of the left joystick X axis in button controller
    public static double getLeftJoystickX() {
        return Math.abs(xboxController.getLeftX())> Constants.OIConstants.DEADZONE_JOYSTICK ? xboxController.getLeftX():0;
    }
    // Returns the value of the left joystick Y axis in button controller
    public static double getLeftJoystickY(){
        return Math.abs(xboxController.getLeftY())> Constants.OIConstants.DEADZONE_JOYSTICK ?-xboxController.getLeftY():0;
    }
    // Gets the value of the right joystick X axis in button controller
    public static double getRightJoystickX() {
        return Math.abs(xboxController.getRightX())> Constants.OIConstants.DEADZONE_JOYSTICK ? xboxController.getRightX():0;
    }
    // Gets the value of the right joystick Y axis in button controller
    public static double getRightJoystickY(){
        return Math.abs(xboxController.getRightY())> Constants.OIConstants.DEADZONE_JOYSTICK ?-xboxController.getRightY():0;
    }
    public static double getRightTrigger() {
        return Math.abs(xboxController.getRightTriggerAxis()) > Constants.OIConstants.DEADZONE_JOYSTICK ?xboxController.getRightTriggerAxis():0;
    }

    public static double getLeftTrigger() {
        return Math.abs(xboxController.getLeftTriggerAxis()) > Constants.OIConstants.DEADZONE_JOYSTICK ?xboxController.getLeftTriggerAxis():0;
    }

    public static double driveY() {
        return -getRightJoystickX();
    }
    public static double driveX() {
        return getRightJoystickY();
    }
    public static double driveRot() {
        return -getLeftJoystickX();
    }
}
