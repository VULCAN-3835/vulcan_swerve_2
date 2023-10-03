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
    private static Joystick rightJoystick = new Joystick(3);
    private static Joystick leftJoystick = new Joystick(2);

    public static double getRightX() {
        return Math.abs(rightJoystick.getX())> 0.15 ?normalizeInput(rightJoystick.getX()):0;
    }
    public static double getRightY() {
        return Math.abs(rightJoystick.getY())> 0.15 ?normalizeInput(rightJoystick.getY()):0;
    }
    public static double getLeftX() {
        return Math.abs(leftJoystick.getX())> 0.15 ?normalizeInput(leftJoystick.getX()):0;
    }
    public static double getLeftY() {
        return Math.abs(leftJoystick.getY())> 0.15 ?normalizeInput(leftJoystick.getY()):0;
    }


    public static double getLeftJoystickX() {
        return Math.abs(xboxController.getLeftX())> Constants.OIConstants.DEADZONE_JOYSTICK ? normalizeInput(xboxController.getLeftX()):0;
    }
    // Returns the value of the left joystick Y axis in button controller
    public static double getLeftJoystickY(){
        return Math.abs(xboxController.getLeftY())> Constants.OIConstants.DEADZONE_JOYSTICK ?-normalizeInput(xboxController.getLeftY()):0;
    }
    // Gets the value of the right joystick X axis in button controller
    public static double getRightJoystickX() {
         return Math.abs(xboxController.getRightX())> Constants.OIConstants.DEADZONE_JOYSTICK ? normalizeInput(xboxController.getRightX()):0;
    }
    // Gets the value of the right joystick Y axis in button controller
    public static double getRightJoystickY(){
         return Math.abs(xboxController.getRightY())> Constants.OIConstants.DEADZONE_JOYSTICK ?-normalizeInput(xboxController.getRightX()):0;
    }
     public static double getRightTrigger() {
         return Math.abs(xboxController.getRightTriggerAxis()) > Constants.OIConstants.DEADZONE_JOYSTICK ?xboxController.getRightTriggerAxis():0;
     }
     public static double getLeftTrigger() {
         return Math.abs(xboxController.getLeftTriggerAxis()) > Constants.OIConstants.DEADZONE_JOYSTICK ?xboxController.getLeftTriggerAxis():0;
     }
     public static boolean getLeftBumper() {
        return xboxController.getLeftBumper();
     }
     public static boolean getRightBumper() {
        return xboxController.getRightBumper();
     }
     public static boolean getAButtonPressed() {
        return xboxController.getAButtonPressed();
     }
     public static boolean getAButtonReleased() {
        return xboxController.getAButtonReleased();
     }
     public static boolean getBButtonPressed() {
        return xboxController.getBButtonPressed();
     }
     public static boolean getBButtonReleased() {
        return xboxController.getBButtonReleased();
     }
     public static boolean getYButtonPressed() {
        return xboxController.getYButtonPressed();
     }
    public static boolean getYButtonReleased() {
        return xboxController.getYButtonReleased();
    }
    public static boolean getXButtonPressed(){
        return xboxController.getXButtonPressed();
    }
    public static boolean getXButtonReleased() {
        return xboxController.getXButtonReleased();
    }

    public static boolean getBackButtonPressed() {
        return xboxController.getBackButtonPressed();
    }
    public static boolean getStartButtonPressed() {
        return xboxController.getStartButtonPressed();
    }



    public static double driveY() {
        if (xboxController.isConnected()) {
            return -getLeftJoystickX();
        }
        return -getRightX();
    }

    public static double driveX() {
        if (xboxController.isConnected()) {
            return getLeftJoystickY();
        }
            return -getRightY();
    }
    public static double driveRot() {
        if (xboxController.isConnected()) {
            return -getRightJoystickX();
        }
        return -getLeftX();
    }

    public static double normalizeInput(double input) {
        return Math.pow(input,3)/Math.abs(input);
    }
}
