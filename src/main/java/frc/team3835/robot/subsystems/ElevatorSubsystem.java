// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor;
  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR);
    elevatorMotor.configFactoryDefault();
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
