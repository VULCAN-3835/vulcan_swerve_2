// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants.ElevatorConstants;
import frc.team3835.robot.commands.ElevatorDeafultCommand;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor;

  private DigitalInput limitSwitch;

  private final int PID_SLOT = 0;
  private final int TIMEOUT_MS = 30;

  private final double maxVelocity = 0; // TODO: Find max velocity
  private final double maxAcceleration = 0; // Todo: Find max Acceleration

  
  public ElevatorSubsystem() {
    this.elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR);

    this.elevatorMotor.setNeutralMode(NeutralMode.Brake);

    this.elevatorMotor.configFactoryDefault();
    this.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_SLOT, TIMEOUT_MS);

    this.elevatorMotor.configMotionCruiseVelocity((int) maxVelocity);
    this.elevatorMotor.configMotionAcceleration((int) maxAcceleration);

    this.elevatorMotor.config_kP(PID_SLOT, 0); // TODO: Find pid values
    this.elevatorMotor.config_kI(PID_SLOT, 0);
    this.elevatorMotor.config_kD(PID_SLOT, 0);
    this.elevatorMotor.config_kF(PID_SLOT, 0);

    setDefaultCommand(new ElevatorDeafultCommand(this));
  }

  public void setArmPosition(double position) {
    this.elevatorMotor.set(TalonFXControlMode.MotionMagic, position);
  }
  public void setPower(double power) {
    this.elevatorMotor.set(TalonFXControlMode.PercentOutput, power);
  }
  public void setIntakePower(double power) {

  }
  public void zeroMotors() {
    this.elevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  public double getCurrentPosition () {
    return this.elevatorMotor.getSelectedSensorPosition();
  }
  public double getCurrentTargetPosition() {
    return this.elevatorMotor.getClosedLoopTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
