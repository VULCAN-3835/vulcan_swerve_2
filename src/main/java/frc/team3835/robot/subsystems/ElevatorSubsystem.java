// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxAnalogSensor;
import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants.ElevatorConstants;
import frc.team3835.robot.commands.ElevatorDeafultCommand;
import edu.wpi.first.wpilibj.AnalogInput;

public class ElevatorSubsystem extends SubsystemBase {
  private DutyCycleEncoder absAxisEncoder;
  private TalonFX elevatorMotor;
  private CANSparkMax axisMotor;
  private CANSparkMax intakeMotor;
  private DigitalInput limitSwitch;

  private final int PID_SLOT = 0;
  private final int TIMEOUT_MS = 30;

  private final double maxVelocity = 0; // TODO: Find max velocity
  private final double maxAcceleration = 0; // Todo: Find max Acceleration

  
  public ElevatorSubsystem() {
    this.elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR);
    this.axisMotor = new CANSparkMax(ElevatorConstants.AXIS_MOTOR, MotorType.kBrushed);
    this.intakeMotor = new CANSparkMax(ElevatorConstants.INTAKE_MOTOR, MotorType.kBrushed);

    this.elevatorMotor.setInverted(ElevatorConstants.ELEVATOR_INVERTED);
    this.axisMotor.setInverted(ElevatorConstants.AXIS_INVERTED);
    this.intakeMotor.setInverted(ElevatorConstants.INTAKE_INVERTED);

    this.elevatorMotor.setNeutralMode(NeutralMode.Brake);
    this.axisMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.elevatorMotor.configFactoryDefault();

    this.limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH);
    this.absAxisEncoder = new DutyCycleEncoder(ElevatorConstants.ABS_ENCODER);

    setDefaultCommand(new ElevatorDeafultCommand(this));
  }

  public void setArmPosition(double position) {
    this.elevatorMotor.set(TalonFXControlMode.MotionMagic, position);
  }
  public void setPower(double power) {
    this.elevatorMotor.set(TalonFXControlMode.PercentOutput, isElevatorDown()&&power<0?0:power);
  }
  public boolean isElevatorDown() {
    return !this.limitSwitch.get();
  }
  public void setAxisPower(double power) {
    this.axisMotor.set(power);
  }
  public void setIntakePower(double power) {
    this.intakeMotor.set(power);
  }
  public void zeroMotors() {
    this.elevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder",this.absAxisEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Limit switch Pressed", isElevatorDown());
  }
}
