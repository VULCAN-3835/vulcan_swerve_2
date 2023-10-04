// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor;
  private DigitalInput limitSwitchClosed;
  private DigitalInput limitSwitchOpen;
  private AnalogInput distSensor;
  private PIDController elevatorPID;

  private final double MAX_ELEVATOR_DIST = 27.5;
  private final double MIN_ELEVATOR_DIST = 0;
  private double position = MIN_ELEVATOR_DIST;
  public ElevatorSubsystem() {
    this.elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR);

    this.elevatorMotor.setInverted(ElevatorConstants.ELEVATOR_INVERTED);

    this.elevatorMotor.setNeutralMode(NeutralMode.Brake);

    this.elevatorMotor.configFactoryDefault();

    this.limitSwitchClosed = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CLOSED);
    this.limitSwitchOpen = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_OPEN);
    this.distSensor = new AnalogInput(ElevatorConstants.DISTANCE_SENSOR);

    this.elevatorPID = new PIDController(0.117,0,0);

    this.elevatorPID.setTolerance(2.4);

    setDefaultCommand(null);
  }
  public void setElevatorPosition(double position) {
    if (position > MAX_ELEVATOR_DIST) {
      position = MAX_ELEVATOR_DIST;
    }
    if (position < MIN_ELEVATOR_DIST) {
      position = MIN_ELEVATOR_DIST;
    }
    this.position = position;
  }
  public void setElevatorPower(double power) {
    if (Math.abs(power) > ElevatorConstants.ELEVATOR_MOTOR_CAP) {
      if (power>0) {
        power = ElevatorConstants.ELEVATOR_MOTOR_CAP;
      }
      if (power<0) {
        power = -ElevatorConstants.ELEVATOR_MOTOR_CAP;
      }
    }

    if ((this.isElevatorDown() && power < 0)||
      (this.isElevatorUp() && power > 0)) {
       power = 0;
    }

    this.elevatorMotor.set(TalonFXControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Elevator Power", power);
  }
  private double getDistanceSensorVolt() {
    double v = this.distSensor.getVoltage(); // Volt
    double distance = (7.16*Math.pow(v,4) - 51.259*Math.pow(v,3) + 139.81*Math.pow(v,2) - 181.23*v + 100.51);
    return distance;
  }
  public double getElevatorDistance() {
    return getDistanceSensorVolt(); // TODO: Add func for getting distance
  }
  public boolean isElevatorDown() { return !this.limitSwitchClosed.get();}
  public boolean isElevatorUp() { return !this.limitSwitchOpen.get(); }

  @Override
  public void periodic() {
//    if(OI.getAButton()) {
//      position = MAX_ELEVATOR_DIST;
//    }
//    else {
//      position = MIN_ELEVATOR_DIST;
//    }

    this.elevatorPID.setSetpoint(this.position);
    double elevatorPower = elevatorPID.calculate(getElevatorDistance());
    this.setElevatorPower(elevatorPower);

    // Limit switches + Dist sensor sanity check
    SmartDashboard.putNumber("Setpoint Elevator", position);
    SmartDashboard.putNumber("Distance", this.getElevatorDistance());
    SmartDashboard.putBoolean("Elevator Down", isElevatorDown());
    SmartDashboard.putBoolean("Elevator Up", isElevatorUp());
  }
}
