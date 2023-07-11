// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3835.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team3835.robot.Constants;

/** Add your docs here. */
public class SwerveModule {

    // Motors + Sensors
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder absEncoder;

    // States
    private SwerveModuleState target;
    private SwerveModuleState state;

    //
    private double trueZero;
    private double driveOutput;

    // Control
    private SimpleMotorFeedforward ff;
    private double positionError;
    private double positionErrorTicks;
    private double positionErrorDeadzone;
    private double falconPosition;

    public SwerveModule(int driveMotorPort, int steerMotorPort, int absEncPort, double trueZero, boolean driveInverted) {
        // Ports setup
        this.driveMotor = new TalonFX(driveMotorPort);
        this.steerMotor = new TalonFX(steerMotorPort);
        this.absEncoder = new CANCoder(absEncPort);

        // Config Motors + Sensors
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.steerMotor.setNeutralMode(NeutralMode.Brake);

        this.driveMotor.config_kP(0, Constants.SwerveConstants.driveP);
        this.driveMotor.config_kI(0, Constants.SwerveConstants.driveI);
        this.driveMotor.config_kD(0, Constants.SwerveConstants.driveD);

        driveMotor.configMotionAcceleration((Constants.SwerveConstants.maxAcceleration*Constants.SwerveConstants.pulseToMeterFalcon)/10); // Acceleration(m/s^2) in Pulses per 100ms
        driveMotor.setSelectedSensorPosition(0);

        this.steerMotor.config_kP(0, Constants.SwerveConstants.steerP);
        this.steerMotor.config_kI(0, Constants.SwerveConstants.steerP);
        this.steerMotor.config_kD(0, Constants.SwerveConstants.steerP);

        this.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180

        // Angle which is considered zero for encoder
        this.trueZero = trueZero;

        // Feedforward setup
        this.ff = new SimpleMotorFeedforward(Constants.SwerveConstants.feedForwardKs, Constants.SwerveConstants.feedForwardKv);

    }

    // Stops the motors completely
    public void stopMotors() {
        this.driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        this.steerMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void set(double angle, double speed) {
        this.target.angle = Rotation2d.fromDegrees(angle);
        this.target.speedMetersPerSecond = speed;
        set(this.target);
    }
    
    public void set(SwerveModuleState target) {
        this.target = target;
        this.state = SwerveModuleState.optimize(this.target, Rotation2d.fromDegrees(getTrueAngle()));

        // Setting drive output and position error in ticks
        this.driveOutput = this.state.speedMetersPerSecond;
        this.positionError = this.state.angle.minus(Rotation2d.fromDegrees(getTrueAngle())).getDegrees();
        this.positionErrorTicks = positionError * Constants.SwerveConstants.ticksPerDegree;
        this.positionErrorDeadzone = Math.abs(positionError) > 1 ? positionError : 0;
        falconPosition = this.steerMotor.getSelectedSensorPosition();
        
        if (driveOutput != 0 || positionError != 0) {
            this.steerMotor.set(TalonFXControlMode.Position,
            positionErrorTicks+falconPosition,
            DemandType.ArbitraryFeedForward,
            Constants.SwerveConstants.steerMotorThreshold*Math.signum(positionErrorDeadzone));
        }

        
        this.driveMotor.set(TalonFXControlMode.Velocity, (driveOutput*Constants.SwerveConstants.pulseToMeterFalcon)/10, DemandType.ArbitraryFeedForward, ff.calculate(driveOutput));   
    }

    private double getTrueAngle() { // Making sure values are within the -180 to 180 range and returning the true position by using the offset
        return -((((this.absEncoder.getAbsolutePosition()-this.trueZero)+180)%360)-180);
    }

}
