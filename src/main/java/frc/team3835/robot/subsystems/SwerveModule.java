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
    private TalonFX driveMotor; // Falcon motor responsible for power
    private TalonFX steerMotor; // Falcon motor responsible for steering module
    private CANCoder absEncoder; // Absolute encoder responsible for keeping track of module position

    // States
    private SwerveModuleState target; // The desired target of the module
    private SwerveModuleState state; // The current state of the module

    private double trueZero; // The offset for the wheel position
    private double driveOutput; // The output given to wheels

    // Control
    private SimpleMotorFeedforward feedforward; // FeedForward controler 

    public SwerveModule(int driveMotorPort, int steerMotorPort, int absEncPort,
                        double trueZero, boolean driveInverted) {
        // Ports setup
        this.driveMotor = new TalonFX(driveMotorPort);
        this.steerMotor = new TalonFX(steerMotorPort);
        this.absEncoder = new CANCoder(absEncPort);

        // Config Motors + Sensors
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.steerMotor.setNeutralMode(NeutralMode.Brake);

        this.driveMotor.config_kP(0, Constants.SwerveConstants.driveP); // TODO: find pid values
        this.driveMotor.config_kI(0, Constants.SwerveConstants.driveI);
        this.driveMotor.config_kD(0, Constants.SwerveConstants.driveD);
        
        driveMotor.configMotionAcceleration //Max Acceleration(m/s^2) in Pulses per 100ms
        ((Constants.SwerveConstants.maxAcceleration*Constants.SwerveConstants.pulseToMeterFalcon)/10); 

        driveMotor.setSelectedSensorPosition(0); // Sets the position of falcon encoder to 0

        this.steerMotor.config_kP(0, Constants.SwerveConstants.steerP); // TODO: find pid values
        this.steerMotor.config_kI(0, Constants.SwerveConstants.steerP);
        this.steerMotor.config_kD(0, Constants.SwerveConstants.steerP);

        this.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180

        // Angle which is considered zero for encoder
        this.trueZero = trueZero;

        // Feedforward setup
        this.feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.feedForwardKs, Constants.SwerveConstants.feedForwardKv);

    }

    /*
     * Stops module completely
     */
    public void stopMotors() {
        this.driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        this.steerMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * Sets the module's position to given values
     *
     * @param  angle  The desired angle of the module
     * @param  speed The desired speed of the module
     */
    public void set(double angle, double speed) { //
        this.target.angle = Rotation2d.fromDegrees(angle);
        this.target.speedMetersPerSecond = speed;
        set(this.target);
    }

    /**
     * Sets the module's state to given one
     *
     * @param  target  The desired target state of the module
     */
    public void set(SwerveModuleState target) { 
        this.target = target;
        this.state = SwerveModuleState.optimize(this.target, Rotation2d.fromDegrees(getTrueAngle()));

        double positionError; // The error between desired wheel position and current one
        double positionErrorTicks; // The error between desired wheel position and current one in ticks
        double positionErrorDeadzone; // The acceptable deadzone for wheel position
        double falconPosition; // The current position of the falcon encoder

        this.driveOutput = this.state.speedMetersPerSecond; // Output for drive motor
        positionError = this.state.angle.minus(Rotation2d.fromDegrees(getTrueAngle())).getDegrees(); // Calculates the error between current state and desired one
        positionErrorTicks = positionError * Constants.SwerveConstants.ticksPerDegree; // Transfers error into falcon ticks
        positionErrorDeadzone = Math.abs(positionError) > 1 ? positionError : 0; // Checks if value is within deadzone threshhold
        falconPosition = this.steerMotor.getSelectedSensorPosition(); // Finds current position of falcon sensor
        
        if (driveOutput != 0 || positionError != 0) {
            this.steerMotor.set(TalonFXControlMode.Position, // Sets the steer motor using the position control mode
            positionErrorTicks+falconPosition,
            DemandType.ArbitraryFeedForward,
            Constants.SwerveConstants.steerMotorThreshold*Math.signum(positionErrorDeadzone));
        }

        
        this.driveMotor.set(TalonFXControlMode.Velocity, // Sets the drive motor using velocity control mode
        (driveOutput*Constants.SwerveConstants.pulseToMeterFalcon)/10, 
        DemandType.ArbitraryFeedForward, 
        feedforward.calculate(driveOutput));   
    }

    /**
     * Finds the true angle of the motor
     *
     * @return  The absolute true angle of the module
     */
    private double getTrueAngle() { // Making sure values are within the -180 to 180 range and returning the true position by using the offset
        return -((((this.absEncoder.getAbsolutePosition()-this.trueZero)+180)%360)-180);
    }

}
