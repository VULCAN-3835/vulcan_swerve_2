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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3835.robot.Constants;
import frc.team3835.robot.util.Conversions;


/** Add your docs here. */
public class SwerveModule {

    // Motors + Sensors
    private TalonFX driveMotor; // Falcon motor responsible for power
    private TalonFX steerMotor; // Falcon motor responsible for steering module
    private CANCoder absEncoder; // Absolute encoder responsible for keeping track of module position

    // States
    private SwerveModuleState target = new SwerveModuleState(); // The desired target of the module
    private SwerveModuleState state = new SwerveModuleState(); // The current state of the module

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

        this.steerMotor.config_kP(0, Constants.SwerveConstants.steerP);
        this.steerMotor.config_kI(0, Constants.SwerveConstants.steerI);
        this.steerMotor.config_kD(0, Constants.SwerveConstants.steerD);

        this.steerMotor.setInverted(driveInverted);

        // Angle which is considered zero for encoder
        this.trueZero = trueZero;

        this.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.absEncoder.configMagnetOffset(this.trueZero);

        this.driveMotor.configClosedloopRamp(2);
        this.driveMotor.configSupplyCurrentLimit(Constants.SwerveConstants.driveSupplyLimit);

        this.steerMotor.configSupplyCurrentLimit(Constants.SwerveConstants.angleSupplyLimit);

        // Feedforward setup
        this.feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.feedForwardKs, Constants.SwerveConstants.feedForwardKv);

        this.state = new SwerveModuleState();
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
        this.state = SwerveModuleState.optimize(this.target, Rotation2d.fromDegrees(GetTrueAngle()));

        double positionError; // The error between desired wheel position and current one
        double positionErrorTicks; // The error between desired wheel position and current one in ticks
        double positionErrorDeadzone; // The acceptable deadzone for wheel position
        double falconPosition; // The current position of the falcon encoder

        positionError = this.state.angle.minus(Rotation2d.fromDegrees(GetTrueAngle())).getDegrees(); // Calculates the error between current state and desired one
        positionErrorTicks = Conversions.degreesToFalcon(positionError, 12.8 / 1.0); // Transfers error into falcon ticks
        positionErrorDeadzone = Math.abs(positionError) > 0.3 ? positionError : 0; // Checks if value is within deadzone threshhold
        falconPosition = this.steerMotor.getSelectedSensorPosition(); // Finds current position of falcon sensor
        
        if (driveOutput != 0 || positionError != 0) {
            this.steerMotor.set(TalonFXControlMode.Position,
            positionErrorTicks+falconPosition,
            DemandType.ArbitraryFeedForward,
            Constants.SwerveConstants.steerMotorThreshold*Math.signum(positionErrorDeadzone));
        }


//        this.driveOutput = this.state.speedMetersPerSecond; // Output for drive motor
//        this.driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput*0.2);


        this.driveOutput = this.state.speedMetersPerSecond;
        this.driveMotor.set(TalonFXControlMode.Velocity,
                Conversions.MPSToFalcon(this.driveOutput, Units.inchesToMeters(4*Math.PI), 6.75),
                DemandType.ArbitraryFeedForward,
                this.feedforward.calculate(driveOutput));
    }
    /**
     * Finds the true angle of the motor
     *
     * @return  The absolute true angle of the module
     */
    public double GetTrueAngle() { // Making sure values are within the -180 to 180 range and returning the true position by using the offset
        return this.absEncoder.getAbsolutePosition();

    }
    public double GetTargetAngle() {
        return this.state.angle.getDegrees();
    }
    public double GetTargetError() {
        return this.state.angle.minus(Rotation2d.fromDegrees(GetTrueAngle())).getDegrees();
    }
    public SwerveModulePosition GetModulePosition() {
        return new SwerveModulePosition(GetDistanceMeters(), Rotation2d.fromDegrees(GetTrueAngle()));
    }
    public double GetVel() {
        return this.driveMotor.getSelectedSensorVelocity();
    }

    public double GetDistanceMeters() {
        return Conversions.falconToMeters(GetDistance(), Units.inchesToMeters(4*Math.PI), 6.75);
    }
    public double GetDistance() {
        return this.driveMotor.getSelectedSensorPosition();
    }


}
