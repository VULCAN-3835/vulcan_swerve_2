package frc.team3835.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.robot.Constants;
import frc.team3835.robot.OI;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax axisMotor;
    private CANSparkMax intakeMotor;
    private DutyCycleEncoder absAxisEncoder;
    private PIDController axisPID;
    private final double MIN_AXIS_ANGLE = 2; // TODO: Find min and max angles
    private final double MAX_AXIS_ANGLE = 155;
    private final double ANGLE_CLOSED = 140;

    private double intakePower;
    private double position = MAX_AXIS_ANGLE;

    public IntakeSubsystem() {
        this.axisMotor = new CANSparkMax(Constants.ElevatorConstants.AXIS_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(Constants.ElevatorConstants.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.axisMotor.setInverted(Constants.ElevatorConstants.AXIS_INVERTED);
        this.intakeMotor.setInverted(Constants.ElevatorConstants.INTAKE_INVERTED);

        this.axisMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.absAxisEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.ABS_ENCODER);

        this.absAxisEncoder.setPositionOffset(Constants.ElevatorConstants.ABS_ENCODER_OFFSET);

        this.axisPID = new PIDController(0.007,0,0);

        this.axisPID.setTolerance(3);

        setDefaultCommand(null);
    }
    public void setAxisPosition(double position) {
        if (position > MAX_AXIS_ANGLE) {
            position = MAX_AXIS_ANGLE;
        }
        if (position < MIN_AXIS_ANGLE) {
            position = MIN_AXIS_ANGLE;
        }
        this.position = position;
    }
    private void setAxisPower(double power) {
        if (Math.abs(power) > Constants.ElevatorConstants.AXIS_MOTOR_CAP) {
            if (power>0) {
                power = Constants.ElevatorConstants.AXIS_MOTOR_CAP;
            }
            if (power<0) {
                power = -Constants.ElevatorConstants.AXIS_MOTOR_CAP;
            }
        }
        if (
           (getAxisAngle() <= MIN_AXIS_ANGLE && power < 0)||
           (getAxisAngle() >= MAX_AXIS_ANGLE && power > 0)){
                power = 0;
        }
        this.axisMotor.set(power);
        SmartDashboard.putNumber("Power To Axis Motor" , power);

    }
    public void setIntakePower(double power) {
        this.intakePower = power;
    }
    public boolean isAtSetpoint() {
        return this.axisPID.atSetpoint();
    }
    public double getAxisAngle() {
        double angle = this.absAxisEncoder.getAbsolutePosition();
        return -(angle-Constants.ElevatorConstants.ABS_ENCODER_OFFSET)*360;
    }

    public boolean isClosed(){
        return this.getAxisAngle() > ANGLE_CLOSED;
    }
    public void periodic() {
//        if(OI.getAButton()) {
//            this.position = MIN_AXIS_ANGLE;
//        }
//        else {
//            this.position = MAX_AXIS_ANGLE;
//        }
        this.axisPID.setSetpoint(this.position);
        double axisPower = this.axisPID.calculate(getAxisAngle());

        this.setAxisPower(axisPower);

        if (OI.getLeftBumper()) {
            this.intakeMotor.set(Constants.ElevatorConstants.INTAKE_POWER);
        }
        else if (OI.getRightBumper()) {
            this.intakeMotor.set(-Constants.ElevatorConstants.INTAKE_POWER);
        }
        // Intake set power
        if (isAtSetpoint() && !(OI.getRightBumper()||OI.getLeftBumper())){
            this.intakeMotor.set(this.intakePower);
        } else if (!(OI.getRightBumper()||OI.getLeftBumper())) {
            this.intakeMotor.set(0);
        }

        // Sensor Sanity Check
        SmartDashboard.putBoolean("Is At setpoint", isAtSetpoint());
        SmartDashboard.putNumber("Absolute Encoder", getAxisAngle());
        SmartDashboard.putNumber("Absolute Position", this.absAxisEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Setpoint Angle", position);


    }
}

