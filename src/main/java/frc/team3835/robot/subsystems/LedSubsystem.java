package frc.team3835.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final int PWM_PORT = 4;
    private final Color8Bit RED = new Color8Bit(255, 0, 0);
    private final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    private final Color8Bit BLUE = new Color8Bit(0, 0, 255);
    private final Color8Bit PURPLE = new Color8Bit(255,0,255);
    private final Color8Bit YELLOW = new Color8Bit(255,255,0);
    private final Color8Bit WHITE = new Color8Bit(255,255,255);
    //LedSectionController ledController;
    RobotLedController ledController;
    AddressableLED led;


    public static enum Operation {
        Idle, Equalizing, Equalized, IntakeCube, IntakeCone, ScoreCube, ScoreCone
    }
    private Operation operation;
    private Color8Bit allianceColor;

    public LedSubsystem(){
        this.led = new AddressableLED(PWM_PORT);
        this.led.setLength(87);
        this.ledController = new RobotLedController(87, 2);
        this.led.start();


        this.controlMode = ControlMode.Teleop;
        this.operation = Operation.Idle;
        this.allianceColor = WHITE;

    }

    private void updateAlliance(){
        switch (DriverStation.getAlliance()){
            case Blue:
                this.allianceColor = BLUE;
                break;
            case Red:
                this.allianceColor = RED;
                break;
            case Invalid:
                this.allianceColor = WHITE;
                break;
        }

    }

    private void updateBuffer(){
        // assumes the robot is enabled

        switch (this.operation){
            case Equalizing:
                this.ledController.blinkingColor(2, WHITE);
                break;
            case Equalized:
                this.ledController.staticColor(GREEN);
                break;
            case ScoreCone:
                this.ledController.staticColor(YELLOW);
                break;
            case ScoreCube:
                this.ledController.staticColor(PURPLE);
                break;
            case IntakeCone:
                this.ledController.blinkingColor(2, YELLOW);
                break;
            case IntakeCube:
                this.ledController.blinkingColor(2, PURPLE);
                break;
            default:{
                if (DriverStation.isAutonomousEnabled()){
                    this.ledController.breathingColor(1, this.allianceColor);
                }
                else { // its Teleop
                    this.ledController.breathingColor(2, this.allianceColor);
                }
            }

        }
    }

    @Override
    public void periodic() {
        updateAlliance();
        if (DriverStation.isEnabled()){
            updateBuffer();
        }
        else{
            this.ledController.trailWaveColor(10, false, this.allianceColor);
        }
        updateLeds();
    }

    private void updateLeds(){
        this.led.setData(this.ledController.getLedBuffer());
    }
}
