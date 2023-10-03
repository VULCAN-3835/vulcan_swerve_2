package frc.team3835.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
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

    public static enum ControlMode{
        Autonomous, Disabled, Teleop
    }

    public static enum Operation {
        Idle, Equalizing, Equalized, IntakeCube, IntakeCone, ScoreCube, ScoreCone
    }

    public static enum AllianceColor{
        Neutral, Blue, Red
    }
    private ControlMode controlMode;
    private Operation operation;
    private Color8Bit allianceColor;

    public LedSubsystem(){
        this.led = new AddressableLED(PWM_PORT);
        this.led.setLength(87);
        this.ledController = new RobotLedController(87, 2);
        this.led.start();


        this.controlMode = LedSubsystem.ControlMode.Disabled;
        this.operation = LedSubsystem.Operation.Idle;
        this.allianceColor = WHITE;

    }

    public void setAllianceColor(AllianceColor allianceColor) {
        switch (allianceColor){
            case Red:
                this.allianceColor = RED;
                break;
            case Blue:
                this.allianceColor = BLUE;
                break;
            default:
                this.allianceColor = WHITE;
                break;
        }

    }

    @Override
    public void periodic() {
        if (this.controlMode == ControlMode.Disabled){
            this.ledController.staticColor(this.allianceColor);
        }
        else if (this.operation == Operation.Equalizing){
            this.ledController.blinkingColor(5, GREEN);
        }
        else if (this.operation == Operation.Equalized){
            this.ledController.staticColor(GREEN);
        }


        updateLeds();
    }

    private void updateLeds(){
        this.led.setData(this.ledController.getLedBuffer());
    }
}
