package frc.team3835.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team3835.lib.ledcontrol.LedController;



public class RobotLedController extends LedController {

    private int tick;
    private static final int TPS = 20;

    private static final Color8Bit BLACK = new Color8Bit(0,0,0);

    public RobotLedController(int total_length, double power_supply) {
        super(total_length, power_supply);
        tick = 0;
    }

    @Override
    public AddressableLEDBuffer getLedBuffer() {
        super.normalizeBuffer();
        return super.getLedBuffer();
    }

    public void staticColor(Color8Bit color){
        for(int i = 0; i < super.ledBuffer.getLength(); i++) {
            super.ledBuffer.setLED(i, color);
        }
    }

    public void blinkingColor(int ticks_per_color, Color8Bit color1){
        blinkingColor(ticks_per_color, color1, BLACK);
    }

    public void blinkingColor(int ticks_per_color, Color8Bit color1, Color8Bit color2){
        Color8Bit color = (tick/ticks_per_color)%2 == 0 ? color1: color2;
        staticColor(color);
        tick+=1;
    }
}
