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

    public void blinkingColor(double frequency, Color8Bit color1){
        blinkingColor(frequency, color1, BLACK);
    }

    public void blinkingColor(double frequency, Color8Bit color1, Color8Bit color2){
        int ticks_in_cycle = (int)(TPS/frequency);
        if (tick % ticks_in_cycle < ticks_in_cycle/2){
            staticColor(color1);
        }
        else {
            staticColor(color2);
        }
        tick+=1;
    }

    public void breathingColor(double frequency, Color8Bit color){// TODO: test?
        double normal = Math.abs((tick%(TPS/frequency))-(TPS/(frequency*2.0)))/(TPS/(2.0*frequency));
        staticColor(new Color8Bit(
                (int)(color.red * normal),
                (int)(color.green * normal),
                (int)(color.blue * normal)
        ));
        tick+=1;
    }

    // makes a moving LED with trail effect
    // the frequency here is how many times a second the effect updates
    public void trailWaveColor(double frequency, boolean backward, Color8Bit color){
        if (tick%((int)(TPS/frequency)) != 0){ // skips runs
            return;
        }
        //for every led on the hopper
        for (int i = 0; i < ledBuffer.getLength(); i++){
            int red = (int)(ledBuffer.getLED(i).red * 255);
            int green = (int)(ledBuffer.getLED(i).green * 255);
            int blue = (int)(ledBuffer.getLED(i).blue * 255);

            ledBuffer.setRGB(i, red/2, green/2, blue/2); // dims every led by half
        }
        int index = (tick/((int)(TPS/frequency)))%ledBuffer.getLength();
        if (backward){
            index = ledBuffer.getLength() - 1 - index;
        }
        ledBuffer.setLED(index, color); // sets a new LED to full brightness
        tick+=1;
    }
}
