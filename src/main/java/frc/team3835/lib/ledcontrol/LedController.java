package frc.team3835.lib.ledcontrol;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LedController { // TODO: document every file in this package (and sub-packages)
    private double power_supply;
    protected AddressableLEDBuffer ledBuffer; // WIP's AddressableLED
    private static final double AMPS_PER_LED = 0.02;
    public LedController(int total_length, double power_supply){
        this.power_supply = power_supply;

        this.ledBuffer = new AddressableLEDBuffer(total_length); // builds the LED strip buffer
    }

    protected void setLength(int total_length){
        this.ledBuffer = new AddressableLEDBuffer(total_length); // builds the LED strip buffer
    }

    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }

    public void updateBuffer(){
        normalizeBuffer();
    }

    public void normalizeBuffer(){
        double amp_sum = 0;
        for (int i = 0; i < this.ledBuffer.getLength(); i++) {
            Color pixel = this.ledBuffer.getLED(i);
            amp_sum += (pixel.red + pixel.green + pixel.blue)*AMPS_PER_LED;
        }
        if (amp_sum > power_supply){
            double scale = power_supply /amp_sum;
            for (int i = 0; i < this.ledBuffer.getLength(); i++) {
                Color8Bit pixel = this.ledBuffer.getLED8Bit(i);
                this.ledBuffer.setRGB(i,
                        (int)(pixel.red * scale), (int)(pixel.green * scale), (int)(pixel.blue * scale));
            }
        }
    }
}
