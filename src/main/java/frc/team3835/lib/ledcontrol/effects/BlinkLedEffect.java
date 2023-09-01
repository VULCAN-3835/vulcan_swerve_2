package frc.team3835.lib.ledcontrol.effects;

import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;

public class BlinkLedEffect extends TimedLedEffect {

    private final Color8Bit secondaryColor;

    public BlinkLedEffect(Color8Bit mainColor, int length, int frequency) {
        this(mainColor, length, frequency, new Color8Bit(0,0,0));
    }

    public BlinkLedEffect(Color8Bit mainColor, int length, int frequency, Color8Bit secondaryColor) {
        super(mainColor, length, frequency);
        this.secondaryColor = secondaryColor;
    }

    @Override
    protected void next() {
        if ((tick/((int)(TPS/ frequency)))%2 == 0){
            Arrays.fill(super.output, super.mainColor);
        }
        else {
            Arrays.fill(super.output, this.secondaryColor);
        }
        tick++;
    }
}
