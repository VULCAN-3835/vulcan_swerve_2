package frc.team3835.lib.ledcontrol.effects;

import edu.wpi.first.wpilibj.util.Color8Bit;

public abstract class TimedLedEffect extends LedEffect{
    protected double frequency;
    protected final int TPS = 50;
    protected int tick;


    protected TimedLedEffect(Color8Bit mainColor, int length, double frequency) {
        super(mainColor, length);
        this.frequency = Math.min(frequency, TPS);
        this.tick = 0;
    }

    protected abstract void next();

    @Override
    public Color8Bit[] getPixelArray() {
        next();
        return super.getPixelArray();
    }
}
