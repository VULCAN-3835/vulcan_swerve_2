package frc.team3835.lib.ledcontrol.effects;

import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;

public class StaticLedEffect extends LedEffect{

    public StaticLedEffect(Color8Bit mainColor, int length){
        super(mainColor, length);
        Arrays.fill(super.output, super.mainColor);
    }

}
