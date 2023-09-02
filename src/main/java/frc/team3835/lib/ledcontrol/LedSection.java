package frc.team3835.lib.ledcontrol;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team3835.lib.ledcontrol.effects.LedEffect;

import java.util.*;

public class LedSection {
    public final int length;
    private final boolean centralized;
    private final boolean inverted;
    private final Map<Integer,LedEffect> ledEffectMap;

    public LedSection(int length, Map<Integer, LedEffect> ledEffectMap){
        this(length, false, false, ledEffectMap);
    }

    public LedSection(int length, boolean centralized, boolean inverted, Map<Integer,LedEffect>  ledEffectMap) {
        this.length = length;
        this.centralized = centralized;
        this.inverted = inverted;

        int effect_length = centralized?(int)Math.ceil(length/2.0):length;
        for (LedEffect effect: ledEffectMap.values()) {
            effect.setLength(effect_length);
        }
        this.ledEffectMap = ledEffectMap;
    }

    public List<Color8Bit> getColors(int state){
        LedEffect effect = this.ledEffectMap.get(state);
        List<Color8Bit> list = Arrays.asList(effect.getPixelArray());

        if (this.inverted){
            invert(list);
        }
        if (this.centralized){
            centralize(list);
        }

        return list;
    }

    private void centralize(List list){
        int i = list.size() - 1;
        if (this.length%2==0) {
            list.add(list.get(i));
        }
        i--;
        while (i >= 0){
            list.add(list.get(i));
            i--;
        }
    }

    private void invert(List list){
        Collections.reverse(list);
    }


}
