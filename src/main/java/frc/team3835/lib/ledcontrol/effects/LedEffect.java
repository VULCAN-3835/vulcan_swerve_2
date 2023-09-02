package frc.team3835.lib.ledcontrol.effects;

import edu.wpi.first.wpilibj.util.Color8Bit;

public abstract class LedEffect {
    protected Color8Bit mainColor;
    protected Color8Bit[] colorArray;

    protected LedEffect(Color8Bit mainColor, int length){
        this.mainColor = mainColor;
        this.colorArray = new Color8Bit[length];
    }
    
    public Color8Bit[] getPixelArray(){
        return colorArray;
    }

    public void setLength(int length){
        this.colorArray = new Color8Bit[length];
    }
}
