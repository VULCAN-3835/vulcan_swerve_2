package frc.team3835.lib.ledcontrol;

import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;

public class LedSectionController extends LedController{
    private final List<LedSection> ledSectionList;
    public LedSectionController(double power_supply, List<LedSection> ledSectionList) {
        super(0, power_supply);
        int total_length = 0;
        for (LedSection ledSection: ledSectionList) {
            total_length += ledSection.length;
        }
        super.setLength(total_length);
        this.ledSectionList = ledSectionList;
    }

    public void updateBuffer(int state) {
        int i = 0;
        for (LedSection section: this.ledSectionList) {
            for (Color8Bit color :section.getColors(state)) {
                this.ledBuffer.setLED(i, color);
                i++;
            }
        }
        //super.updateBuffer();
    }
}
