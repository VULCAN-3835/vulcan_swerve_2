package frc.team3835.lib.ledcontrol;

import java.util.List;

public class LedSectionController extends LedController{
    public LedSectionController(int pwm_port, double power_supply, List<LedSection> ledSectionList) {
        super(pwm_port, 0, power_supply);
        int total_length = 0;
        for (LedSection ledSection: ledSectionList) {
            total_length += ledSection.length;
        }
        super.setLength(total_length);
    }

    // TODO: DO
}
