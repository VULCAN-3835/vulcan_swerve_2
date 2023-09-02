package frc.team3835.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3835.lib.ledcontrol.LedSection;
import frc.team3835.lib.ledcontrol.LedSectionController;
import frc.team3835.lib.ledcontrol.effects.LedEffect;
import frc.team3835.lib.ledcontrol.effects.StaticLedEffect;
import org.ejml.ops.CommonOps_BDRM;

import java.util.ArrayList;
import java.util.HashMap;

public class LedSubsystem extends SubsystemBase {
    private final int PWM_PORT = 9;
    private final Color8Bit RED = new Color8Bit(255, 0, 0);
    private final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    private final Color8Bit BLUE = new Color8Bit(0, 0, 255);
    private final Color8Bit PURPLE = new Color8Bit(255,0,255);
    LedSectionController ledController;
    public LedSubsystem(){
        HashMap<Integer, LedEffect> states1 = new HashMap<>(2);
        states1.put(0, new StaticLedEffect(RED, 22));
        states1.put(1, new StaticLedEffect(GREEN, 22));

        ArrayList<LedSection> ledSections = new ArrayList<>();
        ledSections.add(new LedSection(22, false, false, states1));
        this.ledController = new LedSectionController(PWM_PORT, 2, ledSections);

    }

    @Override
    public void periodic() {
        this.ledController.updateBuffer(0);
    }
}
