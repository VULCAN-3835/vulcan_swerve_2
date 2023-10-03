package frc.team3835.robot.lib.ledcontrol;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team3835.lib.ledcontrol.LedController;
import frc.team3835.lib.ledcontrol.LedSection;
import frc.team3835.lib.ledcontrol.LedSectionController;
import frc.team3835.lib.ledcontrol.effects.BlinkLedEffect;
import frc.team3835.lib.ledcontrol.effects.LedEffect;
import frc.team3835.lib.ledcontrol.effects.StaticLedEffect;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class LedControllerTests {
    // To learn more about how to write unit tests, see the
    // JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

    private final Color8Bit RED = new Color8Bit(255, 0, 0);
    private final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    private final Color8Bit BLUE = new Color8Bit(0, 0, 255);
    private final Color8Bit PURPLE = new Color8Bit(255,0,255);
    private final Color8Bit WHITE = new Color8Bit(255,255,255);

    @Test
    void whiteController()
    {
        int length = 300;
        LedController ledController = new LedController(length, 2);
        for (int i = 0; i < length; i++) {
            ledController.getLedBuffer().setLED(i, WHITE);
        }
        for (int i = 0; i < length; i++) {
            assertEquals(ledController.getLedBuffer().getLED8Bit(i), WHITE);
        }
        ledController.updateBuffer();
    }

    @Test
    void normalize()
    {
        int length = 600;
        double power_supply = 5;
        LedController ledController = new LedController(length, power_supply);
        for (int i = 0; i < length; i++) {
            ledController.getLedBuffer().setLED(i, WHITE);
        }
        ledController.updateBuffer();
        double real = 0;
        for (int i = 0; i < length; i++) {
            Color8Bit color = ledController.getLedBuffer().getLED8Bit(i);
            real += color.red/255.0 * 0.02 + color.green/255.0 * 0.02 + color.blue/255.0 * 0.02;
        }
        double estimated_max = 0.06*length;
        assertTrue(Math.abs(real - Math.min(estimated_max, power_supply)) < 0.07);
    }

    @Test
    void effectGetColors(){
        LedEffect effect = new StaticLedEffect(RED, 3);
        Color8Bit[] array = new Color8Bit[3];
        Arrays.fill(array, RED);
        assertArrayEquals(array, effect.getPixelArray());
    }

    @Test
    void timedEffectGetColors(){
        LedEffect effect = new BlinkLedEffect(RED, 3, 50, BLUE);
        Color8Bit[] REDs = new Color8Bit[3];
        Arrays.fill(REDs, RED);
        Color8Bit[] BLUEs = new Color8Bit[3];
        Arrays.fill(BLUEs, BLUE);
        assertArrayEquals(REDs, effect.getPixelArray());
        assertArrayEquals(BLUEs, effect.getPixelArray());
        assertArrayEquals(REDs, effect.getPixelArray());
        assertArrayEquals(BLUEs, effect.getPixelArray());

    }

    // TODO: write tests for LedSection
    // TODO: write more tests

    @Test
    void makeSectionController(){
        ArrayList<LedSection> section_list = new ArrayList<>(2);
        HashMap<Integer, LedEffect> effectHashMap1 = new HashMap<>(1);
        effectHashMap1.put(0, new StaticLedEffect(RED, 5));
        section_list.add(new LedSection(5, effectHashMap1));
        HashMap<Integer, LedEffect> effectHashMap2 = new HashMap<>(1);
        effectHashMap2.put(0, new StaticLedEffect(BLUE, 5));
        section_list.add(new LedSection(5, effectHashMap2));
        LedSectionController sectionController = new LedSectionController(5, section_list);
        sectionController.updateBuffer(0);


    }
}
