package frc.util.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LEDController {
    AddressableLED leds;
    AddressableLEDBuffer ledBuffer;

    /*
     * <b>Creates a new LEDController. <br>
     * LEDController is a class for managing LED at a low level.
     * 
     * @param numLEDs The number of LEDs in the LED controller.
     * @param pwmPort The PWM port the LED controller is connected to.
     */
    public LEDController(int numLEDs, int pwmPort) {
        leds = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(numLEDs);

        leds.setLength(ledBuffer.getLength());
        leds.start();
    }

    public void apply() {
        leds.setData(ledBuffer);
    }

    public void setColor(int index, int hue, int sat, int val) {
        ledBuffer.setHSV(index, hue, sat, val);
    }

    public void setColor(int index, Color color) {
        ledBuffer.setHSV(index, color.hue(), color.sat(), color.val());
    }

    public void setStripColor(LEDStrip strip, int hue, int sat, int val) {
        for (int i = strip.startIndex(); i < strip.endIndex(); i++) { setColor(i, hue, sat, val); }
    }

    public void setStripColor(LEDStrip strip, Color color) {
        for (int i = strip.startIndex(); i < strip.endIndex(); i++) { setColor(i, color); }
    }

    public void setBlinking(LEDStrip strip, double speed, Color color) {
        setStripColor(strip, ((int)(Timer.getFPGATimestamp() * speed)) % 2 == 0 ? color : Color.OFF);
    }

    public void setPulsing(LEDStrip strip, double speed, Color color) {
        setStripColor(strip, color.newVal((int)(color.val() * (0.5 + 0.5 * Math.sin(Timer.getFPGATimestamp() * speed * 2 * Math.PI)))));
    }

    public void setRainbow(LEDStrip strip, double speed, Color color) {
        for (int i = 0; i < strip.length(); i++) {
            setColor(i + strip.startIndex(), color.newHue((int)((((i * 180.0 / strip.length()) + (int)((Timer.getFPGATimestamp() * 180.0) / speed)) % 180.0))));
        }
    }

    public void setSwirling(LEDStrip strip,  double speed, int segmentSize, Color color1, Color color2) {
        for (int i = 0; i < strip.length(); i++) {
            int segmentIndex = ((i + (int)(Timer.getFPGATimestamp() * speed)) / segmentSize) % 2;
            setColor(i + strip.startIndex(), (segmentIndex == 0) ? color1 : color2);
        }
    }

}
