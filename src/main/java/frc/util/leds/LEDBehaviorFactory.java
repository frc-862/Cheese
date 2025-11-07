package frc.util.leds;

import java.util.function.BooleanSupplier;

public class LEDBehaviorFactory {

    public static LEDBehavior SolidColorBehavior(LEDStrip strip, Color color) {
        return new LEDBehavior((ledController) -> {
            ledController.setStripColor(strip, color);
        });
    }

    public static LEDBehavior RainbowBehavior(LEDStrip strip, int speed) {
        return new LEDBehavior((ledController) -> {
            ledController.setRainbow(strip, speed, Color.RED);
        });
    }

    public static LEDBehavior RainbowBehavior(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            ledController.setRainbow(strip, speed, color);
        });
    }
 
    public static LEDBehavior BlinkColorBehavior(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            ledController.setBlinking(strip, speed, color);
        });
    }
    
    public static LEDBehavior pulseColorBehavior(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            ledController.setPulsing(strip, speed, color);
        });
    }

    public static LEDBehavior SwirlBehabior(LEDStrip strip, double speed, int segmentSize, Color color1, Color color2) {
        return new LEDBehavior((ledController) -> {
            ledController.setSwirling(strip, speed, segmentSize, color1, color2);
        });
    }

    public static LEDBehavior TestStripBehavior(int startIndex, BooleanSupplier... values) {
        return new LEDBehavior((ledController) -> {
            for (int i = 0; i < values.length; i++) {
                ledController.setColor(startIndex + i, (values[i].getAsBoolean() ? Color.GREEN : Color.RED));
            }
        });
    }
}
