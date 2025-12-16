package frc.robot.constants;

import frc.util.leds.LEDStrip;

public class LEDConstants {
    public static final int LED_PWM_PORT = 0;
    public static final int LED_LENGTH = 200;
    
    public static final int RED_HUE = 0;
    public static final int ORANGE_HUE = 5;
    public static final int YELLOW_HUE = 15;
    public static final int GREEN_HUE = 60;
    public static final int BLUE_HUE = 120;
    public static final int PURPLE_HUE = 140;
    public static final int PINK_HUE = 165;

    public static final LEDStrip strip1 = new LEDStrip(12, 0);
    public static final LEDStrip strip2 = new LEDStrip(17, 12);
    public static final LEDStrip strip3 = new LEDStrip(12, 29);
    public static final LEDStrip strip4 = new LEDStrip(17, 41);
    public static final LEDStrip allLEDs = new LEDStrip(200, 0);

    public enum LED_STATES {
        A,
        B,
        X,
        Y,
        TEST,
        AUTO,
        SINGLE_CONTROLLER,
        COLLECTED,
        SHOT,
        SHOOTING,
        COLLECTING;

        public int ID() {
            return this.ordinal();
        }
    }

}