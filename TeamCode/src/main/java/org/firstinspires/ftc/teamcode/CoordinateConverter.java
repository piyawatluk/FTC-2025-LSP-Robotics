package org.firstinspires.ftc.teamcode;

import java.io.InputStream;
import java.util.Properties;

public class CoordinateConverter {

    private static final Properties prop = new Properties();
    public static double saturation;
    public static double sensitivity;
    public static double deadZone;
    public static boolean invertY;
    public static boolean invertX;
    public static String configError = null; // For reporting config errors to OpMode

    static {
        try (InputStream input = CoordinateConverter.class.getResourceAsStream("/Robot.config")) {
            if (input != null) {
                prop.load(input);
            } else {
                configError = "Robot.config not found.";
            }
        } catch (Exception e) {
            configError = "Failed to load Robot.config: " + e.getMessage();
        }

        saturation = 1.0;
        try {
            saturation = Double.parseDouble(prop.getProperty("Robot.Joystick_Saturation", "1.0"));
        } catch (NumberFormatException e) {
            configError = "Invalid saturation format. Using default.";
        }

        sensitivity = 1.0;
        try {
            sensitivity = Double.parseDouble(prop.getProperty("Robot.Joystick_Sensitivity", "1.0"));
        } catch (NumberFormatException e) {
            configError = "Invalid sensitivity format. Using default.";
        }

        deadZone = 0.0;
        try {
            deadZone = Double.parseDouble(prop.getProperty("Robot.Joystick_deadZone", "0.0"));
        } catch (NumberFormatException e) {
            configError = "Invalid deadZone format. Using default.";
        }

        String xProp = prop.getProperty("Robot.Joystick_invertX", "false");
        invertX = Boolean.parseBoolean(xProp);
        if (!xProp.equalsIgnoreCase("true") && !xProp.equalsIgnoreCase("false")) {
            configError = "Invalid invertX format. Using default";
        }

        String yProp = prop.getProperty("Robot.Joystick_invertY", "false");
        invertY = Boolean.parseBoolean(yProp);
        if (!yProp.equalsIgnoreCase("true") && !yProp.equalsIgnoreCase("false")) {
            configError = "Invalid invertY format. Using default";
        }
    }

    public static double computeX(float X, float Y, float range) {
        double r = coerceValue(Math.sqrt((X * X) + (Y * Y)));
        double a = Math.atan2(Y, X);
        double value = computeModifiers(r, deadZone, saturation, sensitivity, range);

        double x = value * Math.cos(a);
        if (invertX) x = -x;
        return x;
    }

    public static double computeY(float X, float Y, float range) {
        double r = coerceValue(Math.sqrt((X * X) + (Y * Y)));
        double a = Math.atan2(Y, X);
        double value = computeModifiers(r, deadZone, saturation, sensitivity, range);

        double y = value * Math.sin(a);
        if (invertY) y = -y;
        return y;
    }

    private static double computeModifiers(double value, double deadZone, double saturation, double sensitivity, double range) {
        if (deadZone > 0.0 || saturation < 1.0) {
            double edgeSpace = (1 - saturation) + deadZone;
            if (edgeSpace < 1.0) {
                double multiplier = 1.0 / (1.0 - edgeSpace);
                value = (value - deadZone) * multiplier;
                value = coerceValue(value);
            } else {
                value = Math.round(value);
            }
        }

        if (sensitivity != 0.0) {
            value = value + ((value - Math.sin(value * (Math.PI / 2))) * (sensitivity * 2));
            value = coerceValue(value);
        }

        if (range < 1.0) {
            value = value * range;
        }

        return coerceValue(value);
    }

    private static double coerceValue(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}