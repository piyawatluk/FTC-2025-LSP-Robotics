package org.firstinspires.ftc.teamcode;

public class cordinate_converter {

    // Input variables (should be passed explicitly, not stored in the class)
    public static double computeX(double X, double Y, double deadZone, double saturation, double sensitivity, double range, boolean invertX, boolean invertY) {
        // Convert to polar coordinates
        double r = coerceValue(Math.sqrt((X * X) + (Y * Y)), 0.0, 1.0);
        double a = Math.atan2(Y, X);

        // Apply modifiers
        double value = computeModifiers(r, deadZone, saturation, sensitivity, range);

        // Convert back to Cartesian
        double x = value * Math.cos(a);
        if (invertX) x = -x;

        return x;
    }

    public static double computeY(double X, double Y, double deadZone, double saturation, double sensitivity, double range, boolean invertX, boolean invertY) {
        // Convert to polar coordinates
        double r = coerceValue(Math.sqrt((X * X) + (Y * Y)), 0.0, 1.0);
        double a = Math.atan2(Y, X);

        // Apply modifiers
        double value = computeModifiers(r, deadZone, saturation, sensitivity, range);

        // Convert back to Cartesian
        double y = value * Math.sin(a);
        if (invertY) y = -y;

        return y;
    }

    private static double computeModifiers(double value, double deadZone, double saturation, double sensitivity, double range) {
        // Apply dead-zone and saturation
        if (deadZone > 0.0 || saturation < 1.0) {
            double edgeSpace = (1 - saturation) + deadZone;
            if (edgeSpace < 1.0) {
                double multiplier = 1.0 / (1.0 - edgeSpace);
                value = (value - deadZone) * multiplier;
                value = coerceValue(value, 0.0, 1.0);
            } else {
                value = Math.round(value);
            }
        }

        // Apply sensitivity
        if (sensitivity != 0.0) {
            value = value + ((value - Math.sin(value * (Math.PI / 2))) * (sensitivity * 2));
            value = coerceValue(value, 0.0, 1.0);
        }

        // Apply range
        if (range < 1.0) {
            value = value * range;
        }

        return coerceValue(value, 0.0, 1.0);
    }

    private static double coerceValue(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

}