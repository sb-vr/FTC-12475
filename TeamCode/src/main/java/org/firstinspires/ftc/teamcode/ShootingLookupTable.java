package org.firstinspires.ftc.teamcode;

public class ShootingLookupTable {

    // Measured distances (inches) - must be sorted ascending
    private static final double[] DISTANCES = {
            24, 36, 48, 60, 72
    };

    // Corresponding flywheel power (0.0 - 1.0)
    private static final double[] FLYWHEEL_POWERS = {
            0.7, 0.6, 0.7, 0.78, 0.9
    };

    // Corresponding hood angles (degrees, 15-40)
    private static final double[] HOOD_ANGLES = {
            15, 20, 25, 32, 40
    };

    public static double getFlywheelPower(double distance) {
        double power = lookup(distance, DISTANCES, FLYWHEEL_POWERS);
        return Math.max(0.0, Math.min(1.0, power));
    }

    public static double getHoodAngle(double distance) {
        double angle = lookup(distance, DISTANCES, HOOD_ANGLES);
        return Math.max(15.0, Math.min(40.0, angle));
    }

    private static double lookup(double x, double[] bounds, double[] values) {
        int n = bounds.length;

        if (x <= bounds[0]) {
            double t = (x - bounds[0]) / (bounds[1] - bounds[0]);
            return lerp(values[0], values[1], t);
        }

        if (x >= bounds[n - 1]) {
            double t = (x - bounds[n - 2]) / (bounds[n - 1] - bounds[n - 2]);
            return lerp(values[n - 2], values[n - 1], t);
        }

        for (int i = 0; i < n - 1; i++) {
            if (x >= bounds[i] && x <= bounds[i + 1]) {
                double t = (x - bounds[i]) / (bounds[i + 1] - bounds[i]);
                return lerp(values[i], values[i + 1], t);
            }
        }

        return values[0];
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}