package org.firstinspires.ftc.teamcode;

public class ShootingLookupTable {

    private static final double[] DISTANCES = {10, 36, 48, 60, 72, 84, 96, 110
    };

    // RPM omgezet naar ticks per seconde (28 ticks/rev)
    // rpm / 60 * 28
    private static final double[] FLYWHEEL_VELOCITIES = {1400, 1500, 1620, 1680, 1840, 1950, 2040, 2100
    };

    private static final double[] HOOD_ANGLES = {15, 18, 20, 29, 32, 35, 38, 40
    };

    public static double getFlywheelVelocity(double distance) {
        return lookup(distance, DISTANCES, FLYWHEEL_VELOCITIES);
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
