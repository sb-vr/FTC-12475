package org.firstinspires.ftc.teamcode.subsystems.constants;

public class HoodConstants {
    public static final double GEAR_RATIO = 180.0 / 18.0; // = 10

    // Hood angle limits (degrees)
    public static double MIN_HOOD_ANGLE = 0.0;
    public static double MAX_HOOD_ANGLE = 60.0;

    public static final double SERVO_RANGE_DEGREES = 300.0;

    public static final double BASE_SERVO_OFFSET_DEGREES = MIN_HOOD_ANGLE * GEAR_RATIO;
}
