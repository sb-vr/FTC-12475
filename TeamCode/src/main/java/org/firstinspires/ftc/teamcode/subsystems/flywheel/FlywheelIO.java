package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class ShooterIO {
    private DcMotor s1;

    private double targetPower = 0.0;
    private double currentPower = 0.0;
    private double maxRampRate = 0.05;
    private boolean shooting = true;
    private double shootingPower;
    public ShooterIO(DcMotor s1) {
        this.s1 = s1;
    }

    public void updateShooter(Gamepad gamepad1) {
        if (gamepad1.dpadUpWasPressed()) {
            shooting = !shooting;
        }
        if (shooting) {
            targetPower = shootingPower;
        } else {
            targetPower = 0.0;
        }

        if (currentPower < targetPower) {
            currentPower += maxRampRate;
            if (currentPower > targetPower) currentPower = targetPower;
        } else if (currentPower > targetPower) {
            currentPower -= maxRampRate;
            if (currentPower < targetPower) currentPower = targetPower;
        }

        s1.setPower(currentPower);
    }

//    public double lookup(double x, double[] bounds, double[] values) {
//        if (x < bounds[0]) {
//            double t = (x - bounds[0]) / (bounds[1] - bounds[0]);
//            return lerp(values[0], values[1], t);
//        }
//
//        if (x > bounds[bounds.length - 1]) {
//            int n = bounds.length - 1;
//            double t = (x - bounds[n]) / (bounds[n] - bounds[n-1]);
//            return lerp(values[n], values[n-1], t);
//        }
//
//        for (int i = 0; i < bounds.length - 1; i++) {
//            if (x >= bounds[i] && x <= bounds[i+1]) {
//                double t = (x - bounds[i]) / (bounds[i+1] - bounds[i]);
//                return lerp(values[i], values[i+1], t);
//            }
//        }
//
//        return -1;
//    }
//    private double lerp(double a, double b, double t) {
//        return a + (b - a) * t;
//    }

    public void setRampRate(double rampRate) {
        this.maxRampRate = rampRate;
    }
    public void setShootingPower(double shootingPower) {
        this.shootingPower = shootingPower;
    }
}
