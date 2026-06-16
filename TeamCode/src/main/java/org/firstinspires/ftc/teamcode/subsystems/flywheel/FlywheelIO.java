package org.firstinspires.ftc.teamcode.subsystems.flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class FlywheelIO {
    private final DcMotor s1, s2;

    private double targetPower = 0.0;
    private double currentPower = 0.0;
    private double maxRampRate = 0.05;
    private boolean shooting = false;
    private double shootingPower = 0.0;

    public FlywheelIO(DcMotor s1, DcMotor s2) {
        this.s1 = s1;
        this.s2 = s2;
    }

    public void updateShooter(Gamepad gamepad1) {
        if (gamepad1.rightBumperWasPressed()) {
            shooting = true;
        } else if (gamepad1.right_trigger > 0.5) {
            shooting = false;
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
        s2.setPower(currentPower);
    }

    public void setRampRate(double rampRate) {
        this.maxRampRate = rampRate;
    }

    public void setShootingPower(double shootingPower) {
        this.shootingPower = shootingPower;
    }

    public boolean isShooting() {
        return shooting;
    }
}