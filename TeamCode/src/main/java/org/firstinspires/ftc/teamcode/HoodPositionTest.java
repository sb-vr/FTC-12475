package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Position Test")
public final class HoodPositionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo hood = hardwareMap.get(Servo.class, "hood");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Reported Position", hood.getPosition());
            telemetry.update();
            sleep(50);
        }
    }
}