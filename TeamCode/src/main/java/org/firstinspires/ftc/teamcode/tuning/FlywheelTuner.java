package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;

@TeleOp(name = "Flywheel Tuner")
public class FlywheelTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Replace normal telemetry with Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(FlywheelIO.P, FlywheelIO.I, FlywheelIO.D, FlywheelIO.F));
            s1.setVelocity(FlywheelIO.TARGET_VELOCITY);

            telemetry.addData("Target velocity", FlywheelIO.TARGET_VELOCITY);
            telemetry.addData("Current velocity", s1.getVelocity());
            telemetry.addData("Error", FlywheelIO.TARGET_VELOCITY - s1.getVelocity());
            telemetry.update();
        }
    }
}