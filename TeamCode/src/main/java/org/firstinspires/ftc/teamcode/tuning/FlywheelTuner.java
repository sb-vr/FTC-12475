package org.firstinspires.ftc.teamcode.tuning;


import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.D;
import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.F;
import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.I;
import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel Tuner")
public class FlywheelTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            s1.setVelocity(500.0);

            telemetry.addData("Target velocity", 500.0);
            telemetry.addData("Current velocity", s1.getVelocity());
            telemetry.addData("Error", 500.0 - s1.getVelocity());
            telemetry.update();
        }
    }
}
