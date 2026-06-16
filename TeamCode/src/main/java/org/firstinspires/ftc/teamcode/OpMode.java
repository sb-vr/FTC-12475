package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ALLIANCE_TAG_ID;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Hood.HoodIO;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalize;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class OpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor i = null, st = null, s1 = null, s2 = null;
        Servo h = null;
        WebcamName camera = null;

        final ElapsedTime timer = new ElapsedTime();
        boolean wasShootingLastTick = false;

        try { s1 = hardwareMap.dcMotor.get( "flywheel1"); } catch (Exception e) { telemetry.addLine("Missing: flywheel1"); }
        try { s2 = hardwareMap.dcMotor.get( "flywheel2"); } catch (Exception e) {telemetry.addLine("Mising: flywheel2"); }
        try { st = hardwareMap.dcMotor.get("storage"); } catch (Exception e) { telemetry.addLine("Missing: storage"); }
        try { i = hardwareMap.dcMotor.get("intake"); } catch (Exception e) { telemetry.addLine("Missing: intake"); }
        try { h = hardwareMap.get(Servo.class, "hood"); } catch (Exception e) { telemetry.addLine("Missing: hood"); }

        try { camera = hardwareMap.get(WebcamName.class, "Webcam 1"); } catch (Exception e) { telemetry.addLine("Missing: Webcam 1"); }

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.update();

        if (s1 != null) s1.setDirection(DcMotorSimple.Direction.FORWARD);
        if (s2 != null) s2.setDirection(DcMotorSimple.Direction.REVERSE);
        if (st != null) st.setDirection(DcMotorSimple.Direction.FORWARD);
        if (i != null) i.setDirection(DcMotorSimple.Direction.FORWARD);


        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, new Pose2d(0,0,0));

        FlywheelIO flywheel= (s1 != null && s2 != null) ? new FlywheelIO(s1, s2) : null;
        if (flywheel!= null) {flywheel.setRampRate(Constants.SHOOTER_RAMP_RATE);}

        IntakeIO intake = (i != null) ? new IntakeIO(i) : null;

        StorageIO storage = (st != null) ? new StorageIO(st) : null;

        HoodIO hood = (h != null) ? new HoodIO(h) : null;

        VisionIO vision = (camera != null) ? new VisionIO(camera) : null;

        VisionLocalize localize = (vision != null) ? new VisionLocalize(vision) : null;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (drive != null) {
                // Drive is now field-centric
                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                //Joystick input
                double inputY = -gamepad1.left_stick_y;
                double inputX = gamepad1.left_stick_x;
                double inputRx = gamepad1.right_stick_x;

                double headingRadians = currentPose.heading.toDouble();
                double rotatedX = inputX * Math.cos(-headingRadians) - inputY * Math.sin(-headingRadians);
                double rotatedY = inputX * Math.sin(-headingRadians) + inputY * Math.cos(-headingRadians);

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(rotatedY, -rotatedX),
                        inputRx
                ));

                // reset field centric drive
                if (gamepad1.dpadDownWasPressed()) {
                    drive.localizer.setPose(new Pose2d(0, 0, 0));
                }
            }

            if (intake != null) intake.updateIntake(gamepad1);
            if (flywheel!= null) flywheel.updateShooter(gamepad1);
            if (storage != null) storage.updateStorage(gamepad1);
            if (localize != null) localize.update();

            // Shooting lookup table based on AprilTag distance
            double distance = 0;
            if (vision != null) {
                AprilTagDetection detection = vision.getTagDistance(Constants.ALLIANCE_TAG_ID);
                distance = (detection != null) ? detection.ftcPose.range : 0.0;
            }

            if (distance > 0) {
                double targetVelocity = ShootingLookupTable.getFlywheelPower(distance);
                double targetHoodAngle = ShootingLookupTable.getHoodAngle(distance);

                if (flywheel != null) {
                    flywheel.setShootingPower(targetVelocity);
                }
                if (hood != null) {
                    hood.setHoodAngle(targetHoodAngle);
                }
            }

            if (flywheel != null && flywheel.isShooting() && !wasShootingLastTick) {
                timer.reset();
            }
            wasShootingLastTick = flywheel != null && flywheel.isShooting();

            if (storage != null) {
                if (flywheel != null && flywheel.isShooting()) {
                    if (timer.milliseconds() >= 2000) {
                        storage.setPower(1);
                    } else {
                        storage.setPower(0);
                    }
                } else if (intake != null && intake.isIntaking()) {
                    storage.setPower(1);
                } else {
                    storage.setPower(0);
                }
            }

            telemetry.addLine(imu.getRobotYawPitchRollAngles().toString());
            telemetry.addLine(Double.toString(distance));
            telemetry.addLine(Double.toString(ShootingLookupTable.getFlywheelPower(distance)));
            telemetry.addLine(Double.toString(ShootingLookupTable.getHoodAngle(distance)));

            telemetry.update();
        }
        if (vision != null) vision.stop();
    }
}
