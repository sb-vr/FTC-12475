package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalize;

@TeleOp
public class OpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor s1 = null, s2 = null, i = null, st = null;
        WebcamName camera = null;

        try { s1 = hardwareMap.dcMotor.get("flywheel1"); } catch (Exception e) { telemetry.addLine("Missing: flywheel1"); }
        try { s2 = hardwareMap.dcMotor.get("flywheel2"); } catch (Exception e) {telemetry.addLine("Mising: flywheel2"); }
        try { st = hardwareMap.dcMotor.get("storage"); } catch (Exception e) { telemetry.addLine("Missing: storage"); }
        try { i = hardwareMap.dcMotor.get("intake"); } catch (Exception e) { telemetry.addLine("Missing: intake"); }

        try { camera = hardwareMap.get(WebcamName.class, "Webcam 1"); } catch (Exception e) { telemetry.addLine("Missing: Webcam 1"); }

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.update();

        if (s1 != null) s1.setDirection(DcMotorSimple.Direction.FORWARD);
        if (s2 != null) s2.setDirection(DcMotorSimple.Direction.REVERSE);
        if(st != null) st.setDirection(DcMotorSimple.Direction.REVERSE);
        if (i != null) i.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, new Pose2d(0,0,0));

        FlywheelIO flywheel= (s1 != null && s2 != null) ? new FlywheelIO(s1, s2) : null;
        if (flywheel!= null) {flywheel.setRampRate(Constants.SHOOTER_RAMP_RATE); flywheel.setShootingPower(Constants.DEFAULT_SHOOTER_SPEED);}

        IntakeIO intake = (i != null) ? new IntakeIO(i) : null;

        StorageIO storage = (st != null) ? new StorageIO(st) : null;

        VisionIO vision = (camera != null) ? new VisionIO(camera) : null;

        VisionLocalize localize = (vision != null) ? new VisionLocalize(vision) : null;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (drive != null) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
                drive.updatePoseEstimate();
            }

            if (intake != null) intake.updateIntake(gamepad1);
            if (flywheel!= null) flywheel.updateShooter(gamepad1);
            if (storage != null) storage.updateStorage(gamepad1);
            if (localize != null) localize.update();

            if (vision != null) {
                double tagDistance = vision.getTagDistance(20).ftcPose.range;
                telemetry.addLine(Double.toString(tagDistance));
            }
        }

        telemetry.addLine(imu.getRobotYawPitchRollAngles().toString());
        telemetry.update();
        if (vision != null) vision.stop();
    }

}
