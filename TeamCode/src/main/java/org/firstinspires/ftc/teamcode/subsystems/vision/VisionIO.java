package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionIO {
    private WebcamName cameraName;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    public VisionIO(WebcamName webcamName) {
        this.cameraName = webcamName;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(this.cameraName)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public AprilTagDetection getTagDistance(int targetId) {
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }

        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public boolean isActive() {
        return visionPortal != null &&
                visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public List<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }
}
