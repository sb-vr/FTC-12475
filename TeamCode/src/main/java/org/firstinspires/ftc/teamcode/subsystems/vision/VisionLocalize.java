package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class VisionLocalize {
    private VisionIO vision;

    private Pose3D lastPose;

    public VisionLocalize(VisionIO vision) {
        this.vision = vision;
    }

    public void update() {
        List<AprilTagDetection> detections = vision.getDetections();

        if (detections.size() > 0) {
            for (AprilTagDetection tag : detections) {
                if (tag.metadata != null && !tag.metadata.name.contains("Obelisk")) {
                    lastPose = tag.robotPose;
                    break;
                }
            }
        }
    }

    public Pose3D getLastPose() {
        return lastPose;
    }
}
