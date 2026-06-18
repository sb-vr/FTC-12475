package org.firstinspires.ftc.teamcode.roadrunnerMeuk.messages;

import java.util.List;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class TankLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair[] left;
    public PositionVelocityPair[] right;

    public TankLocalizerInputsMessage(List<PositionVelocityPair> left, List<PositionVelocityPair> right) {
        this.timestamp = System.nanoTime();
        this.left = left.toArray(new PositionVelocityPair[0]);
        this.right = right.toArray(new PositionVelocityPair[0]);
    }
}
