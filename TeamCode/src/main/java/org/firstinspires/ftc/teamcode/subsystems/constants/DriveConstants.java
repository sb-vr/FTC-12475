package org.firstinspires.ftc.teamcode.subsystems.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DriveConstants {
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // drive model parameters
    public double inPerTick = (java.lang.Math.PI * (32.0 / 25.4)) / 2000;
    public double lateralInPerTick = 0.0012495431986690628;
    public double trackWidthTicks = 5388.626357451902;

    // feedforward parameters (in tick units)
    public double kS = 0.9252953786341847;
    public double kV = 0.000280281340167147;
    public double kA = 0.00018;

    // path profile parameters (in inches)
    public double maxWheelVel = 50;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public double maxAngVel = java.lang.Math.PI; // shared with path
    public double maxAngAccel = java.lang.Math.PI;

    // path controller gains
    public double axialGain = 2;
    public double lateralGain = 6;
    public double headingGain = 1; // shared with turn

    public double axialVelGain = 0.0;
    public double lateralVelGain = 0.0;
    public double headingVelGain = 0.0; // shared with turn
}
