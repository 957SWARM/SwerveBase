package com.team957.comp2024.peripherals;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.team957.lib.math.UtilityMath;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Annotations.Log;
import monologue.Logged;

public class IMU implements Logged {
    private final Pigeon2 pigeon = new Pigeon2(0);

    private final DeltaTimeUtil dtUtil;

    private double previousAngleRadians = 957;
    // radian value is clamped between -pi and pi,
    // so this basically serves as NaN when we can't use it here

    private double yawVelocity = 0;

    private Rotation2d angleOffset = new Rotation2d();
    // this is the number ADDED to the raw value to get the corrected angle

    public IMU() {
        dtUtil = new DeltaTimeUtil();
    }

    public void setAngleOffset(Rotation2d angleOffset) {
        previousAngleRadians = 957;
        // changing the offset totally messes up velocity calculations, so we don't count this next
        // iteration
        this.angleOffset = angleOffset;
    }

    @Log.NT
    public Rotation2d getAngleOffset() {
        return angleOffset;
    }

    /** Sets the yaw offset such that the currrent state is "zero". */
    public void setAngleToZero() {
        angleOffset = angleOffset.minus(getCorrectedAngle());
    }

    @Log.NT
    public Rotation2d getCorrectedAngle() {
        return getRawAngle().plus(angleOffset);
    }

    @Log.NT
    public Rotation2d getRawAngle() {
        // this is an object allocation, which may need to remove for loop time reasons
        return Rotation2d.fromDegrees(-pigeon.getAngle());
    }

    @Log.NT
    public double getAngularVelocityRadiansperSecond() {
        return yawVelocity;
    }

    public void periodic() {
        double currentRotation =
                UtilityMath.normalizeAngleRadians(getCorrectedAngle().getRadians());

        double delta;

        if (previousAngleRadians != 957) {
            delta = UtilityMath.smallestAngleRadiansBetween(previousAngleRadians, currentRotation);
        } else delta = 0;

        double dt = dtUtil.getTimeSecondsSinceLastCall();

        if (dt != 0) yawVelocity = delta / dt;
        else yawVelocity = 0; // dt == 0 should never happen, but who knows?
    }
}
