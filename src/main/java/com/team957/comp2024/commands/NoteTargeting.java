package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.util.LimelightLib;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class NoteTargeting {

    private final Swerve swerve;
    private final LLlocalization poseEstimation;
    private final String limelightName;

    private double map(
            double input,
            double inputStart,
            double inputEnd,
            double outputStart,
            double outputEnd) {
        double slope = 1 * (outputEnd - outputStart) / (inputEnd - inputStart);
        return outputStart + slope * (input - inputStart);
    }

    public NoteTargeting(Swerve swerve, LLlocalization poseEstimation, String limelightName) {
        this.swerve = swerve;
        this.poseEstimation = poseEstimation;
        this.limelightName = limelightName;
    }

    // AUTO AIMS TO NOTE IF NOTE IS TRACKABLE
    public Command getNoteTrackCommand(
            Supplier<Double> swerveX,
            Supplier<Double> swerveY,
            Supplier<Double> swerveRot,
            Supplier<Double> fieldRelRotationOffset) {

        return swerve.getFieldRelativeControlCommand(
                        () -> {
                            return new ChassisSpeeds(
                                    swerveX.get(),
                                    swerveY.get(),
                                    -getTrackingAngle(getTargetAngle()));
                        },
                        () ->
                                poseEstimation
                                        .getRotationEstimate()
                                        .minus(new Rotation2d(fieldRelRotationOffset.get())))
                .unless(() -> !checkTarget())
                .withName("noteTargeting");
    }

    private double getTrackingAngle(double targetAngle) {
        double kp = VisionConstants.TRACKING_KP;
        double minCommand = VisionConstants.TRACKING_MIN_COMMAND;
        if (Math.abs(targetAngle) > VisionConstants.TRACKING_STOP_THRESHOLD) {
            if (targetAngle > 0 && targetAngle < VisionConstants.MIN_COMMAND_TRESHOLD) {
                return -minCommand;
            } else if (targetAngle < 0 && targetAngle > -VisionConstants.MIN_COMMAND_TRESHOLD) {
                return minCommand;
            } else {
                return kp * targetAngle;
            }
        }
        return 0;
    }

    private double getTargetAngle() {
        double x = getNotePose2dRobot().getX();
        double y = getNotePose2dRobot().getY();
        double c = Math.sqrt((Math.abs(x) * Math.abs(x)) + (y * y));

        double targetAngle = Math.asin(x / c) - Math.PI / 2;
        if (y < 0) {
            return targetAngle;
        } else {
            return -targetAngle;
        }
    }

    @SuppressWarnings("unused")
    private Pose2d getNotePose2dField() { // QUESTIONABLE... ALSO HAVENT ADDED ROBOT POSE
        double c =
                Math.sqrt(
                        (Math.abs(getNotePose2dRobot().getX())
                                        * Math.abs(getNotePose2dRobot().getX()))
                                + (getNotePose2dRobot().getY() * getNotePose2dRobot().getY()));
        double b = Math.asin(getNotePose2dRobot().getY() / c);
        double a = poseEstimation.getRotationEstimate().getRadians();
        double targetAngle = Math.abs(Units.degreesToRadians(90) - b - Math.abs(a));
        double targetYF = c * Math.sin(targetAngle);
        double targetXF = Math.sqrt((c * c) - (targetYF * targetYF));
        return new Pose2d(
                targetXF, targetYF, new Rotation2d(targetAngle)); // ROTATION SHOULDNT MATTER?
    }

    private Pose2d getNotePose2dRobot() {

        double groundDistance = getNoteGroundDistance();
        double tx = LimelightLib.getTX(limelightName);
        double targetYLL;
        double targetXLL;
        if (tx > 0) {
            targetYLL = -(groundDistance * Math.sin(Units.degreesToRadians(Math.abs(tx))));
        } else {
            targetYLL = (groundDistance * Math.sin(Units.degreesToRadians(Math.abs(tx))));
        }
        targetXLL =
                Math.sqrt(
                        (groundDistance * groundDistance)
                                - (Math.abs(targetYLL) * Math.abs(targetYLL)));

        double targetXRobot =
                targetXLL + Units.metersToInches(VisionConstants.LL1_TO_CENTER.getX());
        double targetYRobot =
                targetYLL + Units.metersToInches(VisionConstants.LL1_TO_CENTER.getY());

        Pose2d targetPose =
                new Pose2d(new Translation2d(targetXRobot, targetYRobot), new Rotation2d());

        return targetPose;
    }

    private double getNoteDistance() {
        double tx = LimelightLib.getTX(limelightName);
        double thor = LimelightLib.getTHOR(limelightName);
        double txp =
                map(
                        tx,
                        -(VisionConstants.LL_FOV_DEGREES / 2),
                        (VisionConstants.LL_FOV_DEGREES / 2),
                        0,
                        VisionConstants.LL_FOV_PIXELS);
        double txpOffset = txp + (thor / 2);
        double txOffset =
                map(
                        txpOffset,
                        0,
                        VisionConstants.LL_FOV_PIXELS,
                        -(VisionConstants.LL_FOV_DEGREES / 2),
                        (VisionConstants.LL_FOV_DEGREES / 2));
        double angle = Math.abs(txOffset - tx);

        double c = (VisionConstants.NOTE_WIDTH / 2) / Math.sin(Units.degreesToRadians(angle));

        double distance =
                Math.sqrt(
                        (c * c)
                                - ((VisionConstants.NOTE_WIDTH / 2)
                                        * (VisionConstants.NOTE_WIDTH / 2)));

        // super secret krabby patty distance formula 4
        double distanceCorrected =
                distance - 10; // (distance - ((1.15 * (distance - 5)) - distance)) * .95;

        if (checkTarget()) {
            return distanceCorrected;
        }
        return 0;
    }

    private double getNoteGroundDistance() {
        double distance = getNoteDistance();
        double groundDistance =
                Math.sqrt(
                        (distance * distance)
                                - (VisionConstants.LL1_TO_CENTER.getZ()
                                        * VisionConstants.LL1_TO_CENTER.getZ()));
        return groundDistance;
    }

    private boolean checkTarget() { // CHECKS IF NOTE IS TRACKABLE
        boolean trackable = true;
        /*if (!LimelightLib.getTV(limelightName)) {
            trackable = false;
        } else if ((Math.abs(LimelightLib.getTX(limelightName))
                > VisionConstants.TARGET_TX_CUTOFF)) {
            trackable = false;
        } else if ((Math.abs(LimelightLib.getTHOR(limelightName))
                < VisionConstants.TARGET_THOR_CUTOFF)) {
            trackable = false;
        }*/
        return trackable;
    }
}
