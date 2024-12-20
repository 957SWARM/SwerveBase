package com.team957.comp2024.commands;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.AutoConstants;
import com.team957.comp2024.UI;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;

public class TrajectoryFollowing implements Logged {
    public static record CommandWithTime(Command command, double timeSeconds) {}
    ;

    // being instance is slightly janky but needed to have a monologue context, but singleton works

    @IgnoreLogged public static final TrajectoryFollowing instance = new TrajectoryFollowing();

    private TrajectoryFollowing() {}

    // split out for logging/ command state
    @Log.NT private Pose2d poseSetpoint = new Pose2d();

    @Log.NT
    private ChoreoTrajectoryState referenceState =
            new ChoreoTrajectoryState(0.01, 0, 0, 0, 0, 0, 0);

    private ChoreoControlFunction alternateControlFunction(PID x, PID y, PID theta) {
        return (pose, referenceState) -> {
            this.referenceState = referenceState;

            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double tFF = referenceState.angularVelocity;

            this.log("referenceChassisSpeeds", new ChassisSpeeds(xFF, yFF, tFF));

            x.setSetpoint(referenceState.x);
            double xFB = x.calculate(pose.getX());

            y.setSetpoint(referenceState.y);
            double yFB = y.calculate(pose.getY());

            theta.setSetpoint(referenceState.heading);
            double tFB = theta.calculate(pose.getRotation().getRadians());

            // negating angular is janky
            return new ChassisSpeeds(xFF + xFB, yFF + yFB, -(tFF + tFB));
        };
    }

    public CommandWithTime getPathFollowingCommand(
            Swerve swerve,
            ChoreoTrajectory trajectory,
            LLlocalization localization,
            boolean resetPoseToInitial,
            Supplier<Alliance> alliance) {

        final PID xController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID yController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID thetaController = new PID(AutoConstants.ROTATIONAL_PATHFINDING_GAINS, 0, true);

        final ChoreoControlFunction controlFunction =
                alternateControlFunction(xController, yController, thetaController);

        final Timer timer = new Timer();

        final ChoreoTrajectory flipped = trajectory.flipped();
        final Supplier<ChoreoTrajectory> getMirroredPath =
                () -> {
                    return (alliance.get() == Alliance.Blue) ? trajectory : flipped;
                };

        return new CommandWithTime(
                Commands.runOnce(
                                () -> {
                                    timer.restart();
                                    if (resetPoseToInitial)
                                        localization.setPose(
                                                getMirroredPath.get().getInitialPose());
                                })
                        // .andThen(Commands.runOnce((this.log("trajectory",trajectory))))
                        // choreotrajectory not supported in monologue :(
                        .andThen(
                                swerve.getFieldRelativeControlCommand(
                                                () ->
                                                        controlFunction.apply(
                                                                localization.getPoseEstimate(),
                                                                getMirroredPath
                                                                        .get()
                                                                        .sample(timer.get())),
                                                localization::getRotationEstimate)
                                        .alongWith(
                                                Commands.run(
                                                        () -> {
                                                            Pose2d pose =
                                                                    getMirroredPath
                                                                            .get()
                                                                            .sample(timer.get())
                                                                            .getPose();

                                                            poseSetpoint = pose;

                                                            UI.instance.setSetpointPose(pose);
                                                        })))
                        .until(
                                () ->
                                        getMirroredPath.get().getTotalTime()
                                                < timer.get()
                                                        - Constants.AutoConstants
                                                                .PROFILE_OVERRUN_TOLERANCE_SECONDS)
                        .withName("choreoFollowing"),
                getMirroredPath.get().getTotalTime());
    }

    // drives the specified translation, relative to the current localized position. The setpoint
    // pose is locked in at command scheduling, relative to the localized pose at that time.
    public Command driveToRelativePose(
            Swerve swerve, Supplier<Pose2d> localization, Supplier<Transform2d> setpoint) {
        final PID xController = new PID(AutoConstants.LINEAR_GAINS_NO_FF, 0);
        final PID yController = new PID(AutoConstants.LINEAR_GAINS_NO_FF, 0);
        final PID thetaController = new PID(AutoConstants.ROTATIONAL_GAINS_NO_FF, 0);

        final ChoreoControlFunction controlFunc =
                alternateControlFunction(xController, yController, thetaController);

        return Commands.runOnce(
                        () -> {
                            poseSetpoint = localization.get().transformBy(setpoint.get());
                            referenceState =
                                    new ChoreoTrajectoryState(
                                            0,
                                            poseSetpoint.getX(),
                                            poseSetpoint.getY(),
                                            poseSetpoint.getRotation().getRadians(),
                                            0,
                                            0,
                                            0);
                        })
                .alongWith(
                        swerve.getFieldRelativeControlCommand(
                                () -> controlFunc.apply(localization.get(), referenceState),
                                () -> localization.get().getRotation()));
    }
}
