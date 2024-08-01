package com.team957.comp2024.commands;

import com.team957.comp2024.Constants;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import monologue.Annotations.IgnoreLogged;
import monologue.Logged;

public class OnTheFlyPathing implements Logged {
    @IgnoreLogged public static final OnTheFlyPathing instance = new OnTheFlyPathing();

    private OnTheFlyPathing() {}

    public Command otfPathingCommand(
            Swerve swerve, Supplier<Pose2d> setpoint, Supplier<Pose2d> localization) {
        PID xController = new PID(Constants.AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        PID yController = new PID(Constants.AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        PID thetaController =
                new PID(Constants.AutoConstants.ROTATIONAL_PATHFINDING_GAINS, 0, true);

        return swerve.getFieldRelativeControlCommand(
                () -> {
                    Pose2d sp = setpoint.get();
                    Pose2d loc = localization.get();

                    log("poseSetpoint", sp);

                    xController.setSetpoint(sp.getX());
                    yController.setSetpoint(sp.getY());

                    thetaController.setSetpoint(sp.getRotation().getRadians());

                    return new ChassisSpeeds(
                            xController.calculate(loc.getX()),
                            yController.calculate(loc.getY()),
                            thetaController.calculate(loc.getRotation().getRadians()));
                },
                () -> localization.get().getRotation());
    }
}
