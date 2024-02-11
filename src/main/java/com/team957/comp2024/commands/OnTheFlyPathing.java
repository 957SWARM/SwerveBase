package com.team957.comp2024.commands;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Robot;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class OnTheFlyPathing {
    public static Command otfPathingCommand(
            Swerve swerve, Supplier<Pose2d> setpoint, Supplier<Pose2d> localization) {
        // naive (but computationally performant compared to actual numerical optimization)
        // generation of one linear motion profile for each linear DoF and one for angular
        // totally oblivious to obstacles and silly things like what is physically possible

        TrapezoidProfile linearProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.OnTheFlyPathingConstants.OTF_MAX_LINEAR_VEL,
                                Constants.OnTheFlyPathingConstants.OTF_MAX_LINEAR_ACCEL));
        // no fancy exponential profiles because the swerve does not act at all like a dc motor with
        // all the corners we cut

        TrapezoidProfile angularProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.OnTheFlyPathingConstants.OTF_MAX_ANGULAR_VEL,
                                Constants.OnTheFlyPathingConstants.OTF_MAX_ANGULAR_ACCEL));

        PID xController = new PID(Constants.AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        PID yController = new PID(Constants.AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        PID thetaController =
                new PID(Constants.AutoConstants.ROTATIONAL_PATHFINDING_GAINS, 0, true);

        // i love java
        State reusableCurrentState = new State();
        State reusableGoalState = new State();

        return swerve.getChassisRelativeControlCommand(
                () -> {
                    Pose2d actualPose = localization.get();
                    Pose2d finalPose = setpoint.get();

                    double rotationRadians = actualPose.getRotation().getRadians();

                    double chassisRelVX =
                            swerve.getForwardKinematicChassisSpeeds().vxMetersPerSecond;
                    double chassisRelVY =
                            swerve.getForwardKinematicChassisSpeeds().vyMetersPerSecond;

                    // have to transform chassis-relative ik into world-relative
                    reusableCurrentState.position = actualPose.getX();
                    reusableCurrentState.velocity =
                            (chassisRelVX * Math.cos(rotationRadians))
                                    - (chassisRelVY * Math.sin(rotationRadians));

                    reusableGoalState.position = finalPose.getX();

                    double xProfiled =
                            linearProfile.calculate(
                                            Robot.kDefaultPeriod,
                                            reusableCurrentState,
                                            reusableGoalState)
                                    .position;

                    reusableCurrentState.position = actualPose.getY();
                    reusableCurrentState.velocity =
                            (chassisRelVY * Math.cos(rotationRadians))
                                    + (chassisRelVX * Math.sin(rotationRadians));

                    reusableGoalState.position = finalPose.getY();

                    double yProfiled =
                            linearProfile.calculate(
                                            Robot.kDefaultPeriod,
                                            reusableCurrentState,
                                            reusableGoalState)
                                    .position;

                    xController.setSetpoint(xProfiled);
                    yController.setSetpoint(yProfiled);

                    // TODO: figure out profiling for wrapped angle and then finish?

                    return new ChassisSpeeds(
                            xController.calculate(actualPose.getX()),
                            yController.calculate(actualPose.getY()),
                            thetaController.calculate(actualPose.getRotation().getRadians()));
                });
    }
}
