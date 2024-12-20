package com.team957.comp2024.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.commands.TrajectoryFollowing.CommandWithTime;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class Autos {
    private class AutoPhaseFactory {
        private final Swerve swerve;
        private final IntakePivot pivot;
        private final ArrayList<ChoreoTrajectory> traj;
        private final LLlocalization localization;
        private final Supplier<Alliance> alliance;

        AutoPhaseFactory(
                Swerve swerve,
                IntakePivot pivot,
                ArrayList<ChoreoTrajectory> traj,
                LLlocalization localization,
                Supplier<Alliance> alliance) {
            this.swerve = swerve;
            this.pivot = pivot;
            this.traj = traj;
            this.localization = localization;
            this.alliance = alliance;
        }

        Command shootTrajectoryPhase(
                int phaseIndex, boolean resetPose, double shotHeadStartSeconds, double endTimeout) {
            ChoreoTrajectory trajPhase;

            try {
                trajPhase = traj.get(phaseIndex);
            } catch (IndexOutOfBoundsException e) {
                return new InstantCommand();
            }

            CommandWithTime following =
                    TrajectoryFollowing.instance.getPathFollowingCommand(
                            swerve, trajPhase, localization, resetPose, alliance);

            double startShotDelay;

            if (shotHeadStartSeconds < 0) startShotDelay = following.timeSeconds();
            else if (shotHeadStartSeconds > following.timeSeconds()) startShotDelay = 0;
            else startShotDelay = following.timeSeconds() - shotHeadStartSeconds;

            return following
                    .command()
                    .alongWith(
                            new WaitCommand(startShotDelay)
                                    .andThen(
                                            ScoringSequences.coordinatedSubwooferShot(
                                                    shooter, intakePivot, intakeRoller)))
                    .withTimeout(following.timeSeconds() + endTimeout);
        }

        Command floorTrajectoryPhase(
                int phaseIndex, boolean resetPose, double pivotHeadStart, double timeout) {
            ChoreoTrajectory trajPhase;

            try {
                trajPhase = traj.get(phaseIndex);
            } catch (IndexOutOfBoundsException e) {
                return new InstantCommand();
            }

            return new WaitCommand(pivotHeadStart > 0 ? pivotHeadStart : 0)
                    .andThen(
                            TrajectoryFollowing.instance
                                    .getPathFollowingCommand(
                                            swerve, trajPhase, localization, resetPose, alliance)
                                    .command())
                    .alongWith(ScoringSequences.coordinatedFloorIntake(intakePivot, intakeRoller))
                    .withTimeout(timeout > 0 ? timeout : 0);
        }

        Command stowTrajectoryPhase(int phaseIndex, boolean resetPose) {
            ChoreoTrajectory trajPhase;

            try {
                trajPhase = traj.get(phaseIndex);
            } catch (IndexOutOfBoundsException e) {
                return new InstantCommand();
            }

            return TrajectoryFollowing.instance
                    .getPathFollowingCommand(swerve, trajPhase, localization, resetPose, alliance)
                    .command()
                    .alongWith(pivot.toStow());
        }

        Command driveTrajectoryPhase(int phaseIndex, boolean resetPose) {
            ChoreoTrajectory trajPhase;

            try {
                trajPhase = traj.get(phaseIndex);
            } catch (IndexOutOfBoundsException e) {
                return new InstantCommand();
            }

            return TrajectoryFollowing.instance
                    .getPathFollowingCommand(swerve, trajPhase, localization, resetPose, alliance)
                    .command();
        }
    }

    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Shooter shooter;
    private final LLlocalization localization;
    private final Supplier<Alliance> alliance;

    private final Alert loadFailureAlert =
            new Alert("Autonomous trajectory failed to load!", AlertType.ERROR);

    public Autos(
            Swerve swerve,
            IntakePivot intakePivot,
            IntakeRoller intakeRoller,
            Shooter shooter,
            LLlocalization localization,
            Supplier<Alliance> alliance) {
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.shooter = shooter;
        this.localization = localization;
        this.alliance = alliance;
    }

    private Optional<ArrayList<ChoreoTrajectory>> safeLoadTrajectory(String trajName) {
        var trajPhases = Choreo.getTrajectoryGroup(trajName);

        if (trajPhases == null) {
            loadFailureAlert.set(true);
            return Optional.empty();
        } else {
            loadFailureAlert.set(false);
            return Optional.of(trajPhases);
        }
    }

    public Command doNothingAuto() {
        return new InstantCommand();
    }

    private Command singleTrajectoryOnlyAuto(String trajName, boolean resetPoseToInitial) {
        var maybeTraj = safeLoadTrajectory(trajName);

        if (!maybeTraj.isPresent()) return new InstantCommand();

        return TrajectoryFollowing.instance
                .getPathFollowingCommand(
                        swerve, maybeTraj.get().get(0), localization, resetPoseToInitial, alliance)
                .command();
    }

    public Command justLeaveAmp() {
        return singleTrajectoryOnlyAuto("justLeaveAmp", true);
    }

    public Command justLeaveCenter() {
        return singleTrajectoryOnlyAuto("justLeaveCenter", true);
    }

    public Command justLeaveSource() {
        return singleTrajectoryOnlyAuto("justLeaveSource", true);
    }

    public Command testPath() {
        return singleTrajectoryOnlyAuto("testPath", true);
    }

    public Command fivePieceMockup() {
        return singleTrajectoryOnlyAuto("fivePieceMockup", true);
    }

    public Command fourPieceMockup() {
        return singleTrajectoryOnlyAuto("fourPieceMockup", true);
    }

    public Command threePieceMockup() {
        return singleTrajectoryOnlyAuto("threePieceMockup", true);
    }

    public Command centerTwoPiece() {
        var maybeTraj = safeLoadTrajectory("centerTwoPiece");

        if (!maybeTraj.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, maybeTraj.get(), localization, alliance);

        return ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                .withTimeout(1)
                .andThen(factory.floorTrajectoryPhase(0, true, .25, 1))
                .andThen(factory.shootTrajectoryPhase(1, false, .25, .75))
                .andThen(factory.stowTrajectoryPhase(2, false));
    }

    public Command centerFourPiece() {
        var maybeTraj = safeLoadTrajectory("centerFourPiece");

        if (!maybeTraj.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, maybeTraj.get(), localization, alliance);

        return ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                .withTimeout(1)
                .andThen(factory.floorTrajectoryPhase(0, true, 0, 2))
                .andThen(factory.shootTrajectoryPhase(1, true, .5, .75))
                .andThen(factory.floorTrajectoryPhase(2, false, 0.25, 2))
                .andThen(factory.shootTrajectoryPhase(3, false, 0.5, 0.75))
                .andThen(factory.floorTrajectoryPhase(4, false, 0, 2))
                .andThen(factory.shootTrajectoryPhase(5, false, 0.25, 1))
                .andThen(factory.stowTrajectoryPhase(6, false));
    }

    public Command sourceFarThreePiece() {
        var maybeTraj = safeLoadTrajectory("sourceFarThreePiece");

        if (!maybeTraj.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, maybeTraj.get(), localization, alliance);

        return ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                .withTimeout(1)
                .andThen(factory.floorTrajectoryPhase(0, true, 0, 5))
                .andThen(factory.shootTrajectoryPhase(1, false, 0.5, 0.75))
                .andThen(factory.floorTrajectoryPhase(2, false, .25, 5))
                .andThen(factory.shootTrajectoryPhase(3, false, 0.5, 0.75))
                .andThen(factory.stowTrajectoryPhase(4, false));
    }

    public Command sourceTwoPiece() {
        var maybeTraj = safeLoadTrajectory("sourceTwoPiece");

        if (!maybeTraj.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, maybeTraj.get(), localization, alliance);

        return ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                .withTimeout(1)
                .andThen(factory.floorTrajectoryPhase(0, true, 0, 5))
                .andThen(factory.shootTrajectoryPhase(1, false, 0.5, 0.75));
    }

    public Command justShoot() {
        return ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                .withTimeout(1);
    }

    // avoids going wide, more straight lines
    public Command narrowFourPiece() {
        var maybeTraj = safeLoadTrajectory("narrowCenterFourPiece");

        if (!maybeTraj.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, maybeTraj.get(), localization, alliance);

        return factory.driveTrajectoryPhase(0, false)
                .andThen(factory.driveTrajectoryPhase(1, false))
                .andThen(factory.driveTrajectoryPhase(2, false))
                .andThen(factory.driveTrajectoryPhase(3, false))
                .andThen(factory.driveTrajectoryPhase(4, false))
                .andThen(factory.driveTrajectoryPhase(5, false))
                .andThen(factory.driveTrajectoryPhase(6, false));
    }
}
