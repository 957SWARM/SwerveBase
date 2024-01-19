package com.team957.comp2024;

import com.ctre.phoenix6.SignalLogger;
import com.team957.comp2024.Constants.PDHConstants;
import com.team957.comp2024.commands.ChoreoFollowing;
import com.team957.comp2024.subsystems.IMU;
import com.team957.comp2024.subsystems.PDH;
import com.team957.comp2024.subsystems.PneumaticsHub;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.util.SwarmChoreo;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot implements Logged {
    // these need to be constructed so that
    // monologue can work its reflection magic
    private final IMU imu = new IMU();

    @SuppressWarnings("unused")
    private final PDH pdh = new PDH(PDHConstants.STARTING_SWITCHABLE_CHANNEL_STATE);

    @SuppressWarnings("unused")
    private final PneumaticsHub ph = new PneumaticsHub();

    private final Swerve swerve = Swerve.getSwerve(isReal());

    private final DeltaTimeUtil dt = new DeltaTimeUtil();

    private final Localization localization =
            new Localization(
                    swerve::getStates, swerve::getPositions, imu::getCorrectedAngle, !isReal());

    private final XboxController controller = new XboxController(0);

    private final Command teleopDrive =
            swerve.getFieldRelativeControlCommand(
                    () -> {
                        return new ChassisSpeeds(
                                -5 * controller.getLeftY(),
                                -5 * controller.getLeftX(),
                                10 * controller.getLeftTriggerAxis());
                    },
                    localization::getRotationEstimate);

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();

        if (isReal()) URCL.start(); // segfaults in sim

        Monologue.setupMonologue(this, "Robot", false, false);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        log("loopTimeSeconds", dt.getTimeSecondsSinceLastCall());

        Monologue.updateAll();

        localization.update();
    }

    @Override
    public void teleopInit() {
        teleopDrive.schedule();
    }

    @Override
    public void autonomousInit() {
        ChoreoFollowing.getPathFollowingCommand(
                        swerve,
                        SwarmChoreo.getTrajectory("TestPath"),
                        localization::getPoseEstimate)
                .schedule();
    }
}
