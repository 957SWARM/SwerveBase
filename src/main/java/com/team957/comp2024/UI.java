package com.team957.comp2024;

import com.team957.comp2024.input.DefaultDriver;
import com.team957.comp2024.input.DefaultOperator;
import com.team957.comp2024.input.DriverInput;
import com.team957.comp2024.input.OperatorInput;
import com.team957.comp2024.input.SimKeyboardDriver;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Consumer;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class UI implements Logged {
    private SwerveModuleState[] moduleStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };

    @IgnoreLogged public static final UI instance = new UI();

    private double rotation = 0;

    private double currentAmps = 0;
    private double voltage = 0;

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final Sendable swerveVis =
            new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                    builder.setSmartDashboardType("SwerveDrive");

                    builder.addDoubleProperty(
                            "Front Left Angle", () -> moduleStates[0].angle.getRadians(), null);
                    builder.addDoubleProperty(
                            "Front Left Velocity",
                            () -> moduleStates[0].speedMetersPerSecond,
                            null);

                    builder.addDoubleProperty(
                            "Front Right Angle", () -> moduleStates[1].angle.getRadians(), null);

                    builder.addDoubleProperty(
                            "Front Right Velocity",
                            () -> moduleStates[1].speedMetersPerSecond,
                            null);

                    builder.addDoubleProperty(
                            "Back Right Angle", () -> moduleStates[2].angle.getRadians(), null);
                    builder.addDoubleProperty(
                            "Back Right Velocity",
                            () -> moduleStates[2].speedMetersPerSecond,
                            null);

                    builder.addDoubleProperty(
                            "Back Left Angle", () -> moduleStates[3].angle.getRadians(), null);

                    builder.addDoubleProperty(
                            "Back Left Velocity", () -> moduleStates[3].speedMetersPerSecond, null);

                    builder.addDoubleProperty("Robot Angle", () -> rotation, null);
                }
            };

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final Field2d fieldVis = new Field2d();

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final Mechanism2d intakeVis = new Mechanism2d(2, 2);

    private final MechanismLigament2d intakeVisBody =
            intakeVis
                    .getRoot("root", 1.25, .25)
                    .append(
                            new MechanismLigament2d(
                                    "intake",
                                    Constants.IntakePivotConstants.PIVOT_TO_TIP_METERS,
                                    0));

    private final Alert overcurrent = new Alert("General overcurrent!", AlertType.WARNING);

    private final Alert highResistance =
            new Alert("High main power path resistance!", AlertType.WARNING);

    private final Alert lowVoltage = new Alert("Low battery voltage!", AlertType.WARNING);

    @Log.NT private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    @Log.NT private final SendableChooser<DriverInput> driverChooser = new SendableChooser<>();

    @Log.NT private final SendableChooser<OperatorInput> operatorChooser = new SendableChooser<>();

    private UI() {
        if (!Robot.isReal()) new Alert("Robot is simulated!", AlertType.INFO).set(true);
        if (DriverStation.isFMSAttached()) new Alert("FMS connected!", AlertType.INFO).set(true);

        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

        if (Robot.isReal()) {
            driverChooser.setDefaultOption("Default Driver", new DefaultDriver());
            driverChooser.addOption("Sim Keyboard Driver", new SimKeyboardDriver());
        } else {
            driverChooser.setDefaultOption("Sim Keyboard Driver", new SimKeyboardDriver());
            driverChooser.addOption("Default Driver", new DefaultDriver());
        }
        operatorChooser.setDefaultOption("Default Operator", new DefaultOperator());
    }

    public void setSwerveStates(SwerveModuleState[] states) {
        moduleStates = states;
    }

    public void setPose(Pose2d pose) {
        fieldVis.setRobotPose(pose);

        rotation = pose.getRotation().getRadians(); // for the sake of swerve viz getting an angle
    }

    public void setSetpointPose(Pose2d setpointPose) {
        fieldVis.getObject("setpointPose").setPose(setpointPose);
    }

    public void setBatteryVoltage(double volts) {
        log("batteryVoltage", volts);

        voltage = volts;

        lowVoltage.set(volts < Constants.AlertConstants.LOW_VOLTAGE_THRESHOLD);

        logPowerPathResistance();
    }

    public void setTotalCurrentDraw(double amps) {
        log("totalCurrent", amps);

        currentAmps = amps;

        overcurrent.set(amps > Constants.AlertConstants.OVERCURRENT_THRESHOLD_AMPS);

        logPowerPathResistance();
    }

    public void setIntakeAngle(double radians) {
        intakeVisBody.setAngle(Rotation2d.fromRadians(radians));
    }

    private void logPowerPathResistance() {
        double resistance;

        if (currentAmps == 0) resistance = 0;
        else resistance = voltage / currentAmps;

        log("mainPowerPathResistanceOhms", resistance);

        highResistance.set(resistance > Constants.AlertConstants.HIGH_RESISTANCE_THRESHOLD_OHMS);
    }

    public void addAuto(String name, Command auto) {
        autoChooser.addOption(name, auto);
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

    public void setDriverInputChangeCallback(Consumer<DriverInput> callback) {
        driverChooser.onChange(callback);
    }

    public void setOperatorInputChangeCallback(Consumer<OperatorInput> callback) {
        operatorChooser.onChange(callback);
    }
    // TODO: push automation indicators to dashboard
}
