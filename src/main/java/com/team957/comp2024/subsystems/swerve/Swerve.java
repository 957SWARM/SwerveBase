package com.team957.comp2024.subsystems.swerve;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.UI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Swerve implements Subsystem, Logged {
    public abstract static class ModuleIO implements Logged {
        protected ModuleIO() {}

        // starting values of these does not effect how the module is controlled
        // just what "*IsOnboardControlLoop() returns before "set*()" is called
        protected boolean steerOnboardControl = false;
        protected boolean driveOnboardControl = false;

        /**
         * Sets the setpoint for the onboard controller. If none is present (running in simulation),
         * instantly snaps the steer to its setpoint.
         *
         * <p>If steer is currently not running off of onboard closed-loop, switches to that mode.
         *
         * @param radians Setpoint for the azimuth, in radians.
         */
        public abstract void setSteerSetpoint(double radians);

        /**
         * Sets the applied voltage to the steer motor.
         *
         * <p>If steer is currently not running off of voltage control mode, switches to that mode.
         *
         * @param radians Voltage to be applied.
         */
        public abstract void setSteerVoltage(double volts);

        /**
         * Gets the voltage applied, either directly or through a controller, to the steer motor.
         *
         * <p>Does not work with controllers in simulation.
         *
         * @return The set steer voltage.
         */
        @Log.NT
        public abstract double getSteerControlEffortVolts();

        @Log.NT
        public boolean steerIsOnboardClosedLoop() {
            return steerOnboardControl;
        }

        @Log.NT
        public abstract double getSteerPositionRadians();

        @Log.NT
        public abstract double getSteerCurrentAmps();

        public abstract void setBrakeMode(boolean active);

        @Log.NT
        public abstract boolean brakeModeIsActive();

        /**
         * Sets the setpoint for the onboard controller. If none is present (running in simulation),
         * instantly snaps the drive to its setpoint.
         *
         * <p>If drive is currently not running off of onboard closed-loop, switches to that mode.
         *
         * @param radians Setpoint for the drive, in radians per second.
         */
        public abstract void setDriveSetpoint(double radPerSecond);

        /**
         * Sets the setpoint for the onboard controller, using units of meters per second. If none
         * is present (running in simulation), instantly snaps the drive to its setpoint.
         *
         * <p>If drive is currently not running off of onboard closed-loop, switches to that mode.
         *
         * @param radians Setpoint for the drive, in meters per second.
         */
        public void setDriveSetpointMPS(double metersPerSecond) {
            setDriveSetpoint(
                    metersPerSecond
                            / (SwerveConstants.WHEEL_RADIUS_METERS)); // got rid of dividing by 2pi
            // System.out.println("MPS: " + metersPerSecond);
        }

        /**
         * Sets the applied voltage to the drive motor.
         *
         * <p>If drive is currently not running off of voltage control mode, switches to that mode.
         *
         * @param radians Voltage to be applied.
         */
        public abstract void setDriveVoltage(double volts);

        /**
         * Gets the voltage applied, either directly or through a controller, to the drive motor.
         *
         * <p>Does not work with controllers in simulation.
         *
         * @return The set drive voltage.
         */
        @Log.NT
        public abstract double getDriveControlEffortVolts();

        @Log.NT
        public boolean driveIsOnboardClosedLoop() {
            return driveOnboardControl;
        }

        @Log.NT
        public abstract double getDriveVelocityRadPerSecond();

        @Log.NT
        public double getDriveVelocityMetersPerSecond() {
            return getDriveVelocityRadPerSecond() * SwerveConstants.WHEEL_RADIUS_METERS;
        }

        @Log.NT
        public abstract double getDrivePositionRad();

        @Log.NT
        public double getDrivePositionMeters() {
            return getDrivePositionRad() * SwerveConstants.WHEEL_RADIUS_METERS;
        }

        @Log.NT
        public abstract double getDriveCurrentAmps();

        // object allocations :(
        @Log.NT
        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    getDriveVelocityMetersPerSecond(),
                    Rotation2d.fromRadians(getSteerPositionRadians()));
        }

        @Log.NT
        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    -getDrivePositionMeters(), new Rotation2d(getSteerPositionRadians()));
        }

        protected abstract void update(double dt);
    }

    public static Swerve getSwerve(boolean isReal, boolean isCompetitionRobot) {
        return (isReal) ? new SwerveHW(isCompetitionRobot) : new SwerveSim();
    }

    public final ModuleIO frontLeft;
    public final ModuleIO frontRight;
    public final ModuleIO backRight;
    public final ModuleIO backLeft;

    private final SysIdRoutine steerRoutine;

    private final SysIdRoutine driveRoutine;

    protected Swerve(
            ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backRight, ModuleIO backLeft) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;

        steerRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) -> {
                                    double asNumber = volts.magnitude();

                                    frontLeft.setSteerVoltage(asNumber);
                                    frontRight.setSteerVoltage(asNumber);
                                    backRight.setSteerVoltage(asNumber);
                                    backLeft.setSteerVoltage(asNumber);
                                },
                                null,
                                this));

        driveRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) -> {
                                    double asNumber = volts.magnitude();

                                    frontLeft.setDriveVoltage(asNumber);
                                    frontRight.setDriveVoltage(asNumber);
                                    backRight.setDriveVoltage(asNumber);
                                    backLeft.setDriveVoltage(asNumber);
                                },
                                null,
                                this));

        register();
    }

    @Log.NT
    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backRight.getState(), backLeft.getState()
        };
    }

    @Log.NT
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
        };
    }

    @Log.NT
    public ChassisSpeeds getForwardKinematicChassisSpeeds() {
        return Constants.SwerveConstants.KINEMATICS.toChassisSpeeds(getStates());
    }

    public Command getModuleControllerCommand(Supplier<SwerveModuleState[]> setpoints) {
        return run(() -> {
                    SwerveModuleState[] setpointz = setpoints.get();

                    this.log("moduleSetpoints", setpointz);

                    frontLeft.setSteerSetpoint(setpointz[0].angle.getRadians());
                    frontLeft.setDriveSetpointMPS(setpointz[0].speedMetersPerSecond);

                    frontRight.setSteerSetpoint(setpointz[1].angle.getRadians());
                    frontRight.setDriveSetpointMPS(setpointz[1].speedMetersPerSecond);

                    backRight.setSteerSetpoint(setpointz[2].angle.getRadians());
                    backRight.setDriveSetpointMPS(setpointz[2].speedMetersPerSecond);

                    backLeft.setSteerSetpoint(setpointz[3].angle.getRadians());
                    backLeft.setDriveSetpointMPS(setpointz[3].speedMetersPerSecond);
                })
                .withName("moduleControl");
    }

    public Command getChassisRelativeControlCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return getModuleControllerCommand(
                        () -> {
                            ChassisSpeeds setpoint = chassisSpeeds.get();

                            this.log("chassisSpeedsSetpoint", setpoint);

                            SwerveModuleState[] states =
                                    SwerveConstants.KINEMATICS.toSwerveModuleStates(
                                            ChassisSpeeds.discretize(
                                                    setpoint,
                                                    MiscConstants.NOMINAL_LOOP_TIME_SECONDS));

                            SwerveDriveKinematics.desaturateWheelSpeeds(
                                    states, SwerveConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);

                            return new SwerveModuleState[] {
                                SwerveModuleState.optimize(
                                        states[0],
                                        new Rotation2d(frontLeft.getSteerPositionRadians())),
                                SwerveModuleState.optimize(
                                        states[1],
                                        new Rotation2d(frontRight.getSteerPositionRadians())),
                                SwerveModuleState.optimize(
                                        states[2],
                                        new Rotation2d(backRight.getSteerPositionRadians())),
                                SwerveModuleState.optimize(
                                        states[3],
                                        new Rotation2d(backLeft.getSteerPositionRadians()))
                            };
                        })
                .withName("chassisRelativeControl");
    }

    public Command getFieldRelativeControlCommand(
            Supplier<ChassisSpeeds> fieldRelativeChassisSpeeds, Supplier<Rotation2d> robotHeading) {
        return getChassisRelativeControlCommand(
                        () ->
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        fieldRelativeChassisSpeeds.get(), robotHeading.get()))
                .withName("fieldRelativeControl");
    }

    public Command getSysIdSteerQuasistatic(boolean forward) {
        return steerRoutine
                .quasistatic(forward ? Direction.kForward : Direction.kReverse)
                .withName("steerQuasistatic" + (forward ? "Forward" : "Reverse"));
    }

    public Command getSysIdSteerDynamic(boolean forward) {
        return steerRoutine
                .dynamic(forward ? Direction.kForward : Direction.kReverse)
                .withName("steerDynamic" + (forward ? "Forward" : "Reverse"));
    }

    public Command getSysIdDriveQuasistatic(boolean forward) {
        return driveRoutine
                .quasistatic(forward ? Direction.kForward : Direction.kReverse)
                .withName("driveQuasistatic" + (forward ? "Forward" : "Reverse"));
    }

    public Command getSysIdDriveDynamic(boolean forward) {
        return driveRoutine
                .dynamic(forward ? Direction.kForward : Direction.kReverse)
                .withName("driveDynamic" + (forward ? "Forward" : "Reverse"));
    }

    public Command lockDrivetrain() {
        return run(() -> {
                    this.frontLeft.setBrakeMode(true);
                    this.frontRight.setBrakeMode(true);
                    this.backRight.setBrakeMode(true);
                    this.backLeft.setBrakeMode(true);
                })
                .andThen(
                        getModuleControllerCommand(
                                () ->
                                        new SwerveModuleState[] {
                                            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                                            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                                            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                                            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4))
                                        }))
                .withName("lockDrivetrain");
    }

    @Override
    public void periodic() {
        UI.instance.setSwerveStates(getStates());

        Command activeCommand = getCurrentCommand();

        if (activeCommand != null) log("activeCommand", activeCommand.getName());
    }
}
