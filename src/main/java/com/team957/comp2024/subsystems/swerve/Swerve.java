package com.team957.comp2024.subsystems.swerve;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
                            * Math.PI
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

    public static Swerve getSwerve(boolean isReal) {
        return (isReal) ? new SwerveHW() : new SwerveSim();
    }

    public final ModuleIO frontLeft;
    public final ModuleIO frontRight;
    public final ModuleIO backRight;
    public final ModuleIO backLeft;

    protected Swerve(
            ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backRight, ModuleIO backLeft) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;

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

    public Command getModuleControllerCommand(Supplier<SwerveModuleState[]> setpoints) {
        return run(
                () -> {
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
                });
    }

    public Command getChassisRelativeControlCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return getModuleControllerCommand(
                () -> {
                    ChassisSpeeds setpoint = chassisSpeeds.get();

                    this.log("chassisSpeedsSetpoint", setpoint);

                    SwerveModuleState[] states =
                            SwerveConstants.KINEMATICS.toSwerveModuleStates(
                                    ChassisSpeeds.discretize(setpoint, Robot.kDefaultPeriod));

                    SwerveDriveKinematics.desaturateWheelSpeeds(
                            states, SwerveConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);

                    return new SwerveModuleState[] {
                        SwerveModuleState.optimize(
                                states[0], new Rotation2d(frontLeft.getSteerPositionRadians())),
                        SwerveModuleState.optimize(
                                states[1], new Rotation2d(frontRight.getSteerPositionRadians())),
                        SwerveModuleState.optimize(
                                states[2], new Rotation2d(backRight.getSteerPositionRadians())),
                        SwerveModuleState.optimize(
                                states[3], new Rotation2d(backLeft.getSteerPositionRadians()))
                    };
                });
    }

    public Command getFieldRelativeControlCommand(
            Supplier<ChassisSpeeds> fieldRelativeChassisSpeeds, Supplier<Rotation2d> robotHeading) {
        return getChassisRelativeControlCommand(
                () ->
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                fieldRelativeChassisSpeeds.get(), robotHeading.get()));
    }

    @Override
    public void periodic() {
        Robot.ui.setSwerveStates(getStates());
    }
}
