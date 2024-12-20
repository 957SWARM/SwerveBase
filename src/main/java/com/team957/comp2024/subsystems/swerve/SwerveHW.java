package com.team957.comp2024.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.util.SparkMaxUtils;
import com.team957.lib.math.UtilityMath;
import edu.wpi.first.math.util.Units;
import monologue.Annotations.Log;

public class SwerveHW extends Swerve {

    private static class HWModuleIO extends ModuleIO {
        private final CANSparkMax steer;
        private final CANSparkMax drive;

        private final SparkPIDController steerController;
        private final SparkPIDController driveController;

        private final AbsoluteEncoder steerEncoder;
        private final RelativeEncoder driveEncoder;

        private final double steerOffsetRadians;
        private final boolean invertDrive;

        private boolean brakeModeIsActive = false;

        HWModuleIO(int steerCANID, int driveCANID, double steerOffsetRadians, boolean invertDrive) {
            steer =
                    SparkMaxUtils.slowUnusedPeriodics(
                            new CANSparkMax(steerCANID, MotorType.kBrushless),
                            true, // needs to change to false if we start seeding
                            true,
                            true,
                            false,
                            false);

            drive =
                    SparkMaxUtils.slowUnusedPeriodics(
                            new CANSparkMax(driveCANID, MotorType.kBrushless),
                            false,
                            true,
                            true,
                            true,
                            true);

            steer.restoreFactoryDefaults();
            drive.restoreFactoryDefaults();

            // steer.setInverted(true);

            steer.setSmartCurrentLimit(
                    (int) Constants.SwerveConstants.STEER_CURRENT_LIMIT); // WHY IS IT AN INT, REV?
            drive.setSmartCurrentLimit((int) Constants.SwerveConstants.DRIVE_CURRENT_LIMIT); // WHY

            steerController = steer.getPIDController();
            driveController = drive.getPIDController();

            steerController.setP(Constants.SwerveConstants.ONBOARD_STEER_CONTROLLER_KP);
            steerController.setI(Constants.SwerveConstants.ONBOARD_STEER_CONTROLLER_KI);
            steerController.setD(Constants.SwerveConstants.ONBOARD_STEER_CONTROLLER_KD);

            driveController.setP(Constants.SwerveConstants.ONBOARD_DRIVE_CONTROLLER_KP);
            driveController.setI(Constants.SwerveConstants.ONBOARD_DRIVE_CONTROLLER_KI);
            driveController.setD(Constants.SwerveConstants.ONBOARD_DRIVE_CONTROLLER_KD);
            driveController.setFF(Constants.SwerveConstants.ONBOARD_DRIVE_CONTROLLER_KFF);

            steerEncoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
            steerEncoder.setInverted(true);
            driveEncoder = drive.getEncoder();

            steerController.setFeedbackDevice(steerEncoder);

            // normalizeangleradians gives 0 ~ 2pi, converted to rots that's 0 ~ 1
            steerController.setPositionPIDWrappingMaxInput(1);
            steerController.setPositionPIDWrappingMinInput(0);
            steerController.setPositionPIDWrappingEnabled(true);

            this.steerOffsetRadians = steerOffsetRadians;
            this.invertDrive = invertDrive;

            setBrakeMode(Constants.SwerveConstants.INITIAL_BRAKE_MODE_ACTIVE);
        }

        @Override
        public void setSteerSetpoint(double radians) {
            steerOnboardControl = true;

            steerController.setReference(
                    Units.radiansToRotations(
                            UtilityMath.normalizeAngleRadians((radians + steerOffsetRadians))),
                    ControlType.kPosition);
        }

        @Override
        public void setSteerVoltage(double volts) {
            steerOnboardControl = false;

            steer.setVoltage(-UtilityMath.clamp(Constants.MiscConstants.SATURATION_VOLTAGE, volts));
            // I honestly have no idea what this code does but it was in the bunnybots
            // swerve and
            // that worked fine???
        }

        @Override
        public double getSteerControlEffortVolts() {
            return steer.getAppliedOutput() * getSteerBusVoltage();
        }

        @Override
        public boolean steerIsOnboardClosedLoop() {
            return steerOnboardControl;
        }

        @Override
        public double getSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(
                    getUnoffsetSteerPositionRadians() - steerOffsetRadians);
        }

        @Override
        public double getSteerCurrentAmps() {
            return steer.getOutputCurrent();
        }

        @Override
        public void setBrakeMode(boolean active) {
            drive.setIdleMode(active ? IdleMode.kBrake : IdleMode.kCoast);
            steer.setIdleMode(active ? IdleMode.kBrake : IdleMode.kCoast);

            brakeModeIsActive = active;
        }

        @Override
        public boolean brakeModeIsActive() {
            return brakeModeIsActive;
        }

        private double driveWheelRadPerSecondToMotorRPM(double radPerSecond) {
            return Constants.SwerveConstants.DRIVE_GEARING_HELPER.inputFromOutput(
                    Units.radiansPerSecondToRotationsPerMinute(radPerSecond)
                            * (invertDrive ? -1 : 1));
        }

        private double driveMotorRotationsToWheelRad(double rot) {
            return Constants.SwerveConstants.DRIVE_GEARING_HELPER.outputFromInput(
                    Units.rotationsToRadians(rot) * (invertDrive ? -1 : 1));
        }

        @Override
        public void setDriveSetpoint(double radPerSecond) {
            driveOnboardControl = true;

            driveController.setReference(
                    driveWheelRadPerSecondToMotorRPM(radPerSecond), ControlType.kVelocity);
        }

        @Override
        public void setDriveVoltage(double volts) {
            driveOnboardControl = false;

            drive.setVoltage(
                    UtilityMath.clamp(Constants.MiscConstants.SATURATION_VOLTAGE, volts)
                            * (invertDrive ? -1 : 1));
        }

        @Override
        public double getDriveControlEffortVolts() {
            return drive.getAppliedOutput() * getDriveBusVoltage();
        }

        @Override
        @Log
        public boolean driveIsOnboardClosedLoop() {
            return driveOnboardControl;
        }

        @Override
        @Log
        public double getDriveVelocityRadPerSecond() {
            return driveMotorRotationsToWheelRad(
                    driveEncoder.getVelocity() / 60); // velocity in RPM units
        }

        public double getDrivePositionRad() {
            return driveMotorRotationsToWheelRad(driveEncoder.getPosition());
        }

        @Override
        public double getDriveCurrentAmps() {
            return drive.getOutputCurrent();
        }

        @Log.NT
        public double getSteerTemperatureC() {
            return steer.getMotorTemperature();
        }

        @Log.NT
        public double getDriveTemperatureC() {
            return drive.getMotorTemperature();
        }

        @Log.NT
        public double getSteerBusVoltage() {
            return steer.getBusVoltage();
        }

        @Log.NT
        public double getDriveBusVoltage() {
            return drive.getBusVoltage();
        }

        @Log.NT
        public double getUnoffsetSteerPositionRadians() {
            return steerEncoder.getPosition() * 2 * Math.PI;
        }

        @Log.NT
        public String[] getSteerStickyFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(steer, true);
        }

        @Log.NT
        public String[] getSteerFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(steer, false);
        }

        @Log.NT
        public String[] getDriveStickyFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(drive, true);
        }

        @Log.NT
        public String[] getDriveFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(drive, false);
        }

        @Override
        protected void update(double dt) {}
    }

    public SwerveHW(boolean isCompetitionRobot) {
        super(
                new HWModuleIO(
                        SwerveConstants.FRONT_LEFT_STEER_CANID,
                        SwerveConstants.FRONT_LEFT_DRIVE_CANID,
                        isCompetitionRobot
                                ? Constants.SwerveConstants.FRONT_LEFT_STEER_OFFSET_RADIANS
                                : SwerveConstants.PRACTICE_FRONT_LEFT_STEER_OFFSET_RADIANS,
                        SwerveConstants.FRONT_LEFT_DRIVE_INVERTED),
                new HWModuleIO(
                        SwerveConstants.FRONT_RIGHT_STEER_CANID,
                        SwerveConstants.FRONT_RIGHT_DRIVE_CANID,
                        isCompetitionRobot
                                ? SwerveConstants.FRONT_RIGHT_STEER_OFFSET_RADIANS
                                : SwerveConstants.PRACTICE_FRONT_RIGHT_STEER_OFFSET_RADIANS,
                        SwerveConstants.FRONT_RIGHT_DRIVE_INVERTED),
                new HWModuleIO(
                        SwerveConstants.BACK_RIGHT_STEER_CANID,
                        SwerveConstants.BACK_RIGHT_DRIVE_CANID,
                        isCompetitionRobot
                                ? SwerveConstants.BACK_RIGHT_STEER_OFFSET_RADIANS
                                : SwerveConstants.PRACTICE_BACK_RIGHT_STEER_OFFSET_RADIANS,
                        SwerveConstants.BACK_RIGHT_DRIVE_INVERTED),
                new HWModuleIO(
                        SwerveConstants.BACK_LEFT_STEER_CANID,
                        SwerveConstants.BACK_LEFT_DRIVE_CANID,
                        isCompetitionRobot
                                ? SwerveConstants.BACK_LEFT_STEER_OFFSET_RADIANS
                                : SwerveConstants.PRACTICE_BACK_LEFT_STEER_OFFSET_RADIANS,
                        SwerveConstants.BACK_LEFT_DRIVE_INVERTED));
    }

    @Override
    public void periodic() {
        super.periodic();

        // not required to override this, but want to prevent accidentially overriding the code in
        // superclass
    }
}
