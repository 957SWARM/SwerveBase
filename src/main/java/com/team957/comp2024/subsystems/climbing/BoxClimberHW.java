package com.team957.comp2024.subsystems.climbing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team957.comp2024.Constants.BoxClimberConstants;

public class BoxClimberHW extends BoxClimber {

    private final TalonSRX climbMotor = new TalonSRX(BoxClimberConstants.MOTOR_CANID);

    public BoxClimberHW() {
        // not sure if there is an equivalent of restore factory defaults that needs to be run for
        // talons
        climbMotor.clearStickyFaults();

        climbMotor.configPeakCurrentLimit(BoxClimberConstants.CURRENT_LIMIT);

        climbMotor.setInverted(BoxClimberConstants.MOTOR_INVERTED);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        climbMotor.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    @Override
    public double getMotorVoltage() {
        return climbMotor.getBusVoltage();
    }

    @Override
    public double getMotorAmps() {
        return climbMotor.getSupplyCurrent();
    }
}
