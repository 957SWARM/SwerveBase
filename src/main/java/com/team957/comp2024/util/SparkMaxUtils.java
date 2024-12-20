package com.team957.comp2024.util;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import java.util.ArrayList;

public class SparkMaxUtils {
    // object allocation bad!
    private static final ArrayList<String> faultsBuffer = new ArrayList<>();

    public static String[] getFaultsAsStringArray(CANSparkMax sMax, boolean sticky) {
        faultsBuffer.clear();

        for (FaultID fault : FaultID.values()) {
            if ((sticky) ? sMax.getStickyFault(fault) : sMax.getFault(fault))
                faultsBuffer.add(fault.toString());
        }

        return faultsBuffer.toArray(new String[] {});
    }

    /**
     * Can't Spark Max, if you will
     *
     * <p>Mutates & returns the given SparkMax with all selected periodic frames (indicated with
     * `true`) slowed down
     */
    public static CANSparkMax slowUnusedPeriodics(
            CANSparkMax sparkMax,
            boolean rotorPosition,
            boolean analogEncoder,
            boolean alternateEncoder,
            boolean dutyCyclePosition,
            boolean dutyCycleVelocity) {
        int normal = 20; // milliseconds
        int slow = 1000; // milliseconds

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, rotorPosition ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, analogEncoder ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, alternateEncoder ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, dutyCyclePosition ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, dutyCycleVelocity ? slow : normal);

        return sparkMax;
    }
}
