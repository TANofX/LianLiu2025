// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import java.util.ArrayList;

/** Add your docs here. */
public class TunablePIDSet implements TunablePID {
    private final ArrayList<TunablePID> pidList = new ArrayList<>();

    private double kP;
    private double kI;
    private double kD;
    private double kFF;
    private double kIZone;
    private double minValue;
    private double maxValue;

    public void add(TunablePID pid) {
        pidList.add(pid);

        kP = pid.getP();
        kI = pid.getI();
        kD = pid.getD();
        kFF = pid.getFF();
        kIZone = pid.getIZone();
        minValue = pid.getOutputMin();
        maxValue = pid.getOutputMax();
    }

    @Override
    public double getD() {
        return kD;
    }

    @Override
    public double getFF() {
        return kFF;
    }

    @Override
    public double getI() {
        return kI;
    }

    @Override
    public double getIZone() {
        return kIZone;
    }

    @Override
    public double getOutputMax() {
        return maxValue;
    }

    @Override
    public double getOutputMin() {
        return minValue;
    }

    @Override
    public double getP() {
        return kP;
    }

    @Override
    public void setD(double kD) {
        this.kD = kD;

        for (TunablePID pid: pidList) {
            pid.setD(kD);
        }
    }

    @Override
    public void setFF(double kFF) {
        this.kFF = kFF;

        for (TunablePID pid: pidList) {
            pid.setFF(kFF);
        }
    }

    @Override
    public void setI(double kI) {
        this.kI = kI;

        for (TunablePID pid: pidList) {
            pid.setI(kI);
        }
    }

    @Override
    public void setIZone(double Izone) {
         this.kIZone = Izone;

        for (TunablePID pid: pidList) {
            pid.setIZone(Izone);
        }
   }

    @Override
    public void setOutputRange(double minOutput, double maxOutput) {
        this.minValue = minOutput;
        this.maxValue = maxOutput;

        for (TunablePID pid: pidList) {
            pid.setOutputRange(minOutput, maxOutput);
        }
    }

    @Override
    public void setP(double kP) {
        this.kP = kP;

        for (TunablePID pid: pidList) {
            pid.setP(kP);
        }
    }

}
