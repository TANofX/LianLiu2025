// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class TuneVelocitySparkPIDController extends TuneSparkPIDController {

    private double targetVelocity;

    public TuneVelocitySparkPIDController(String motorName, SparkFlex sparkMotor, Subsystem motorOwner) {
        super(motorName, sparkMotor, motorOwner);
    }

    @Override
    public void initialize() {
        super.initialize();
        targetVelocity = 0;

        SmartDashboard.putNumber(name + " Target Velocity", targetVelocity);
    }

    @Override
    public void execute() {
        super.execute();

        double target = SmartDashboard.getNumber(name + " Target Velocity", 0.0);
        if (target != targetVelocity) { tuningController.getClosedLoopController().setReference(target, ControlType.kVelocity); targetVelocity = target; }
        double error = (target-encoder.getVelocity())/target;
        
        SmartDashboard.putNumber(name + " Current Velocity", encoder.getVelocity());
        SmartDashboard.putNumber(name + " Percent Error", error);
        
        SmartDashboard.putNumber(name + " Output Voltage", tuningController.getAppliedOutput());
    }
}
