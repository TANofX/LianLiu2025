// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.Color;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Elevator;

/** Add your docs here. */
public class RobotMechanicalConfiguration {
    Mechanism2d m_mechanism;
    MechanismRoot2d m_root;
    MechanismLigament2d m_elevatorExtension;

    // TODO Investigate Mechanism2d
    // Does this gets drawn correctly in SmartDashboard? AdvantageScope did not seem to draw the mechanism in a manner
    // to understand the robot's configuration.
    public RobotMechanicalConfiguration() {
        m_mechanism = new Mechanism2d(Units.inchesToMeters(15.0), Units.inchesToMeters(4) + Elevator.MAX_HEIGHT_METERS);
        m_root = m_mechanism.getRoot("Elevator", Units.inchesToMeters(15.0 - 7.5), Units.inchesToMeters(4.0));
        MechanismLigament2d m_elevatorBase = new MechanismLigament2d("Elavator Extension", 0, 90.0, 30/*Units.inchesToMeters(3.0)*/, new Color8Bit(200, 200, 200));
        m_elevatorExtension = new MechanismLigament2d("Elavator Base", Units.inchesToMeters(4), 90.0, 5/*Units.inchesToMeters(5.0)*/, new Color8Bit(128, 128, 128));
        m_root.append(m_elevatorExtension);
        m_root.append(m_elevatorBase);
    }

    public void updateMechanism() {
        m_elevatorExtension.setLength(RobotContainer.elevator.getElevation());

        SmartDashboard.putData("Robot Mechanism", m_mechanism);
    }
}
