// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralHandlerAngleEstimator extends Command {
  /** Creates a new CoralHandlerEstimator. */
  public CoralHandlerAngleEstimator() {
    addRequirements(RobotContainer.coralHandler, RobotContainer.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.elevator.getElevation() - Constants.Elevator.level1Height < 0.5) & RobotContainer.coralHandler.hasCoral() == false) {
      RobotContainer.coralHandler.setIntakeAngle();
    }
    else if ((RobotContainer.elevator.getElevation() - Constants.Elevator.level1Height < 0.5) & RobotContainer.coralHandler.hasCoral() == true) {
      // RobotContainer.coralHandler.setLevelOneAngle(); //TODO add setLevelOneAngle back in
    }
    else if ((RobotContainer.elevator.getElevation() - Constants.Elevator.level2Height < 0.5) & RobotContainer.coralHandler.hasCoral() == true) {
      RobotContainer.coralHandler.setLevelTwoAngle();
    }
    else if ((RobotContainer.elevator.getElevation() - Constants.Elevator.level3Height < 0.5) & RobotContainer.coralHandler.hasCoral() == true) {
      RobotContainer.coralHandler.setLevelThreeAngle();
    }
    else if ((RobotContainer.elevator.getElevation() - Constants.Elevator.level4Height < 0.5) & RobotContainer.coralHandler.hasCoral() == true) {
      RobotContainer.coralHandler.setLevelFourAngle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
