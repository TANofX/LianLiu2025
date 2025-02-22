// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorJoystickControl extends Command {
  private final DoubleSupplier joystick;
  private double targetHeight = 0.0;

  /** Creates a new ElevatorJoystickControl. */
  public ElevatorJoystickControl(DoubleSupplier joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick;

    addRequirements(RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetHeight = RobotContainer.elevator.getElevation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetHeight += joystick.getAsDouble() * 0.1;
    if (targetHeight < Constants.Elevator.MIN_HEIGHT_METERS) {
      targetHeight = Constants.Elevator.MIN_HEIGHT_METERS;
    } else if (targetHeight > Constants.Elevator.MAX_HEIGHT_METERS) {
      targetHeight = Constants.Elevator.MAX_HEIGHT_METERS;
    }

    RobotContainer.elevator.toHeightMeters(targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
