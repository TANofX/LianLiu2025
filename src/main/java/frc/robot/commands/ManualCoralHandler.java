// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralHandler extends Command {
  private DoubleSupplier vertController;
  private DoubleSupplier hortController;
  /** Creates a new ManualCoralHandlerHorizontal. */
  public ManualCoralHandler(DoubleSupplier vertJoystickSupplier, DoubleSupplier hortJoystickSupplier) {
    vertController = vertJoystickSupplier;
    hortController = hortJoystickSupplier;

    addRequirements(RobotContainer.coralHandler);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double verticalTargetAngleInDegrees = Constants.CoralHandler.horizontalAngleChangeDegreesPerSecond * 1;
    double horizontalTargetAngleInDegrees = Constants.CoralHandler.verticalAngleChangeDegreesPerSecond * 1;

    RobotContainer.coralHandler.setVerticalAngle(Rotation2d.fromDegrees(RobotContainer.coralHandler.getVerticalAngle().getDegrees() + (verticalTargetAngleInDegrees * vertController.getAsDouble())));
    RobotContainer.coralHandler.setHorizontalAngle(Rotation2d.fromDegrees(RobotContainer.coralHandler.getHorizontalAngle().getDegrees() + (horizontalTargetAngleInDegrees * hortController.getAsDouble())));
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
