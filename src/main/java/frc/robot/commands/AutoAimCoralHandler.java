// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimCoralHandler extends Command {
  /** Creates a new AutoAimCoralHandler. */
  public AutoAimCoralHandler() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.coralHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the horizontal angle from the AutoAiming subsystem in the RobotContainer
    // and set a local variable to that value
    Rotation2d horizontalAngle = RobotContainer.autoAimer.horizontalRotationToCoral();

    Rotation2d currentAngle = Rotation2d.fromDegrees(MathUtil.clamp(horizontalAngle.getDegrees(), -80.0, 83.0));

    // Set the horizontal angle of the CoralHandler subsystem in the RobotContainer
    RobotContainer.coralHandler.setHorizontalAngleCommand(currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.coralHandler.stopHorizontalMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.coralHandler.isHorizontalAtSetpoint();
  }
}
