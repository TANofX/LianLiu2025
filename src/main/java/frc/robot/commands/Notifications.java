// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
 import frc.robot.subsystems.LEDs.AnimationTypes;
 

public class Notifications extends Command {
  private enum LED_State {
    CORAL_HOLD,
    ALGAE_HOLD,
    CLIMB,
    DEFAULT
  }

  private LED_State setState;
  private LED_State currentState;

  /** Creates a new Notifications. */
  public Notifications() {
    // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(RobotContainer.LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = null;
    setState = LED_State.DEFAULT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (currentState != setState) {
    switch (setState) {
      case ALGAE_HOLD -> RobotContainer.LEDs.changeAnimation(AnimationTypes.OrangeSolid);
      case CLIMB -> RobotContainer.LEDs.changeAnimation(AnimationTypes.BlueTilt);
      case CORAL_HOLD -> RobotContainer.LEDs.changeAnimation(AnimationTypes.GreenBreeze);
      default -> RobotContainer.LEDs.changeAnimation(AnimationTypes.Rainbow);
    }
   }

   currentState = setState;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     RobotContainer.LEDs.changeAnimation(AnimationTypes.Rainbow);
  }

  // Returns true when the command should end.
  // The command should never end because it's a default command for LEDs subsystem
  @Override
  public boolean isFinished() {
    return false;
  }
}
