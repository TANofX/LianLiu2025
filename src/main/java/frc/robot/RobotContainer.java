// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;import frc.robot.commands.Notifications;
import frc.robot.subsystems.*;




public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  

  // Subsystems
  public static final Vision vision = new Vision();
  public static final Swerve swerve = new Swerve();// new Swerve();
   public static final LEDs LEDs = new LEDs();
  // public static final Elevator elevator = new Elevator(Constants.Elevator.motorCanID);
 // public static final RobotMechanism robotMechanism = new RobotMechanism();
  // public static final CoralHandler coralHandler = new CoralHandler(Constants.CoralHandler.outtakeMotorID, Constants.CoralHandler.horizontalMotorID, Constants.CoralHandler.verticalMotorID, Constants.CoralHandler.horizontalEncoderID, Constants.CoralHandler.verticalEncoderID);
  public static final AlgaeHandler leftAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.leftAlgaeMotorCANID, Constants.AlgaeHandler.leftAlgaeSolenoidID, Constants.AlgaeHandler.leftAlgaeLimitID);
  public static final AlgaeHandler rightAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.rightAlgaeMotorCANID, Constants.AlgaeHandler.rightAlgaeSolenoidID, Constants.AlgaeHandler.rightAlgaeLimitID);
 
  public static final Climber climber = new Climber(Constants.Climber.MOTOR_CANID, Constants.Climber.PCMID, Constants.Climber.FORWARDSOLENOID, Constants.Climber.REVERSESOLENOID, Constants.Climber.ENCODERID);

  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {
    // coralHandler.registerSystemCheckWithSmartDashboard();
    // SmartDashboard.putData("Elevator Test", elevator.getSystemCheckCommand());

    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
     LEDs.setDefaultCommand(new Notifications());
    //elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));
    // SmartDashboard.putData("Left Algae Handler Test", leftAlgaeHandler.getSystemCheckCommand());
    // SmartDashboard.putData("Right Algae Handler Test", rightAlgaeHandler.getSystemCheckCommand());
   
   
    SmartDashboard.putData("Prepare Climber", climber.getPrepareCommand());
    SmartDashboard.putData("Climb Climber", climber.climbCommand());
    SmartDashboard.putData("Run Claw Motor", climber.runClimberMotorCommand());
    SmartDashboard.putData("Reverse Claw Motor", climber.runClimberMotorCommandOpposite());
    SmartDashboard.putData("Climber System Check", climber.getSystemCheckCommand());
    SmartDashboard.putData("Calibrate Climber", climber.getCalibrateCommand(false));
    SmartDashboard.putData("Reverse Calibrate Command", climber.getCalibrateCommand(true));
    SmartDashboard.putData("Climber/Set-90", climber.setClimberNeg90(Rotation2d.fromDegrees(-90)));
  }
  

  private void configureButtonBindings() {
   
    driver.LT().whileTrue(leftAlgaeHandler.getAlgaeIntakeCommand());
    driver.LB().onTrue(leftAlgaeHandler.shootAlgaeCommand());
    driver.RT().whileTrue(rightAlgaeHandler.getAlgaeIntakeCommand());
    driver.RB().onTrue(rightAlgaeHandler.shootAlgaeCommand());
    // coDriver.A().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    // coDriver.B().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    // coDriver.Y().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(40.0)));
    // coDriver.X().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MAX_HEIGHT_METERS));
    coDriver.A().whileTrue(climber.runClimberMotorCommand());
    coDriver.B().onTrue(climber.getOpenCommand());
    coDriver.X().onTrue(climber.getStowCommand());
    coDriver.START();
    // coDriver.RT().onTrue(new CoralHandlerAngleEstimator());

   //ONCE WE ADD ALGAE TO MAIN THESE COMMANDS SHOULD WORK:
   //driver.LT().onTrue(new getAlgaeIntakeCommand());
   //driver.RT().onTrue(new shootAlgaeCommand());
   //driver.START().onTrue(new ); //calibrate elevator

    coDriver.START();
  }


  public static void periodic() { }
}