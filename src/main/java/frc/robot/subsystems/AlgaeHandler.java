// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//All my imports ;D
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class AlgaeHandler extends AdvancedSubsystem {
  private final SparkMax algaeMotor;
  private final Solenoid algaePiston;
  private final DigitalInput algaeLimitSwitch;

  private final SparkMaxSim algaeHandlerMotorSim;

  private final RelativeEncoder algaeEncoder;

  // Creating simulation for algae handler ??Need to add constants??
  private final FlywheelSim m_algaeHandlerSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1),
          Constants.AlgaeHandler.MOMENT_OF_INERTIA_OF_THE_BOTTOM_INTAKE_WHEEL,
          Constants.AlgaeHandler.ALGAE_GEAR_RATIO),
      DCMotor.getNeoVortex(1));

  /** Creates a new AlgaeHandler. */
  public AlgaeHandler(int algaeMotorCANID, int algaeSolenoidID, int algaeLimitID) {
    // creating motor/solenoid/switches/controllers
    algaeMotor = new SparkMax(algaeMotorCANID, MotorType.kBrushless);
    algaePiston = new Solenoid(PneumaticsModuleType.REVPH, algaeSolenoidID);
    algaeLimitSwitch = new DigitalInput(algaeLimitID);
    algaeEncoder = algaeMotor.getEncoder();

    SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();
    ClosedLoopConfig algaeMotorPIDConfig = algaeMotorConfig.closedLoop;
    algaeMotorConfig.idleMode(IdleMode.kBrake);
    algaeMotorConfig.smartCurrentLimit(80);
    algaeMotorPIDConfig.pidf(Constants.AlgaeHandler.ALGAE_MOTOR_P, Constants.AlgaeHandler.ALGAE_MOTOR_I,
        Constants.AlgaeHandler.ALGAE_MOTOR_D, Constants.AlgaeHandler.ALGAE_MOTOR_FF);

    algaeMotorPIDConfig.iZone(Constants.AlgaeHandler.ALGAE_MOTOR_I_ZONE);
    algaeMotorPIDConfig.maxMotion.maxVelocity(Constants.AlgaeHandler.ALGAE_MOTOR_MAX_VELOCITY);
    algaeMotorPIDConfig.maxMotion.maxAcceleration(Constants.AlgaeHandler.ALGAE_MOTOR_MAX_ACCELERATION);
    algaeMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.AlgaeHandler.ALGAE_MOTOR_ALLOWED_CLOSED_LOOP_ERROR);
    algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    registerHardware("Algae Motor", algaeMotor);

    // Configure the motor simulation
    algaeHandlerMotorSim = new SparkMaxSim(algaeMotor, DCMotor.getNeoVortex(1));
  }

  @Override
  public void simulationPeriodic() {
    // puts stuff on smart dashboard, cool stuff
    double inputVoltage = algaeHandlerMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    // SmartDashboard.putNumber("Algae Handler Simulated Voltage", inputVoltage);
    // SmartDashboard.putNumber("Algae Handler Simulated Motor Position", algaeHandlerMotorSim.getPosition());
    // SmartDashboard.putNumber("Algae Handler Simulated Motor Velocity", algaeHandlerMotorSim.getVelocity());

    // updates simulated voltage
    m_algaeHandlerSim.setInputVoltage(inputVoltage);
    m_algaeHandlerSim.update(0.020);

    // Iterate the motor simulation9
    algaeHandlerMotorSim.iterate(m_algaeHandlerSim.getAngularVelocityRPM() * Constants.AlgaeHandler.ALGAE_GEAR_RATIO,
        RobotController.getBatteryVoltage(), 0.020);

    // Update the RoboRioSim voltage to the default battery voltage based on the
    // elevator motor current.
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(algaeHandlerMotorSim.getMotorCurrent()));
  }

  // Methods

  public void lowerAlgaeIntake() {
    // solenoid will lower the intake
    algaePiston.set(true);
  }

  public void raiseAlgaeIntake() {
    // solenoid will raise the intake
    algaePiston.set(false);
  }

  public void stopAlgaeMotor() {
    // Algae motor is stopped
    algaeMotor.stopMotor();
  }

  public void runAlgaeMotor() {
    // velocity value is a place holder because I didn't know what I was doing :D do
    // we need math for spins per motor revolution??
    // algaeMotorController.setReference(577.26, ControlType.kVelocity);
    algaeMotor.set(-.5);
  }

  public void reverseAlgaeMotor() {
    // velocity value is a place holder :D
    // algaeMotorController.setReference(-577.26, ControlType.kVelocity);
    algaeMotor.set(1);
  }

  public boolean hasAlgae() {
    // Will be true when algae handler has algae
    return !algaeLimitSwitch.get();
  }

  public boolean isAlgaeIntakeUp() {
    // returns a boolean to tell the robot whether or not algae intake is up
    return !algaePiston.get();
  }

  public Command AlgaeRunMotor() {
    return Commands.sequence(
        Commands.runOnce(() -> runAlgaeMotor(), this));

  }

  // This command intakes an algae
  public Command getAlgaeIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          runAlgaeMotor();
          lowerAlgaeIntake();
        }, this),
        Commands.waitUntil(() -> {
          return hasAlgae();
        }),
        Commands.runOnce(() -> {
          stopAlgaeMotor();
          raiseAlgaeIntake();
        }, this),
        Commands.waitUntil(() -> {
          return isAlgaeIntakeUp();
        }),
        Commands.run(() -> {
          if ((!hasAlgae()))
            runAlgaeMotor();
          else
            stopAlgaeMotor();
        }, this)).finallyDo(() -> {
          stopAlgaeMotor();
          raiseAlgaeIntake();
        });

    // hello traveler, viewing my code I see
  }

  public Command shootAlgaeCommand() {
    // This command makes sure intake is up, and reverses algae motors until limit
    // switch isn't triggered
    return Commands.sequence(
        Commands.waitUntil(() -> {
          return isAlgaeIntakeUp();
        }),
        Commands.runOnce(() -> {
          reverseAlgaeMotor();
        }, this),
        Commands.waitUntil(() -> {
          return !hasAlgae();
        }),
        Commands.runOnce(() -> {
          stopAlgaeMotor();
        }).finallyDo(() -> stopAlgaeMotor()));
  }

  public Command runAlgaeIntakeManuallyCommand() {
    // this command will allow the drivers to manually run the algae intake
    return Commands.sequence(
        Commands.runOnce(() -> {
          runAlgaeMotor();
        }));
  }

  public Command reverseAlgaeIntakeManuallyCommand() {
    // this command will allow the drivers to manually reverse the algae intake
    return Commands.sequence(
        Commands.runOnce(() -> {
          reverseAlgaeMotor();
        }));
  }

  public Command raiseAlgaeIntakeManually() {
    // this command will allow the drivers to manually raise the algae intake
    return Commands.sequence(
        Commands.runOnce(() -> {
          raiseAlgaeIntake();
        }));
  }

  public Command lowerAlgaeIntakeManually() {
    // this command will allow the drivers to manually lower the algae intake
    return Commands.sequence(
        Commands.runOnce(() -> {
          lowerAlgaeIntake();
        }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Handler/Motor Velocity", algaeEncoder.getVelocity());
    SmartDashboard.putBoolean("Has Algae", hasAlgae());
    SmartDashboard.putBoolean("Algae Intake is Raised", isAlgaeIntakeUp());
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              runAlgaeMotor();
            }, this),
        Commands.waitSeconds(5),
        Commands.runOnce(
            () -> {
              // Checks to make sure that motor is going at minimum speed needed to intake
              // algae (way slower than our maximum potential ;)
              if ((algaeEncoder.getVelocity()) < 500) {
                addFault("[System Check] Algae Handler velocity too slow", false, true);
              }
              stopAlgaeMotor();
            }, this),
        Commands.runOnce(
            () -> {
              lowerAlgaeIntake();
            }, this),
        Commands.runOnce(
            () -> {
              raiseAlgaeIntake();
              if (!isAlgaeIntakeUp()) {
                addFault("Hall Effect Sensor does not know algae intake is raised", false, true);
              }
            }, this),
        Commands.runOnce(
            () -> {
              if (!hasAlgae()) {
                addFault("Limit Switch is not triggered", false, true);
              }
            })
    );
  }
}
