// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//All my imports ;D
package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;



public class AlgaeHandler extends AdvancedSubsystem {
private SparkFlex algaeMotor;
private Solenoid algaePiston;
private DigitalInput algaeLimitSwitch;
private  SparkClosedLoopController algaeMotorController;

private SparkFlexSim algaeHandlerMotorSim;

private RelativeEncoder algaeEncoder;

//Creating simulation for algae handler ??Need to add constants??
private final FlywheelSim m_algaeHandlerSim = 
  new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 
                  Constants.AlgaeHandler.momentOfInertiaOfTheBottomIntakeWheel, 
                  Constants.AlgaeHandler.algaeGearRatio),
                  DCMotor.getNeoVortex(1));


  
  /** Creates a new AlgaeHandler. */
  public AlgaeHandler(int algaeMotorCANID, int algaeSolenoidID, int algaeLimitID) {
    //creating motor/solenoid/switches/controllers
    algaeMotor = new SparkFlex(algaeMotorCANID, MotorType.kBrushed);
    algaePiston = new Solenoid(PneumaticsModuleType.REVPH, algaeSolenoidID);
    algaeLimitSwitch = new DigitalInput(algaeLimitID);
    algaeMotorController = algaeMotor.getClosedLoopController();
    algaeEncoder = algaeMotor.getEncoder(); 
   


    SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();
    ClosedLoopConfig algaeMotorPIDConfig = algaeMotorConfig.closedLoop;
    algaeMotorConfig.idleMode(IdleMode.kBrake);
    algaeMotorConfig.smartCurrentLimit(80);
    algaeMotorPIDConfig.pidf(Constants.AlgaeHandler.algaeMotorP, Constants.AlgaeHandler.algaeMotorI, Constants.AlgaeHandler.algaeMotorD, Constants.AlgaeHandler.algaeFF);
    
    algaeMotorPIDConfig.iZone(Constants.AlgaeHandler.algaeIZone);
    algaeMotorPIDConfig.maxMotion.maxVelocity(Constants.AlgaeHandler.algaeMotorMaxVelocity);
    algaeMotorPIDConfig.maxMotion.maxAcceleration(Constants.AlgaeHandler.algaeMotorMaxAcceleration);
    algaeMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.AlgaeHandler.algaeMotorAllowedError);
    algaeMotor.configure(algaeMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
    registerHardware("Algae Motor", algaeMotor);
  
   //Configure the motor simulation
   algaeHandlerMotorSim = new SparkFlexSim(algaeMotor, DCMotor.getNeoVortex(1));


   
  }

@Override
public void simulationPeriodic() {

  //puts stuff on smart dashboard, cool stuff
double inputVoltage = algaeHandlerMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
SmartDashboard.putNumber("Algae Handler Simulated Voltage", inputVoltage);
SmartDashboard.putNumber("Algae Handler Simulated Motor Position", algaeHandlerMotorSim.getPosition());
SmartDashboard.putNumber("Algae Handler Simulated Motor Velocity", algaeHandlerMotorSim.getVelocity());


//updates simulated voltage
m_algaeHandlerSim.setInputVoltage(inputVoltage);
m_algaeHandlerSim.update(0.020);

 // Iterate the motor simulation9
algaeHandlerMotorSim.iterate(m_algaeHandlerSim.getAngularVelocityRPM() * Constants.AlgaeHandler.algaeGearRatio, RobotController.getBatteryVoltage(), 0.020);


  // Update the RoboRioSim voltage to the default battery voltage based on the elevator motor current.
  RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(algaeHandlerMotorSim.getMotorCurrent()));
}

//Methods 

public void lowerAlgaeIntake() {
  //solenoid will lower the intake
  algaePiston.set(true);

}

public void raiseAlgaeIntake() {
  //solenoid will raise the intake
  algaePiston.set(false);
  
}

public void stopAlgaeMotor() {
  //Algae motor is stopped
  algaeMotor.stopMotor();
}

public void runAlgaeMotor() {
  //velocity value is a place holder because I didnt know what I was doing :D do we need math for spins per motor revolution??
  algaeMotorController.setReference(577.26, ControlType.kVelocity);
  //algaeMotor.set(.5);
}

public void reverseAlgaeMotor() {
  //velocity value is a place holder :D
  algaeMotorController.setReference(-577.26, ControlType.kVelocity);
  //algaeMotor.set(-.5);
}


public boolean hasAlgae() {
  //Will be true when algae handler has algae
  return algaeLimitSwitch.get();
}

public boolean isAlgaeIntakeUp() {
  //returns a boolean to tell the robot whether or not algae intake is up
  return !algaePiston.get();
  }



//This command intakes an algae
public Command getAlgaeIntakeCommand() {
  return Commands.sequence (
    Commands.runOnce(() -> {
      runAlgaeMotor();
      lowerAlgaeIntake();
    }, this),
    Commands.waitUntil(() -> {
    return hasAlgae();
    }),

    Commands.runOnce(() -> {
      raiseAlgaeIntake();
    }, this),
    Commands.waitUntil(()-> {
      return isAlgaeIntakeUp();
    }),
    Commands.run(()-> {
      if((hasAlgae() ) || (!isAlgaeIntakeUp())) 
      runAlgaeMotor();
      else
      stopAlgaeMotor();    
    }, this)
    ).finallyDo(() -> {
      stopAlgaeMotor();
      raiseAlgaeIntake();
    });


    //hello traveller, viewing my code I see 
    
}

public Command shootAlgaeCommand() {
  //This command makes sure intake is up, and reverses algae motors until limit switch isnt triggered
   return Commands.sequence (
    Commands.waitUntil (() -> {
      return isAlgaeIntakeUp();
    }),

    Commands.runOnce(() -> {
      reverseAlgaeMotor();
    }, this),

    Commands.waitUntil(()-> {
       return !hasAlgae();
    }),
    Commands.runOnce(()-> {
      reverseAlgaeMotor();
    })
);
}
public Command runAlgaeIntakeManuallyCommand() {
  //this command will allow the drivers to manually run the algae intake
  return Commands.sequence (
    Commands.runOnce(() -> {
      runAlgaeMotor();
    })
  );
}

public Command reverseAlgaeIntakeManuallyCommand() {
  //this command will allow the drivers to manually reverse the algae intake
  return Commands.sequence (
    Commands.runOnce (()-> {
      reverseAlgaeMotor();
    })
  );
}

public Command raiseAlgaeIntakeManually() {
  //this command will allow the drivers to manually raise the algae intake
  return Commands.sequence(
    Commands.runOnce(()-> {
      raiseAlgaeIntake();
    })
  );
}

public Command lowerAlgaeIntakeManually() {
  //this command will allow the drivers to manually lower the algae intake
  return Commands.sequence(
    Commands.runOnce(()-> {
      lowerAlgaeIntake();
  })
  );
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Handler/Motor Velocity" ,algaeEncoder.getVelocity());
    SmartDashboard.putBoolean("Has Algae", hasAlgae());
    SmartDashboard.putBoolean("Algae Intake is Raised", isAlgaeIntakeUp());

  }

  @Override
  protected Command systemCheckCommand() {
    // TODO Auto-generated method stub
    return Commands.sequence(
      Commands.runOnce(
        () -> {
          runAlgaeMotor();
        }, this),
        Commands.waitSeconds(5),

        Commands.runOnce(
          () -> {
            //Checks to make sure that motor is going at minimum speed needeed to intake algae (WAAYY slower than our maximum potential ;)
            if ((algaeEncoder.getVelocity())< 500) {
              addFault("[System Check] Algae Handler velocity too slow", false, true);
          }
          stopAlgaeMotor();
         }, this ),

         Commands.runOnce(
          () -> {
            lowerAlgaeIntake();
          }, this ),
  
         Commands.runOnce(
         () -> {
          raiseAlgaeIntake();
          if (!isAlgaeIntakeUp()) {
            addFault("Hall Effect Sensor does not know algae intake is raised", false, true);  
         }
        }, this ),
       
        Commands.runOnce(
          () -> {
            if (!hasAlgae()) {
              addFault("Limit Switch is not triggered", false, true);
            }
          })
           
      );
  }
}
