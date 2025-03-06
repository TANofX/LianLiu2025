package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDriveWithGamepad extends Command {
  private final SlewRateLimiter xVelLimiter;
  private final SlewRateLimiter yVelLimiter;
  private final SlewRateLimiter angularVelLimiter;
  private DoubleSupplier heightFraction;
  private static double speedSet = 1;


  public SwerveDriveWithGamepad(boolean aimAtGamePiece) {
    this.xVelLimiter = new SlewRateLimiter(Constants.Swerve.TELEOP_MAX_ACCELERATION);
    this.yVelLimiter = new SlewRateLimiter(Constants.Swerve.TELEOP_MAX_ACCELERATION);
    this.angularVelLimiter = new SlewRateLimiter(Constants.Swerve.TELEOP_MAX_ANGULAR_ACCELERATION);
    heightFraction = () -> {
      return 0.0;
    };

    addRequirements(RobotContainer.swerve);

  }

  public SwerveDriveWithGamepad(DoubleSupplier heightFrac) {
    this(false);
    heightFraction = heightFrac;
  }

  @Override
  public void initialize() {
    // ChassisSpeeds currentSpeeds = RobotContainer.swerve.getCurrentSpeeds();
    // Translation2d hack = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
    //     .rotateBy(RobotContainer.swerve.getPose().getRotation());
    // this.xVelLimiter.reset(hack.getX());
    // this.yVelLimiter.reset(hack.getY());
    // this.angularVelLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
    this.xVelLimiter.reset(0);
    this.yVelLimiter.reset(0);
    this.angularVelLimiter.reset(0);

    SmartDashboard.putNumber("Speed Dial", speedSet);
  }

  @Override
  public void execute() {
    speedSet = SmartDashboard.getNumber("Speed Dial", 0);
    double maxSpeedForChild = speedSet * (0.25 + 0.75 * (1 - heightFraction.getAsDouble()));

    double x = -RobotContainer.driver.getLeftY() * maxSpeedForChild;
    x = Math.copySign(x * x, x);
    double y = -RobotContainer.driver.getLeftX()* maxSpeedForChild;
    y = Math.copySign(y * y, y);
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
      x = -1 * x;
      y = -1 * y;
    }
    double rot;
      rot = -RobotContainer.driver.getRightX() * maxSpeedForChild;
      rot = Math.copySign(rot * rot, rot);

    double targetAngularVel = rot * Constants.Swerve.TELEOP_MAX_ANGULAR_VELOCITY;
    boolean stop = x == 0 && y == 0 && rot == 0;

    // Only take over game piece aim if driver is not rotating and intake is
    // spinning (we don't have
    // a game piece in intake)

    double xVel = this.xVelLimiter.calculate(x * Constants.Swerve.TELEOP_MAX_VELOCITY);
    double yVel = this.yVelLimiter.calculate(y * Constants.Swerve.TELEOP_MAX_VELOCITY);
    double angularVel = this.angularVelLimiter.calculate(targetAngularVel);

    SmartDashboard.putNumber("Joystick/X", xVel);
    SmartDashboard.putNumber("Joystick/Y", yVel);
    SmartDashboard.putNumber("Joystick/Ang", angularVel);

    if (stop)
      RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));

    RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
