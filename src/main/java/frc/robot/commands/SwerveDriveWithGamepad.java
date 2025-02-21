package frc.robot.commands;

import java.util.Optional;

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

  private static double maxSpeedForChild = 1;

  public SwerveDriveWithGamepad(boolean aimAtGamePiece) {
    this.xVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelTele);
    this.yVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelTele);
    this.angularVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAngularAccelTele);
    addRequirements(RobotContainer.swerve);
  }

  public SwerveDriveWithGamepad() {
    this(false);
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

    SmartDashboard.putNumber("Speed Dial", maxSpeedForChild);
  }

  @Override
  public void execute() {
    maxSpeedForChild = SmartDashboard.getNumber("Speed Dial", 0);
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

    double targetAngularVel = rot * Constants.Swerve.maxAngularVelTele;
    boolean stop = x == 0 && y == 0 && rot == 0;

    // Only take over game piece aim if driver is not rotating and intake is
    // spinning (we don't have
    // a game piece in intake)

    double xVel = this.xVelLimiter.calculate(x * Constants.Swerve.maxVelTele);
    double yVel = this.yVelLimiter.calculate(y * Constants.Swerve.maxVelTele);
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
