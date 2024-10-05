package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class LowPassCommand extends Command {

  private Swerve swerve;
  private Superstructure superstructure;
  private Shooter shooter;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0.2);

  public LowPassCommand(Swerve swerve, Superstructure superstructure, Shooter shooter) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.shooter = shooter;

    addRequirements(swerve, superstructure, shooter);

    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    // Swerve turn PID calcs
    double setpoint = RobotContainer.visionSupplier.robotToLowPassingAngle().getRadians();
    double measurement = swerve.getPose().getRotation().getRadians();

    double output = turnPID.calculate(measurement, setpoint);

    // Raw inputs
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();

    // Apply polar deadband
    double[] polarDriveCoord = BreadUtil.polarDeadband(x, y);
    double driveMag = polarDriveCoord[0];
    double driveTheta = polarDriveCoord[1];

    // Quadratic scaling of drive inputs
    driveMag = driveMag * driveMag;

    // Normalize vector magnitude so as not to give an invalid input
    if (driveMag > 1) {
      driveMag = 1;
    }

    double dx = driveMag * Math.cos(driveTheta);
    double dy = driveMag * Math.sin(driveTheta);

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      dx *= 5.0;
      dy *= 5.0;

    } else {
      dx *= -5.0;
      dy *= -5.0;
    }

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);

    // RobotContainer.superstructure.requestPass(true, RobotContainer.driver.getYButton());
    // RobotContainer.shooter.requestPass();
    RobotContainer.shooter.requestLowPass();
    RobotContainer.superstructure.requestLowPass(
        true, RobotContainer.swerve.atAngularSetpoint(setpoint));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle();
    // superstructure.requestPass(false, false);
    superstructure.requestLowPass(false, false);
  }
}
