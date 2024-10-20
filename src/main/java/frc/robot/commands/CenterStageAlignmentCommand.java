package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.swerve.Swerve;

public class CenterStageAlignmentCommand extends Command {

  // Subsystems
  private Swerve swerve;

  // Feedback controllers
  private PIDController turnPID = new PIDController(7, 0, 0);

  public CenterStageAlignmentCommand(Swerve swerve) {
    this.swerve = swerve;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // Calculate swerve outputs
    double setpoint = 0;
    // double setpoint = RobotContainer.visionSupplier.robotToAmpAngle().getRadians();
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
      dx *= 2.0;
      dy *= 2.0;

    } else {
      dx *= -2.0;
      dy *= -2.0;
    }

    // Apply swerve Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);

    // Apply superstructure requests
    // if (Math.abs(setpoint - measurement) < Units.degreesToRadians(15.0)) {
    //   RobotContainer.superstructure.requestAmp(true, RobotContainer.driver.getYButton());
    //   RobotContainer.shooter.requestAmp();
    // } else {
    //   RobotContainer.superstructure.requestAmp(false, false);
    //   RobotContainer.shooter.requestIdle();
    // }
  }

  @Override
  public void end(boolean interrupted) {
  }
}
