package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class TeleopShootCommand extends Command {

  // Subsystems
  private Swerve swerve;

  // Feedback controllers
  LoggedTunableNumber p = new LoggedTunableNumber("TeleopShootCommand/p", 7);
  LoggedTunableNumber d = new LoggedTunableNumber("TeleopShootCommand/d", 0.2);
  private PIDController turnPID = new PIDController(p.get(), 0, d.get());

  public TeleopShootCommand(Swerve swerve) {
    this.swerve = swerve;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Set p and d of the controller
    if (p.hasChanged(0)) {
      turnPID.setP(p.get());
    }

    if (d.hasChanged(0)) {
      turnPID.setD(d.get());
    }
    // Swerve turn PID calcs
    double setpoint = RobotContainer.visionSupplier.robotToSpeakerAngle().getRadians();
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
      dx *= 1.75;
      dy *= 1.75;

    } else {
      dx *= 1.75;
      dy *= 1.75;
    }

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(dx, dy, output), true);
    // RobotContainer.superstructure.requestFender(
    //     true, RobotContainer.swerve.atAngularSetpoint(setpoint));
    // RobotContainer.shooter.requestFender();

    double tolerance = RobotContainer.visionSupplier.getSwerveAngleTolerance();

    if (Math.abs(dx) < 0.01 && Math.abs(dy) < 0.01) {
      RobotContainer.superstructure.requestVisionSpeaker(
          true,
          RobotContainer.swerve.atAngularSetpoint(setpoint, tolerance)
              && RobotContainer.swerve.notRotating(),
          RobotContainer.operator.getYButton());
    } else {
      RobotContainer.superstructure.requestVisionSpeaker(
          true,
          RobotContainer.swerve.atAngularSetpoint(setpoint, tolerance),
          RobotContainer.operator.getYButton());
    }
    RobotContainer.shooter.requestVisionSpeaker(false);

    // Logs
    Logger.recordOutput(
        "TeleopShootCommand/MeasurementDegrees", Units.radiansToDegrees(measurement));
    Logger.recordOutput("TeleopShootCommand/SetpointDegrees", Units.radiansToDegrees(setpoint));
    Logger.recordOutput(
        "TeleopShootCommand/atTurnSetpoint",
        RobotContainer.swerve.atAngularSetpoint(setpoint, tolerance));
    Logger.recordOutput("TeleopShootCommand/Tolerance", tolerance);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.requestIdle();
    RobotContainer.superstructure.requestVisionSpeaker(false, false, false);
    // RobotContainer.superstructure.requestFender(false, false);
  }
}
