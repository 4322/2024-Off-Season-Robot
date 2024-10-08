package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class StationaryShootCommand extends Command {
  // Subsystems
  private Swerve swerve;
  private Superstructure superstructure;
  private Shooter shooter;

  // Feedback controllers
  private PIDController turnPID = new PIDController(10, 0, 0);

  public StationaryShootCommand(Swerve swerve, Superstructure superstructure, Shooter shooter) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.shooter = shooter;

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, superstructure, shooter);
  }

  @Override
  public void execute() {
    // Calculuate swerve output
    double turnSetpoint = RobotContainer.visionSupplier.robotToSpeakerAngle().getRadians();
    double currentRotation = swerve.getPose().getRotation().getRadians();

    double thetaOutput = turnPID.calculate(currentRotation, turnSetpoint);

    // Subsystem Requests
    swerve.requestVelocity(new ChassisSpeeds(0, 0, thetaOutput), true);

    shooter.requestVisionSpeaker(false);
    superstructure.requestVisionSpeaker(true, true, false);

    Logger.recordOutput(
        "AimAtSpeakerCommand/MeasurementDegrees>", Units.radiansToDegrees(currentRotation));
    Logger.recordOutput(
        "AimAtSpeakerCommand/SetpointDegrees", Units.radiansToDegrees(turnSetpoint));
  }

  @Override
  public boolean isFinished() {
    return !superstructure.hasPiece()
        && superstructure.getSystemState() != SuperstructureState.VISION_SPEAKER;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestVisionSpeaker(false, false, false);
  }
}
