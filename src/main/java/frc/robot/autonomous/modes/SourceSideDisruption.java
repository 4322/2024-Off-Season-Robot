package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class SourceSideDisruption extends SequentialCommandGroup {
  public SourceSideDisruption(Swerve swerve, Intake intake, Superstructure superstructure) {
    setName("SOURCE_SIDE_DISRUPTION");
    addRequirements(swerve, intake, superstructure);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.sourceSideDisruptionA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new TrajectoryFollowerCommand(() -> Robot.sourceSideDisruptionA, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                }),
        new TrajectoryFollowerCommand(
            () -> Robot.sourceSideDisruptionB, swerve, false, () -> false));
  }
}
