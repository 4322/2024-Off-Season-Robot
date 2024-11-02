package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.StationaryShootCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class FourNoteClose extends SequentialCommandGroup {

  public FourNoteClose(
      Superstructure superstructure, Swerve swerve, Shooter shooter, Intake intake) {
    setName("FOUR_NOTE_CLOSE");
    addRequirements(superstructure, swerve, shooter, intake);
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.fourNoteCloseA;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              swerve.resetPose(path.getPreviewStartingHolonomicPose());
            }),
        new FenderShotCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.fourNoteCloseA, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.fourNoteCloseB, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new StationaryShootCommand(swerve, superstructure, shooter),
        new TrajectoryFollowerCommand(() -> Robot.sixNoteAmpSideC, swerve, false, () -> false)
            .beforeStarting(
                () -> {
                  intake.requestIntake();
                  superstructure.requestIntake(true);
                  superstructure.requestVisionSpeaker(false, false, false);
                }),
        new StationaryShootCommand(swerve, superstructure, shooter));
  }
}
