package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.modes.AmpSideRush;
import frc.robot.autonomous.modes.CheekyThreePiece;
import frc.robot.autonomous.modes.DriveBackTwo;
import frc.robot.autonomous.modes.FiveNoteAmpSide;
import frc.robot.autonomous.modes.FourNoteAmpSide;
import frc.robot.autonomous.modes.FourNoteCenter;
import frc.robot.autonomous.modes.FourNoteClose;
import frc.robot.autonomous.modes.FourNoteSourceSide;
import frc.robot.autonomous.modes.Preload;
import frc.robot.autonomous.modes.SixNoteAmpSide;
import frc.robot.autonomous.modes.SixNoteAmpSideAlternate;
import frc.robot.autonomous.modes.SourceSideRush12;
import frc.robot.autonomous.modes.SourceSideRush21;
import frc.robot.autonomous.modes.ThreeNoteCenter;
import frc.robot.commons.AutoStartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  private ArrayList<AutoStartPose> autoNames = new ArrayList<AutoStartPose>();

  public AutonomousSelector(
      Swerve swerve, Superstructure superstructure, Shooter shooter, Intake intake) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
    autoNames.add(new AutoStartPose("DO_NOTHING", new Pose2d()));

    autonomousSelector.addOption(
        "THREE_NOTE_CENTER", new ThreeNoteCenter(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose(
            "THREE_NOTE_CENTER", Robot.fourNoteCenterA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "CHEEKY_THREE_PIECE", new CheekyThreePiece(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose(
            "CHEEKY_THREE_PIECE", Robot.cheekyThreePieceA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "SIX_NOTE_AMP_SIDE", new SixNoteAmpSide(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose(
            "SIX_NOTE_AMP_SIDE", Robot.sixNoteAmpSideA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "SIX_NOTE_AMP_SIDE_ALTERNATE",
        new SixNoteAmpSideAlternate(superstructure, swerve, shooter, intake));

    autonomousSelector.addOption(
        "FOUR_NOTE_CENTER", new FourNoteCenter(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose(
            "FOUR_NOTE_CENTER", Robot.fourNoteCenterA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "FOUR_NOTE_AMP_SIDE", new FourNoteAmpSide(superstructure, swerve, shooter, intake));

    autonomousSelector.addOption(
        "FIVE_NOTE_AMP_SIDE", new FiveNoteAmpSide(superstructure, swerve, shooter, intake));

    autonomousSelector.addOption("PRELOAD", new Preload(superstructure, swerve, shooter, intake));
    autoNames.add(new AutoStartPose("PRELOAD", new Pose2d()));

    autonomousSelector.addOption(
        "DRIVE_BACK_TWO", new DriveBackTwo(superstructure, swerve, shooter, intake));

    autonomousSelector.addOption(
        "FOUR_NOTE_SOURCE_SIDE", new FourNoteSourceSide(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose(
            "FOUR_NOTE_SOURCE_SIDE", Robot.fourNoteSourceSideA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "SOURCE_SIDE_RUSH_12", new SourceSideRush12(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose("SOURCE_SIDE_RUSH_12", Robot.ssrRushA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "SOURCE_SIDE_RUSH_21", new SourceSideRush21(superstructure, swerve, shooter, intake));

    autonomousSelector.addOption(
        "AMP_SIDE_RUSH", new AmpSideRush(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose("AMP_SIDE_RUSH", Robot.asrRushA.getPreviewStartingHolonomicPose()));

    autonomousSelector.addOption(
        "FOUR_NOTE_CLOSE", new FourNoteClose(superstructure, swerve, shooter, intake));
    autoNames.add(
        new AutoStartPose(
            "FOUR_NOTE_CLOSE", Robot.fourNoteCloseA.getPreviewStartingHolonomicPose()));

    SmartDashboard.putData("Autonomus Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }

  public Pose2d getStartingPose() {
    for (AutoStartPose auto : autoNames) {
      if (auto.getName().equals(autonomousSelector.getSelected().getName())) {
        return auto.getStartingPose();
      }
    }
    return new Pose2d();
  }
}
