package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static Alliance alliance = DriverStation.Alliance.Red;

  public static PathPlannerPath fourNoteCenterA;
  public static PathPlannerPath fourNoteCenterB;
  public static PathPlannerPath fourNoteCenterC;
  public static PathPlannerPath fourNoteCenterD;

  public static PathPlannerPath fourNoteCloseA;
  public static PathPlannerPath fourNoteCloseB;
  public static PathPlannerPath fourNoteCloseC;

  public static PathPlannerPath cheekyThreePieceA;
  public static PathPlannerPath cheekyThreePieceB;

  public static PathPlannerPath fourNoteSourceSideA;
  public static PathPlannerPath fourNoteSourceSideB;
  public static PathPlannerPath fourNoteSourceSideC;

  public static PathPlannerPath sixNoteAmpSideA;
  public static PathPlannerPath sixNoteAmpSideB;
  public static PathPlannerPath sixNoteAmpSideC;
  public static PathPlannerPath sixNoteAmpSideD;
  public static PathPlannerPath sixNoteAmpSideCAlternate;
  public static PathPlannerPath sixNoteAmpSideDAlternate;

  public static PathPlannerPath fiveNoteAmpSideA;
  public static PathPlannerPath fiveNoteAmpSideB;
  public static PathPlannerPath fiveNoteAmpSideC;
  public static PathPlannerPath fiveNoteAmpSideD;
  public static PathPlannerPath fiveNoteAmpSideEA;
  public static PathPlannerPath fiveNoteAmpSideEB;

  public static PathPlannerPath ssrRushA;
  public static PathPlannerPath ssrRushB;
  public static PathPlannerPath ssrSkipPivotC;
  public static PathPlannerPath ssrPivotA;
  public static PathPlannerPath ssrPivotB;
  public static PathPlannerPath ssrPivotC;
  public static PathPlannerPath ssrReturnA;
  public static PathPlannerPath ssrReturnB;
  public static PathPlannerPath ssrReturnC;
  public static PathPlannerPath ssrFromShootPoseA;
  public static PathPlannerPath ssrFromShootPoseB;
  public static PathPlannerPath ssrFromShootPoseC;
  public static PathPlannerPath ssrPreloadShoot;

  public static PathPlannerPath asrRushA;
  public static PathPlannerPath asrRushB;
  public static PathPlannerPath asrPivotB;
  public static PathPlannerPath asrPivotC;
  public static PathPlannerPath asrReturnA;
  public static PathPlannerPath asrReturnB;
  public static PathPlannerPath asrReturnC;
  public static PathPlannerPath asrFromShootPoseA;
  public static PathPlannerPath asrFromShootPoseB;
  public static PathPlannerPath asrFromShootPoseC;

  private boolean requestedHome = false;

  private CANdle leds = new CANdle(31, "Clockwork");

  public static LoggedTunableNumber leftSpeed = new LoggedTunableNumber("Tuning/LeftSpeed", 0.0);
  public static LoggedTunableNumber rightSpeed = new LoggedTunableNumber("Tuning/RightSpeed", 0.0);

  public static LoggedTunableNumber elevatorHeight =
      new LoggedTunableNumber("Tuning/ElevatorHeight", 0.0);
  public static LoggedTunableNumber pivotAngle =
      new LoggedTunableNumber("Tuning/PivotAngleDegrees", 0.0);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    if (isReal()) {
      Logger.addDataReceiver(
          new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      RobotController.setBrownoutVoltage(7.5);
    }

    Logger.start();
    Logger.disableDeterministicTimestamps();
    Logger.disableConsoleCapture();

    fourNoteCenterA = PathPlannerPath.fromPathFile("Four Note Center A");
    fourNoteCenterB = PathPlannerPath.fromPathFile("Four Note Center B");
    fourNoteCenterC = PathPlannerPath.fromPathFile("Four Note Center C");
    fourNoteCenterD = PathPlannerPath.fromPathFile("Four Note Center D");

    fourNoteCloseA = PathPlannerPath.fromPathFile("Four Note Close A");
    fourNoteCloseB = PathPlannerPath.fromPathFile("Four Note Close B");
    fourNoteCloseC = PathPlannerPath.fromPathFile("Four Note Close C");

    cheekyThreePieceA = PathPlannerPath.fromPathFile("Cheeky Three Piece A");
    cheekyThreePieceB = PathPlannerPath.fromPathFile("Cheeky Three Piece B");

    sixNoteAmpSideA = PathPlannerPath.fromPathFile("Six Note A");
    sixNoteAmpSideB = PathPlannerPath.fromPathFile("Six Note B");
    sixNoteAmpSideC = PathPlannerPath.fromPathFile("Six Note C");
    sixNoteAmpSideD = PathPlannerPath.fromPathFile("Six Note D");
    sixNoteAmpSideCAlternate = PathPlannerPath.fromPathFile("Six Note C Alternate");
    sixNoteAmpSideDAlternate = PathPlannerPath.fromPathFile("Six Note D Alternate");

    fiveNoteAmpSideA = PathPlannerPath.fromPathFile("Five Note Amp Side A");
    fiveNoteAmpSideB = PathPlannerPath.fromPathFile("Five Note Amp Side B");
    fiveNoteAmpSideC = PathPlannerPath.fromPathFile("Five Note Amp Side C");
    fiveNoteAmpSideD = PathPlannerPath.fromPathFile("Five Note Amp Side D");
    fiveNoteAmpSideEA = PathPlannerPath.fromPathFile("Five Note Amp Side EA");
    fiveNoteAmpSideEB = PathPlannerPath.fromPathFile("Five Note Amp Side EB");

    fourNoteSourceSideA = PathPlannerPath.fromPathFile("Four Note Source Side A");
    fourNoteSourceSideB = PathPlannerPath.fromPathFile("Four Note Source Side B");
    fourNoteSourceSideC = PathPlannerPath.fromPathFile("Four Note Source Side C");

    ssrRushA = PathPlannerPath.fromPathFile("SSR Rush A");
    ssrRushB = PathPlannerPath.fromPathFile("SSR Rush B");
    ssrSkipPivotC = PathPlannerPath.fromPathFile("SSR Skip Pivot C");
    ssrPivotA = PathPlannerPath.fromPathFile("SSR Pivot A");
    ssrPivotB = PathPlannerPath.fromPathFile("SSR Pivot B");
    ssrPivotC = PathPlannerPath.fromPathFile("SSR Pivot C");
    ssrReturnA = PathPlannerPath.fromPathFile("SSR Return A");
    ssrReturnB = PathPlannerPath.fromPathFile("SSR Return B");
    ssrReturnC = PathPlannerPath.fromPathFile("SSR Return C");
    ssrFromShootPoseA = PathPlannerPath.fromPathFile("SSR From Shoot Pose A");
    ssrFromShootPoseB = PathPlannerPath.fromPathFile("SSR From Shoot Pose B");
    ssrFromShootPoseC = PathPlannerPath.fromPathFile("SSR From Shoot Pose C");
    ssrPreloadShoot = PathPlannerPath.fromPathFile("SSR Preload Shoot");

    asrRushA = PathPlannerPath.fromPathFile("ASR Rush A");
    asrRushB = PathPlannerPath.fromPathFile("ASR Rush B");
    asrPivotB = PathPlannerPath.fromPathFile("ASR Pivot B");
    asrPivotC = PathPlannerPath.fromPathFile("ASR Pivot C");
    asrReturnA = PathPlannerPath.fromPathFile("ASR Return A");
    asrReturnB = PathPlannerPath.fromPathFile("ASR Return B");
    asrReturnC = PathPlannerPath.fromPathFile("ASR Return C");
    asrFromShootPoseA = PathPlannerPath.fromPathFile("ASR From Shoot Pose A");
    asrFromShootPoseB = PathPlannerPath.fromPathFile("ASR From Shoot Pose B");
    asrFromShootPoseC = PathPlannerPath.fromPathFile("ASR From Shoot Pose C");

    m_robotContainer.configureAutonomousSelector();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
      double xError =
          m_robotContainer.getAutoStartingPose().getX()
              - RobotContainer.swerve.getAutoPose().getX();
      double yError =
          m_robotContainer.getAutoStartingPose().getY()
              - RobotContainer.swerve.getAutoPose().getY();
      double rotError =
          m_robotContainer.getAutoStartingPose().getRotation().getDegrees()
              - RobotContainer.swerve.getAutoPose().getRotation().getDegrees();

      if (xError < -0.05) {
        if (alliance == Alliance.Red) {
          leds.setLEDs(0, 0, 255, 0, 29, 36);
        } else {
          leds.setLEDs(255, 0, 0, 0, 29, 36);
        }

      } else if (xError > 0.05) {
        if (alliance == Alliance.Red) {
          leds.setLEDs(255, 0, 0, 0, 29, 36);
        } else {
          leds.setLEDs(0, 0, 255, 0, 29, 36);
        }
      } else {
        leds.setLEDs(0, 255, 0, 0, 29, 36);
      }

      if (yError < -0.05) {
        if (alliance == Alliance.Red) {
          leds.setLEDs(255, 0, 0, 0, 8, 20);
          leds.setLEDs(0, 0, 0, 0, 66, 20);
        } else {
          leds.setLEDs(255, 0, 0, 0, 66, 20);
          leds.setLEDs(0, 0, 0, 0, 8, 20);
        }

      } else if (yError > 0.05) {
        if (alliance == Alliance.Red) {
          leds.setLEDs(255, 0, 0, 0, 66, 20);
          leds.setLEDs(0, 0, 0, 0, 8, 20);
        } else {
          leds.setLEDs(255, 0, 0, 0, 8, 20);
          leds.setLEDs(0, 0, 0, 0, 66, 20);
        }
      } else {
        leds.setLEDs(0, 255, 0, 0, 8, 20);
        leds.setLEDs(0, 255, 0, 0, 66, 20);
      }

    } else {
      if (RobotContainer.superstructure.getSystemState() == SuperstructureState.PRE_CLIMB) {
        leds.clearAnimation(0);
        leds.setLEDs(255, 0, 0, 0, 8, Constants.LED_NUM);
        RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
      } else if (RobotContainer.superstructure.hasPiece()) {
        leds.clearAnimation(0);
        leds.setLEDs(0, 0, 255, 0, 8, Constants.LED_NUM);
        RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0.25);
      } else if (RobotContainer.intake.hasPiece()) {
        leds.clearAnimation(0);
        RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
      } else {
        leds.clearAnimation(0);
        leds.setLEDs(0, 0, 0, 0, 8, Constants.LED_NUM);
        RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
      }
    }

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      alliance = allianceOptional.get();
    }
  }

  @Override
  public void disabledInit() {
    RobotContainer.intake.requestIdle();
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.isAutonomous()
            && m_robotContainer.getAutonomousCommand().getName() == "AMP_SIDE_RUSH"
        || m_robotContainer.getAutonomousCommand().getName() == "AMP_SIDE_RUSH_23") {
      RobotContainer.superstructure.requestVisionSpeaker(true, false, false);
      RobotContainer.shooter.requestVisionSpeaker(false);
    } else {
      RobotContainer.superstructure.requestFender(false, false);
      RobotContainer.shooter.requestIdle();
    }
    RobotContainer.swerve.clearHeadingLock(); // don't rotate when re-enabling
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    RobotContainer.superstructure.registerAutoPreload();

    if (!requestedHome) {
      RobotContainer.superstructure.requestHome();
      requestedHome = true;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.shooter.requestIdle();
    RobotContainer.swerve
        .clearHeadingLock(); // Needed to prevent pseudo auto rotate from spinning wildly in place
    RobotController.setBrownoutVoltage(5.75); // roboRIO 2 only
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!requestedHome) {
      RobotContainer.superstructure.requestHome();
      requestedHome = true;
    }

    RobotContainer.shooter.requestIdle();
    RobotContainer.swerve
        .clearHeadingLock(); // Needed to prevent pseudo auto rotate from spinning wildly in place
    RobotController.setBrownoutVoltage(5.75);

    RobotContainer.superstructure.requestVisionSpeaker(false, false, false);
    RobotContainer.superstructure.requestFender(false, false);
  }

  @Override
  public void teleopPeriodic() {
    /* Intake requests */
    if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
      RobotContainer.intake.requestIntake();
      RobotContainer.superstructure.requestIntake(true);
    } else if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
      RobotContainer.intake.requestSpit();
    } else {
      RobotContainer.superstructure.requestIntake(false);
      RobotContainer.intake.requestIdle();
    }

    /* Superstructure spit requests */
    if (RobotContainer.operator.getXButton()) {
      RobotContainer.superstructure.requestSpit(true);
    } else {
      RobotContainer.superstructure.requestSpit(false);
    }

    /* Climb requests */
    if (RobotContainer.operator.getRightBumperPressed()) {
      RobotContainer.superstructure.requestNextClimbState();
    } else if (RobotContainer.operator.getLeftBumperPressed()) {
      RobotContainer.superstructure.requestPrevClimbState();
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
