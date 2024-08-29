package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.LowPassCommand;
import frc.robot.commands.PassCommand;
import frc.robot.commands.TeleopShootCommand;
import frc.robot.commons.BreadUtil;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.VisionSupplier;
import frc.robot.vision.photonvision.PhotonNoteDetection;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public static ShooterIO shooterIO = new ShooterIO() {};
  public static Shooter shooter = new Shooter(shooterIO);

  public static IntakeIO intakeIO = new IntakeIO() {};
  public static Intake intake = new Intake(intakeIO);

  public static ElevatorIO elevatorIO = new ElevatorIO() {};
  public static PivotIO pivotIO = new PivotIO() {};
  public static FeederIO feederIO = new FeederIO() {};
  public static Superstructure superstructure = new Superstructure(elevatorIO, pivotIO, feederIO);
  public static final Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  // April tag cameras
  /*
  public static final BreadPhotonCamera frontLeftCamera = new BreadPhotonCamera("front-left");
  public static final BreadPhotonCamera frontRightCamera = new BreadPhotonCamera("front-right");
  public static final BreadPhotonCamera backLeftCamera = new BreadPhotonCamera("back-left");
  public static final BreadPhotonCamera backRightCamera = new BreadPhotonCamera("back-right");

  // Note detection cameras
  public static final PhotonCamera leftObjCamera = new PhotonCamera("left-obj");
  public static final PhotonCamera rightObjCamera = new PhotonCamera("right-obj");

  public static final PhotonAprilTagVision aprilTagVision =
      new PhotonAprilTagVision(frontLeftCamera, frontRightCamera, backLeftCamera, backRightCamera);
  public static final PhotonNoteDetection noteDetection =
      new PhotonNoteDetection(leftObjCamera, rightObjCamera);
  */

  public static final PhotonNoteDetection noteDetection = new PhotonNoteDetection();
  public static final VisionSupplier visionSupplier = new VisionSupplier();
  public static AutonomousSelector autonomousSelector;

  public RobotContainer() {
    configureBindings();
    // configureAprilTagVision();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        new RunCommand(
            () -> {
              double x = BreadUtil.deadband(driver.getLeftY(), 0.16);
              double y = BreadUtil.deadband(driver.getLeftX(), 0.16);
              double omega = BreadUtil.deadband(driver.getRightX(), 0.16);

              double dx = 0;
              double dy = 0;

              if (Robot.alliance == DriverStation.Alliance.Blue) {
                dx = Math.pow(-x, 1) * 6.0;
                dy = Math.pow(-y, 1) * 6.0;

              } else {
                dx = Math.pow(-x, 1) * -1 * 6.0;
                dy = Math.pow(-y, 1) * -1 * 6.0;
              }
              double rot = Math.pow(-omega, 3) * 12.0;
              swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

              if (driver.getRawButtonPressed(XboxController.Button.kStart.value)) {
                if (Robot.alliance == Alliance.Blue) {
                  swerve.resetPose(new Pose2d());
                } else {
                  swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
                }
              }
            },
            swerve));

    new JoystickButton(driver, XboxController.Button.kB.value)
        .whileTrue(new TeleopShootCommand(swerve));

    new JoystickButton(driver, XboxController.Button.kX.value)
        .whileTrue(new PassCommand(swerve, superstructure, shooter));

    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new AmpCommand(swerve));

    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whileTrue(new LowPassCommand(swerve, superstructure, shooter));

    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileTrue(new FenderShotCommand(swerve, superstructure, shooter));
  }

  /*
  private void configureAprilTagVision() {
    aprilTagVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
  }
  */

  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void configureAutonomousSelector() {
    autonomousSelector = new AutonomousSelector(swerve, superstructure, shooter, intake);
  }
}
