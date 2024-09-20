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
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorpivot.ElevatorIO;
import frc.robot.subsystems.elevatorpivot.ElevatorIOKrakenX60;
import frc.robot.subsystems.elevatorpivot.PivotIO;
import frc.robot.subsystems.elevatorpivot.PivotIOKrakenX60;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOFalcon500;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon500;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakenX60;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.VisionSupplier;
import frc.robot.vision.photonvision.BreadPhotonCamera;
import frc.robot.vision.photonvision.PhotonAprilTagVision;
import frc.robot.vision.photonvision.PhotonNoteDetection;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public static ShooterIO shooterIO = new ShooterIOKrakenX60();
  public static Shooter shooter = new Shooter(shooterIO);

  public static IntakeIO intakeIO = new IntakeIOFalcon500();
  public static Intake intake = new Intake(intakeIO);

  public static ElevatorIO elevatorIO = new ElevatorIOKrakenX60();
  public static PivotIO pivotIO = new PivotIOKrakenX60();
  public static FeederIO feederIO = new FeederIOFalcon500();
  public static Superstructure superstructure = new Superstructure(elevatorIO, pivotIO, feederIO);
  public static final Swerve swerve =
      new Swerve(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  // April tag cameras
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
  // public static final PhotonNoteDetection noteDetection = new PhotonNoteDetection();
  public static final VisionSupplier visionSupplier = new VisionSupplier();
  public static AutonomousSelector autonomousSelector;

  public RobotContainer() {
    configureBindings();
    configureAprilTagVision();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        new RunCommand(
            () -> {
              // Raw inputs
              double x = -driver.getLeftY();
              double y = -driver.getLeftX();
              double omega = BreadUtil.cartesianDeadband(-driver.getRightX(), Constants.Swerve.rotDeadband);

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
                dx *= 6.0;
                dy *= 6.0;

              } else {
                dx *= -6.0;
                dy *= -6.0;
              }
              double rot = Math.pow(omega, 3) * 6.0;
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

  private void configureAprilTagVision() {
    aprilTagVision.setDataInterfaces(swerve::getPose, swerve::addVisionData);
  }

  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void configureAutonomousSelector() {
    autonomousSelector = new AutonomousSelector(swerve, superstructure, shooter, intake);
  }
}
