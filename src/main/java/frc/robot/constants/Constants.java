package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShotParameter;

// By default these constants are the **Beta** constants
public class Constants {

  public static final RobotType robot = RobotType.BETA;

  public static final boolean driveEnabled = true;
  public static final boolean pseudoAutoRotateEnabled = true;
  public static final boolean tuningMode = false;
  public static final boolean visionEnabled = true;

  public static final double pseudoAutoRotateMinMetersPerSec =
      0.6; // disable below this speed for fine adjustments

  public static final double FALCON_FREE_SPEED = 6380.0;
  public static final double KRAKEN_FREE_SPEED = 6000.0;
  public static final double GOAL_INWARD_SHIFT = 0.0;
  public static final int LED_NUM = 86;

  public enum RobotType {
    BETA,
    GAMMA
  }

  /* Constants pertaining to the intake */
  public static class Intake {

    /* IDs */
    public static final int INTAKE_ID;
    public static final int VECTOR_ID;

    /* Setpoints and tolerances */
    public static final double INTAKE_SPEED;
    public static final double SPIT_SPEED;
    public static final double FEED_SPEED;

    /* Physical Measurements */
    public static final InvertedValue INTAKE_INVERSION;
    public static final InvertedValue VECTOR_INVERSION;

    static {
      if (robot == RobotType.BETA) {
        INTAKE_ID = 5;
        VECTOR_ID = 6;

        INTAKE_SPEED = 1.0;
        SPIT_SPEED = -0.4;
        FEED_SPEED = 0.8;

        // Clockwork: Intake must not be inverted due to use of falcon
        INTAKE_INVERSION = InvertedValue.CounterClockwise_Positive;
        VECTOR_INVERSION = InvertedValue.Clockwise_Positive;
      } else {
        INTAKE_ID = 5;
        VECTOR_ID = 6;

        INTAKE_SPEED = 1.0;
        SPIT_SPEED = -0.4;
        FEED_SPEED = 0.8;

        INTAKE_INVERSION = InvertedValue.CounterClockwise_Positive;
        VECTOR_INVERSION = InvertedValue.Clockwise_Positive;
      }
    }
  }

  /* Constants pertaining to the swerve drive */
  public static class Swerve {

    public static final double SWERVE_COAST_TRESHOLD_MPS;
    public static final double SWERVE_COAST_TRESHOLD_SEC;
    public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD;
    public static final double SWERVE_ANGULAR_ERROR_TOLERANCE_RAD_P_S;
    public static final double driveDeadband = 0.1;
    public static final double rotDeadband = 0.1;

    public static final double pseudoAutoRotatekP = 6;
    public static final double pseudoAutoRotatekI = 0;
    public static final double pseudoAutoRotatekD = 0.0;
    public static final double pseudoAutoRotateDegTolerance = Units.degreesToRadians(1.5);
    public static final double inhibitPseudoAutoRotateRadPerSec = Units.degreesToRadians(4);

    static {
      if (robot == RobotType.BETA) {
        SWERVE_COAST_TRESHOLD_MPS = 0.05;
        SWERVE_COAST_TRESHOLD_SEC = 5.0;
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(7);
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD_P_S = Units.degreesToRadians(20.0);
      } else {
        SWERVE_COAST_TRESHOLD_MPS = 0.05;
        SWERVE_COAST_TRESHOLD_SEC = 5.0;
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD = Units.degreesToRadians(7);
        SWERVE_ANGULAR_ERROR_TOLERANCE_RAD_P_S = Units.degreesToRadians(20.0);
      }
    }
  }

  /* Constants pertaining to the elevator */
  public static class Elevator {

    /* IDs */
    public static final int ELEVATOR_LEFT_ID;
    public static final int ELEVATOR_RIGHT_ID;

    /* Elevator setpoints speeds and positions */
    public static final double ELEVATOR_HOMING_VOLTAGE;

    public static final double ELEVATOR_IDLE_HEIGHT;

    public static final double ELEVATOR_INTAKE_HEIGHT;

    public static final double ELEVATOR_SPIT_HEIGHT;

    public static final double ELEVATOR_SPEAKER_DEFENSE_HEIGHT;

    public static final double ELEVATOR_SPEAKER_SHORT_HEIGHT;

    public static final double ELEVATOR_AMP_HEIGHT;

    public static final double ELEVATOR_PRE_CLIMB_HEIGHT;

    public static final double ELEVATOR_HALF_CLIMB_HEIGHT;

    public static final double ELEVATOR_CLIMBED_HEIGHT;

    public static final double ELEVATOR_TRAP_HEIGHT;

    public static final double ELEVATOR_FENDER_HEIGHT;

    /* Physical Measurements */
    public static final double ELEVATOR_SPOOL_DIAMETER;
    public static final double ELEVATOR_SETPOINT_TOLERANCE_METERS;
    public static final double ELEVATOR_HOMING_TRESHOLD_SEC;
    public static final double ELEVATOR_HOMING_TRESHOLD_MPS;
    public static final double ELEVATOR_GEAR_RATIO;
    public static final double ELEVATOR_MAX_SPEED;
    public static final InvertedValue ELEVATOR_LEFT_INVERSION;
    public static final double ELEVATOR_MIN_HEIGHT;
    public static final double ELEVATOR_END_OF_DANGER_ZONE;
    public static final double ELEVATOR_MAX_HEIGHT;
    public static final double ELEVATOR_PASS_HEIGHT;
    public static final double ELEVATOR_LOW_PASS_HEIGHT;

    static {
      if (robot == RobotType.BETA) {

        ELEVATOR_LEFT_ID = 26;
        ELEVATOR_RIGHT_ID = 27;

        ELEVATOR_HOMING_VOLTAGE = -3.0;

        ELEVATOR_IDLE_HEIGHT = 0.015;

        ELEVATOR_INTAKE_HEIGHT = 0.015;

        ELEVATOR_SPIT_HEIGHT = 0.2;

        ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.3;

        ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.125;

        ELEVATOR_AMP_HEIGHT = 0.405 + Units.inchesToMeters(1);

        ELEVATOR_PRE_CLIMB_HEIGHT = 0.62;

        ELEVATOR_HALF_CLIMB_HEIGHT = 0.5;

        ELEVATOR_CLIMBED_HEIGHT = Units.inchesToMeters(0.0);

        ELEVATOR_TRAP_HEIGHT = 0.6308 - Units.inchesToMeters(2);

        ELEVATOR_FENDER_HEIGHT = 0.22; // 0.4

        ELEVATOR_PASS_HEIGHT = 0.4;

        ELEVATOR_LOW_PASS_HEIGHT = 0.05;

        ELEVATOR_SPOOL_DIAMETER = Units.inchesToMeters(1.463);
        ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
        ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
        ELEVATOR_HOMING_TRESHOLD_MPS = 0.01;
        ELEVATOR_GEAR_RATIO = 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (36.0 / 30.0));
        ELEVATOR_MAX_SPEED =
            ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
                * Math.PI
                * ELEVATOR_SPOOL_DIAMETER;
        ELEVATOR_LEFT_INVERSION = InvertedValue.CounterClockwise_Positive;
        ELEVATOR_MIN_HEIGHT = 0.0;
        ELEVATOR_END_OF_DANGER_ZONE = 0.08740336105246657; // Top gun reference?!?!?!
        ELEVATOR_MAX_HEIGHT = 0.6387244800740626;
      } else {

        ELEVATOR_LEFT_ID = 26;
        ELEVATOR_RIGHT_ID = 27;

        ELEVATOR_HOMING_VOLTAGE = -3.0;

        ELEVATOR_IDLE_HEIGHT = 0.125;

        ELEVATOR_INTAKE_HEIGHT = 0.094;

        ELEVATOR_SPIT_HEIGHT = 0.2;

        ELEVATOR_SPEAKER_DEFENSE_HEIGHT = 0.3;

        ELEVATOR_SPEAKER_SHORT_HEIGHT = 0.125;

        ELEVATOR_AMP_HEIGHT = 0.48;

        ELEVATOR_PRE_CLIMB_HEIGHT = 0.4;

        ELEVATOR_HALF_CLIMB_HEIGHT = 0.5;

        ELEVATOR_CLIMBED_HEIGHT = 0.0;

        ELEVATOR_TRAP_HEIGHT = 0.47;

        ELEVATOR_FENDER_HEIGHT = 0.4;

        ELEVATOR_PASS_HEIGHT = 0.4;

        ELEVATOR_LOW_PASS_HEIGHT = 0.05;

        ELEVATOR_SPOOL_DIAMETER = Units.inchesToMeters(1.463);
        ELEVATOR_SETPOINT_TOLERANCE_METERS = 0.01;
        ELEVATOR_HOMING_TRESHOLD_SEC = 0.25;
        ELEVATOR_HOMING_TRESHOLD_MPS = 0.01;
        ELEVATOR_GEAR_RATIO = 1.0 / ((9.0 / 70.0) * (20.0 / 32.0) * (30.0 / 36.0));
        ELEVATOR_MAX_SPEED =
            ((KRAKEN_FREE_SPEED / 60.0) * (1.0 / ELEVATOR_GEAR_RATIO))
                * Math.PI
                * ELEVATOR_SPOOL_DIAMETER;
        ELEVATOR_LEFT_INVERSION = InvertedValue.CounterClockwise_Positive;
        ELEVATOR_MIN_HEIGHT = 0.0;
        ELEVATOR_END_OF_DANGER_ZONE = 0.34461142807517386; // Top gun reference?!?!?!
        ELEVATOR_MAX_HEIGHT = 0.6342116721107851;
      }
    }
  }

  /* Constants pertaining to the pivot */
  public static class Pivot {

    /* IDs and Offsets */
    public static final int PIVOT_ID;
    public static final int PIVOT_AZIMUTH_ID;
    public static final double PIVOT_MAGNET_OFFSET;

    public static final Rotation2d PIVOT_MAX_ANGLE;
    public static final Rotation2d
        PIVOT_MIN_SAFE_ANGLE; // This would be the minimum rotation at the bottom of the elevator's
    // travel
    public static final Rotation2d
        PIVOT_MIN_ANGLE; // This would be the minumum rotation at any point in the elevator's
    // "safe range"

    /* Pivot setpoint angles */
    public static final Rotation2d PIVOT_NEUTRAL_ANGLE;

    public static final Rotation2d PIVOT_IDLE_ANGLE;

    public static final Rotation2d PIVOT_INTAKE_ANGLE;

    public static final Rotation2d PIVOT_SPIT_ANGLE;

    public static final Rotation2d PIVOT_AMP_ANGLE;
    public static final Rotation2d PIVOT_PRE_CLIMB_ANGLE;
    public static final Rotation2d PIVOT_HALF_CLIMB_ANGLE;

    public static final Rotation2d PIVOT_CLIMBED_ANGLE;

    public static final Rotation2d PIVOT_TRAP_ANGLE;

    public static final Rotation2d PIVOT_FENDER_ANGLE;

    public static final Rotation2d PIVOT_PASS_ANGLE;

    public static final Rotation2d PIVOT_LOW_PASS_ANGLE;

    /* Physical Measurements */
    public static final double PIVOT_SETPOINT_TOLERANCE_RADS;
    public static final double PIVOT_DELTA_ERROR_TOLERANCE;
    public static final double PIVOT_DELTA_ERROR_TOLERANCE_SOD;
    public static final InvertedValue PIVOT_INVERSION;
    public static final SensorDirectionValue PIVOT_ENCODER_INVERSION;
    public static final double PIVOT_GEAR_RATIO;
    public static final double PIVOT_MAX_SPEED;

    static {
      if (robot == RobotType.BETA) {
        PIVOT_ID = 4;
        PIVOT_AZIMUTH_ID = 7;
        PIVOT_MAGNET_OFFSET = 0.307128;
        // PIVOT_MAGNET_OFFSET = 0.153809;

        PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(30.0);
        PIVOT_MIN_SAFE_ANGLE =
            Rotation2d.fromDegrees(
                -35); // This would be the minimum rotation at the bottom of the elevator's travel
        PIVOT_MIN_ANGLE =
            Rotation2d.fromDegrees(
                -54); // This would be the minumum rotation at any point in the elevator's
        // "safe range"

        PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(-27);

        PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(-27);

        PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(10.0);

        PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(24.0);

        PIVOT_PRE_CLIMB_ANGLE = Rotation2d.fromDegrees(-40);

        PIVOT_HALF_CLIMB_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(29);

        PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_FENDER_ANGLE = Rotation2d.fromDegrees(-54.0); // -50.0

        PIVOT_PASS_ANGLE = Rotation2d.fromDegrees(-50.0);

        PIVOT_LOW_PASS_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_SETPOINT_TOLERANCE_RADS = Units.degreesToRadians(1.5);
        PIVOT_DELTA_ERROR_TOLERANCE = 0.07;
        PIVOT_DELTA_ERROR_TOLERANCE_SOD = 0.01;
        PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
        PIVOT_ENCODER_INVERSION = SensorDirectionValue.Clockwise_Positive;
        PIVOT_GEAR_RATIO = 114.583;
        PIVOT_MAX_SPEED = ((FALCON_FREE_SPEED / 60.0) * (1.0 / PIVOT_GEAR_RATIO));
      } else {
        PIVOT_ID = 4;
        PIVOT_AZIMUTH_ID = 7;
        PIVOT_MAGNET_OFFSET = 0.2985839;

        PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(33);
        PIVOT_MIN_SAFE_ANGLE =
            Rotation2d.fromDegrees(
                -12.8); // This would be the minimum rotation at the bottom of the elevator's travel
        PIVOT_MIN_ANGLE =
            Rotation2d.fromDegrees(
                -56.42578125); // This would be the minumum rotation at any point in the elevator's
        // "safe range"

        PIVOT_NEUTRAL_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_IDLE_ANGLE = Rotation2d.fromDegrees(-20.0);

        PIVOT_INTAKE_ANGLE = Rotation2d.fromDegrees(-20.0);

        PIVOT_SPIT_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(25.0);

        PIVOT_PRE_CLIMB_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_HALF_CLIMB_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_CLIMBED_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(5.0);

        PIVOT_FENDER_ANGLE = Rotation2d.fromDegrees(-50.0);

        PIVOT_PASS_ANGLE = Rotation2d.fromDegrees(-50.0);

        PIVOT_LOW_PASS_ANGLE = Rotation2d.fromDegrees(0.0);

        PIVOT_SETPOINT_TOLERANCE_RADS = Units.degreesToRadians(3.0);
        PIVOT_DELTA_ERROR_TOLERANCE = 0.002;
        PIVOT_DELTA_ERROR_TOLERANCE_SOD = 0.002;
        PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
        PIVOT_ENCODER_INVERSION = SensorDirectionValue.Clockwise_Positive;
        PIVOT_GEAR_RATIO = (25.0 / 1.0) * (48.0 / 30.0) * (85.0 / 30.0);
        PIVOT_MAX_SPEED = ((FALCON_FREE_SPEED / 60.0) * (1.0 / PIVOT_GEAR_RATIO));
      }
    }
  }

  /* Constants pertaining to the shooter */
  public static class Shooter {

    /* IDs */
    public static final int SHOOTER_LEFT_ID;
    public static final int SHOOTER_RIGHT_ID;

    /* Setpoints and Tolerances */
    public static final double SHOOTER_SETPOINT_TOLERANCE_RPM;

    public static final double SHOOTER_LEFT_IDLE_RPM;
    public static final double SHOOTER_RIGHT_IDLE_RPM;

    public static final double SHOOTER_LEFT_AMP_RPM;
    public static final double SHOOTER_RIGHT_AMP_RPM;

    public static final double SHOOTER_LEFT_TRAP_RPM;
    public static final double SHOOTER_RIGHT_TRAP_RPM;

    public static final double SHOOTER_LEFT_FENDER_RPM;
    public static final double SHOOTER_RIGHT_FENDER_RPM;

    /* Physical Measurements */
    public static final double SHOOTER_LEFT_GEAR_RATIO;
    public static final double SHOOTER_RIGHT_GEAR_RATIO;
    public static final double SHOOTER_MAX_VELOCITY;
    public static final InvertedValue SHOOTER_RIGHT_INVERSION;
    public static final InvertedValue SHOOTER_LEFT_INVERSION;

    public static final double SHOOTER_LEFT_PASS_RPM;
    public static final double SHOOTER_RIGHT_PASS_RPM;

    public static final double SHOOTER_LEFT_LOW_PASS_RPM;
    public static final double SHOOTER_RIGHT_LOW_PASS_RPM;

    static {
      if (robot == RobotType.BETA) {
        SHOOTER_LEFT_ID = 3;
        SHOOTER_RIGHT_ID = 2;

        SHOOTER_SETPOINT_TOLERANCE_RPM = 100.0;

        SHOOTER_LEFT_IDLE_RPM = 0.0;
        SHOOTER_RIGHT_IDLE_RPM = 0.0;

        SHOOTER_LEFT_AMP_RPM = 950.0;
        SHOOTER_RIGHT_AMP_RPM = 950.0;

        SHOOTER_LEFT_TRAP_RPM = 0.0;
        SHOOTER_RIGHT_TRAP_RPM = 0.0;

        SHOOTER_LEFT_FENDER_RPM = 1400.0; // 1800
        SHOOTER_RIGHT_FENDER_RPM = 2000.0; // 1800

        SHOOTER_LEFT_PASS_RPM = 1750.0;
        SHOOTER_RIGHT_PASS_RPM = 1400.0;

        SHOOTER_LEFT_GEAR_RATIO = 1.0;
        SHOOTER_RIGHT_GEAR_RATIO = 1.0;
        SHOOTER_MAX_VELOCITY = (6380.0 / 60.0);
        SHOOTER_RIGHT_INVERSION = InvertedValue.CounterClockwise_Positive;
        SHOOTER_LEFT_INVERSION = InvertedValue.Clockwise_Positive;

        SHOOTER_LEFT_LOW_PASS_RPM = 2500;
        SHOOTER_RIGHT_LOW_PASS_RPM = 1500;
      } else {
        SHOOTER_LEFT_ID = 3;
        SHOOTER_RIGHT_ID = 2;

        SHOOTER_SETPOINT_TOLERANCE_RPM = 200.0;

        SHOOTER_LEFT_IDLE_RPM = 0.0;
        SHOOTER_RIGHT_IDLE_RPM = 0.0;

        SHOOTER_LEFT_AMP_RPM = 1000.0;
        SHOOTER_RIGHT_AMP_RPM = 1000.0;

        SHOOTER_LEFT_TRAP_RPM = 0.0;
        SHOOTER_RIGHT_TRAP_RPM = 0.0;

        SHOOTER_LEFT_FENDER_RPM = 3000.0; // 1800
        SHOOTER_RIGHT_FENDER_RPM = 1500.0; // 1800

        SHOOTER_LEFT_PASS_RPM = 800.0;
        SHOOTER_RIGHT_PASS_RPM = 400.0;

        SHOOTER_LEFT_GEAR_RATIO = 1.0;
        SHOOTER_RIGHT_GEAR_RATIO = 1.0;
        SHOOTER_MAX_VELOCITY = (6380.0 / 60.0);
        SHOOTER_RIGHT_INVERSION = InvertedValue.CounterClockwise_Positive;
        SHOOTER_LEFT_INVERSION = InvertedValue.Clockwise_Positive;

        SHOOTER_LEFT_LOW_PASS_RPM = 2500;
        SHOOTER_RIGHT_LOW_PASS_RPM = 1500;
      }
    }
  }

  /* Constants pertaining to the feeder */
  public static class Feeder {

    /* IDs */
    public static final int FEEDER_ID;
    public static final int BEAMBREAK_ID;

    /* Setpoints and tolerances */
    public static final double FEEDER_INTAKE_SPEED;

    public static final double FEEDER_SPIT_SPEED;

    public static final double FEEDER_SHOOT_SPEED;

    /* Physical Measurements */
    public static final double FEEDER_GEAR_RATIO;
    public static final double FEEDER_ROLLER_DIAMETER;
    public static final InvertedValue FEEDER_INVERSION;

    static {
      if (robot == RobotType.BETA) {
        FEEDER_ID = 25;
        BEAMBREAK_ID = 0;

        FEEDER_INTAKE_SPEED = 65; // 65

        FEEDER_SPIT_SPEED = -1.0;

        FEEDER_SHOOT_SPEED = 0.8;

        FEEDER_GEAR_RATIO = 1.0;
        FEEDER_ROLLER_DIAMETER = 1.0;
        FEEDER_INVERSION = InvertedValue.Clockwise_Positive;
      } else {
        FEEDER_ID = 25;
        BEAMBREAK_ID = 0;

        FEEDER_INTAKE_SPEED = 0.75;

        FEEDER_SPIT_SPEED = -0.1;

        FEEDER_SHOOT_SPEED = 0.3;

        FEEDER_GEAR_RATIO = 1.0;
        FEEDER_ROLLER_DIAMETER = 1.0;
        FEEDER_INVERSION = InvertedValue.CounterClockwise_Positive;
      }
    }
  }

  /* Constants pertaining to the shots taken in auto */
  public static class AutoShots {

    // Five Note Amp Side Shots
    public static final ShotParameter FIVE_NOTE_AMP_FIRST_SHOT =
        new ShotParameter(-32.5, 2500, 1200, 0.3);
    public static final ShotParameter FIVE_NOTE_AMP_SECOND_SHOT =
        new ShotParameter(-33.5, 2500, 1200, 0.3);
    public static final ShotParameter FIVE_NOTE_AMP_THIRD_SHOT =
        new ShotParameter(-33.5, 1200, 2500, 0.3);
    public static final ShotParameter FIVE_NOTE_AMP_FOURTH_SHOT =
        new ShotParameter(-20.0, 1500, 3000, 0.2);
  }
}
