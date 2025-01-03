package frc.robot.subsystems.elevatorpivot;

import static frc.robot.constants.Constants.Pivot.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.AveragingFilter;
import frc.robot.commons.LoggedTunableNumber;

public class PivotIOKrakenX60 implements PivotIO {

  /* Hardware */
  private final TalonFX pivot = new TalonFX(PIVOT_ID);
  private final CANcoder azimuth = new CANcoder(PIVOT_AZIMUTH_ID);

  /* Configurators */
  private TalonFXConfigurator pivotConfigurator;
  private CANcoderConfigurator azimuthConfigurator;

  private final CurrentLimitsConfigs currentLimitConfigs;
  private final MotorOutputConfigs motorOutputConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs pivotMotionMagicConfigs;

  /* Status Signals */
  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> appliedVoltage;
  private final StatusSignal<Double> motionMagicPositionTarget;
  private final StatusSignal<Double> motionMagicVelocityTarget;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", 0.3); // 0.3
  LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/kG", 0.0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", 12); // 12
  LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", 200); // 200
  LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", 9); // 9

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Pivot/MotionAcceleration", 3); // 5
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Pivot/MotionCruiseVelocity", 5); // 5

  LoggedTunableNumber kE = new LoggedTunableNumber("Pivot/kE", 0.0);

  StatusCode pivotStatusCode;

  double setpointDeg = 0.0;

  private AveragingFilter deltaErrorFilter = new AveragingFilter(15);
  private double prevError = 0.0;

  public PivotIOKrakenX60() {
    /* Configurators */
    pivotConfigurator = pivot.getConfigurator();
    azimuthConfigurator = azimuth.getConfigurator();

    /* Configure pivot hardware */
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = 100.0;
    currentLimitConfigs.SupplyCurrentLimit = 40.0;

    motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = PIVOT_INVERSION;
    motorOutputConfigs.PeakForwardDutyCycle = 1.0;
    motorOutputConfigs.PeakReverseDutyCycle = -1.0;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS.get();
    slot0Configs.kG = kG.get();
    slot0Configs.kV = kV.get();
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    MagnetSensorConfigs pivotMagnetSensorConfigs = new MagnetSensorConfigs();
    pivotMagnetSensorConfigs.SensorDirection = PIVOT_ENCODER_INVERSION;
    pivotMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    pivotMagnetSensorConfigs.MagnetOffset = PIVOT_MAGNET_OFFSET;

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotFeedbackConfigs.FeedbackRemoteSensorID = azimuth.getDeviceID();
    pivotFeedbackConfigs.RotorToSensorRatio = PIVOT_GEAR_RATIO;
    pivotFeedbackConfigs.SensorToMechanismRatio = 1.0;

    pivotMotionMagicConfigs = new MotionMagicConfigs();
    pivotMotionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    position = azimuth.getAbsolutePosition().clone();
    velocity = pivot.getVelocity().clone();
    supplyCurrent = pivot.getSupplyCurrent().clone();
    appliedVoltage = pivot.getMotorVoltage().clone();
    motionMagicPositionTarget = pivot.getClosedLoopReference().clone();
    motionMagicVelocityTarget = pivot.getClosedLoopReferenceSlope().clone();

    /* Apply Configurations */
    azimuthConfigurator.apply(pivotMagnetSensorConfigs);
    pivotConfigurator.apply(currentLimitConfigs);
    pivotConfigurator.apply(motorOutputConfigs);
    pivotConfigurator.apply(slot0Configs);
    pivotConfigurator.apply(pivotFeedbackConfigs);
    pivotConfigurator.apply(pivotMotionMagicConfigs);
    pivotConfigurator.apply(openLoopRampsConfigs);
    pivotConfigurator.apply(closedLoopRampsConfigs);
    pivotConfigurator.apply(new VoltageConfigs());

    pivotStatusCode =
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position,
            velocity,
            supplyCurrent,
            appliedVoltage,
            motionMagicPositionTarget,
            motionMagicVelocityTarget);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        supplyCurrent,
        appliedVoltage,
        motionMagicPositionTarget,
        motionMagicVelocityTarget);
    inputs.angleDegrees = Units.rotationsToDegrees(position.getValue());
    inputs.angleRads = Units.rotationsToRadians(position.getValue());
    inputs.velDegreesPerSecond = Units.rotationsToDegrees(velocity.getValue());
    inputs.currentAmps = pivot.getStatorCurrent().getValue();
    inputs.appliedVoltage = appliedVoltage.getValue();
    inputs.tempCelcius = pivot.getDeviceTemp().getValue();
    inputs.motionMagicPositionTargetDeg =
        Units.rotationsToDegrees(motionMagicPositionTarget.getValue());
    inputs.motionMagicVelocityTargetDeg =
        Units.rotationsToDegrees(motionMagicVelocityTarget.getValue());
    inputs.setpointDeg = setpointDeg;
    inputs.shaftPosition = Units.rotationsToDegrees(pivot.getPosition().getValue());

    // Delta error filter calculations
    double err = inputs.setpointDeg - inputs.angleDegrees;
    deltaErrorFilter.addSample(err - prevError);
    inputs.deltaError = deltaErrorFilter.getAverage();
    prevError = err;
  }

  @Override
  public void setVoltage(double voltage) {
    pivot.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setAngle(Rotation2d angle, double elevatorAcceleration) {
    // Elevator acceleration kick
    double elevatorKick = kE.get() * elevatorAcceleration;

    setpointDeg = angle.getDegrees();
    double setpoint =
        MathUtil.clamp(
            angle.getRotations(), PIVOT_MIN_ANGLE.getRotations(), PIVOT_MAX_ANGLE.getRotations());
    pivot.setControl(new MotionMagicVoltage(setpoint).withFeedForward(elevatorKick));
  }

  @Override
  public void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = currentLimit;

    pivotConfigurator.apply(currentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    pivotConfigurator.apply(motorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kG.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)
        || motionAcceleration.hasChanged(0)
        || motionCruiseVelocity.hasChanged(0)) {
      slot0Configs.kS = kS.get();
      slot0Configs.kG = kG.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      pivotMotionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      pivotMotionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      pivotConfigurator.apply(slot0Configs);
      pivotConfigurator.apply(pivotMotionMagicConfigs);
    }
  }
}
