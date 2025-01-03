package frc.robot.subsystems.feeder;

import static frc.robot.constants.Constants.Feeder.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.constants.Constants;

public class FeederIOFalcon500 implements FeederIO {

  /* Hardware */
  private final TalonFX motor = new TalonFX(FEEDER_ID);
  private final DigitalInput beamBreak = new DigitalInput(Constants.Feeder.BEAMBREAK_ID);

  /* Configurator */
  private final TalonFXConfigurator configurator;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitConfigs;
  private final Slot0Configs slot0Configs;
  private final MotorOutputConfigs motorOutputConfigs;
  private final HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs;
  private final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs;

  private StatusSignal<Double> position;
  private StatusSignal<Double> velocity;
  private StatusSignal<Double> current;
  private StatusSignal<Double> temperature;
  private StatusSignal<Double> voltage;
  private Debouncer beamBreakDebounce = new Debouncer(0.25, DebounceType.kFalling);

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Feeder/kS", 0.0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Feeder/kV", 0.12);
  LoggedTunableNumber kP = new LoggedTunableNumber("Feeder/kP", 0.3);
  LoggedTunableNumber kI = new LoggedTunableNumber("Feeder/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Feeder/kD", 0.0);

  private double setpoint;

  public FeederIOFalcon500() {
    /* Instantiate configurator */
    configurator = motor.getConfigurator();

    /* Create configs */
    currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.SupplyCurrentLimit = 30.0;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = FEEDER_INVERSION;
    motorOutputConfigs.PeakForwardDutyCycle = 1.0;
    motorOutputConfigs.PeakReverseDutyCycle = -1.0;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;

    softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = false;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = false;

    /* Set status signals */
    position = motor.getPosition();
    velocity = motor.getVelocity();
    current = motor.getSupplyCurrent();
    temperature = motor.getDeviceTemp();
    voltage = motor.getMotorVoltage();

    /* Apply configs */
    configurator.apply(currentLimitConfigs);
    configurator.apply(motorOutputConfigs);
    configurator.apply(slot0Configs);
    configurator.apply(hardwareLimitSwitchConfigs);
    configurator.apply(softwareLimitSwitchConfigs);
    configurator.apply(new VoltageConfigs());

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, velocity, current, temperature, voltage);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, current, temperature, voltage);

    inputs.posMeters = position.getValue() * Math.PI * FEEDER_ROLLER_DIAMETER / FEEDER_GEAR_RATIO;
    inputs.velocityMps = velocity.getValue() * Math.PI * FEEDER_ROLLER_DIAMETER / FEEDER_GEAR_RATIO;
    inputs.appliedVolts = voltage.getValue();
    inputs.tempCelcius = temperature.getValue();
    inputs.currentAmps = current.getValue();
    inputs.beamBreakTriggered = beamBreakDebounce.calculate(!beamBreak.get());
    inputs.rawBeamBreakTriggered = !beamBreak.get();
    inputs.setpoint = setpoint;
  }

  @Override
  public void setPercent(double percent) {
    motor.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setVelocity(double velocityMps) {
    setpoint = velocityMps;
    if (velocityMps > 0.0) {
      motor.setControl(
          new VelocityVoltage(
              (velocityMps * FEEDER_GEAR_RATIO) / (Math.PI * FEEDER_ROLLER_DIAMETER)));
    } else {
      motor.setControl(new DutyCycleOut(0.0));
    }
  }

  @Override
  public void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = currentLimit;

    configurator.apply(currentLimitConfigs);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    configurator.apply(motorOutputConfigs);
  }

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {

      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      configurator.apply(slot0Configs);
    }
  }
}
