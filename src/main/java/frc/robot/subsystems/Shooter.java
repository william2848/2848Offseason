package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.RangerHelpers.*;

import java.util.regex.Matcher;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import lombok.Getter;
import lombok.Setter;

/** Shooter Subsystem. */
public class Shooter extends SubsystemBase {
  private final TalonFX m_FlywheelLeftLeader;
  private final TalonFX m_FlywheelRightFollower;


  @Getter @Setter private boolean shooting = false;
  private boolean shootingDebounced = false;

  @Getter private boolean reversing = false;
  
  private final TalonFX m_Turret;
  private final CANcoder m_TurretCANcoder;



  private final CANrange m_MagazineSensor = new CANrange(kCANRangeID);
  private final CANrangeConfiguration magazineSensorConfig = new CANrangeConfiguration();

  private final Debouncer shootingDebouncer = new Debouncer(3, DebounceType.kRising);
  private final Debouncer magazineSensorDebouncer = new Debouncer(3, DebounceType.kFalling);
  private final Debouncer fuelPerSecondDebouncer = new Debouncer(.01);
  // private final Debouncer currentDebouncer = new Debouncer(0.167, DebounceType.kBoth);

  private final TalonFX m_Magazine;

  private double targetDist =0;

  private double turretSetpoint = 0;

  private double targetTheta = 0;

  private double distanceTrim = 0;
  private double angularTrim = 0;

  private boolean isShootingByCANrange = false;
  // private boolean isShootingByCurrent = false;  
  
  private enum flywheelStates {
    IDLE,
    SHOOT
  };

  private flywheelStates currentState = flywheelStates.IDLE;
  // create a Motion Magic request, voltage output


  //private final PositionVoltage m_HoodVoltage = new PositionVoltage(0).withSlot(0);

  //private @Getter double m_FlywheelOutputDutyCycle = 0;
  private @Getter long m_TurretAngle = 0; // Use Radians, 0 is from the front of the robot

  private final VelocityVoltage flyWheelVelocityVoltage = new VelocityVoltage(10);
  private final VelocityVoltage magazineVelocityVoltage = new VelocityVoltage(0);
  private final PositionVoltage turretOut = new PositionVoltage(0);

  /** Shooter Subsystem. */
  public Shooter() {
    m_FlywheelLeftLeader = new TalonFX(kFlywheelLeftMotorID);
    m_FlywheelRightFollower = new TalonFX(kFlywheelRightMotorID);
    m_Turret = new TalonFX(kTurretMotorID);
    m_TurretCANcoder = new CANcoder(kTurretCANcoderID);
    m_Magazine = new TalonFX(kMagazineMotorID);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kS = kFlywheelkS;
    flywheelConfig.Slot0.kV = kFlywheelkV;
    flywheelConfig.Slot0.kP = kFlywheelkP;//.000175;  
    flywheelConfig.Slot0.kI = kFlywheelkI;
    flywheelConfig.Slot0.kD = kFlywheelkD;
    flywheelConfig.Slot0.kA = kFlywheelkA;
    flywheelConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kFlywheelPeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kFlywheelPeakVoltage));
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    setupTalonFx(m_FlywheelLeftLeader, flywheelConfig);
    m_FlywheelRightFollower.setControl(new Follower(m_FlywheelLeftLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.kS = kTurretkS;
    turretConfig.Slot0.kV = kTurretkV;
    turretConfig.Slot0.kA = kTurretkA;
    turretConfig.Slot0.kP = kTurretkP;
    turretConfig.Slot0.kI = kTurretkI;
    turretConfig.Slot0.kD = kTurretkD;
    turretConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kTurretPeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kTurretPeakVoltage));
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = kTurretCANcoderID;
    //turretConfig.Feedback.SensorToMechanismRatio =-1;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    var motionMagicConfigs = turretConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kTurretMMCruiseVelocity; 
    motionMagicConfigs.MotionMagicAcceleration = kTurretMMAcceleration; 
    motionMagicConfigs.MotionMagicJerk = kTurretMMJerk;
    //CANcoderConfigurator turretCANcoderConfigurator = m_TurretCANcoder.getConfigurator();
    m_TurretCANcoder.getConfigurator().apply(kTurretCANcoderMagnetSensorConfigs);
    //retryConfigApply(() -> turretCANcoderConfigurator.apply(kTurretCANcoderMagnetSensorConfigs)); 
    setupTalonFx(m_Turret, turretConfig);

    SoftwareLimitSwitchConfigs turretSwitchConfigs = new SoftwareLimitSwitchConfigs();
    turretSwitchConfigs.ForwardSoftLimitThreshold = kTurretSwitchForwardLimit;
    turretSwitchConfigs.ReverseSoftLimitThreshold = kTurretSwitchReverseLimit;
    turretSwitchConfigs.ForwardSoftLimitEnable = true;
    turretSwitchConfigs.ReverseSoftLimitEnable = true;
    m_Turret.getConfigurator().apply(turretSwitchConfigs);
    
    m_Turret.getConfigurator().apply(motionMagicConfigs);

    m_TurretCANcoder.getConfigurator().apply(kTurretCANcoderMagnetSensorConfigs);
    

    TalonFXConfiguration magazineConfig = new TalonFXConfiguration();
    magazineConfig.Slot0.kS = kMagazinekS;
    magazineConfig.Slot0.kV = kMagazinekV;
    magazineConfig.Slot0.kA = kMagazinekA;
    magazineConfig.Slot0.kP = kMagazinekP;  
    magazineConfig.Slot0.kI = kMagazinekI;
    magazineConfig.Slot0.kD = kMagazinekD;
    magazineConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kMagazinePeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kMagazinePeakVoltage));
    magazineConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    retryConfigApply(() -> m_Magazine.getConfigurator().apply(magazineConfig));

    magazineSensorConfig.ProximityParams.ProximityThreshold =.13;
    magazineSensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
    magazineSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    m_MagazineSensor.getConfigurator().apply(magazineSensorConfig);
  }
public double getTurretAngle(){
    
    double rawAngle = m_Turret.getPosition().getValueAsDouble() * 2 * Math.PI;
    double shiftedAngle = rawAngle + Math.PI/2;
    double wrapped = MathUtil.angleModulus(shiftedAngle);

    return wrapped;
}
}