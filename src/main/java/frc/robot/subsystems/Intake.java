package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import lombok.Getter;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.RangerHelpers.*;

public class Intake extends SubsystemBase {
    private final TalonFX m_RollersL = new TalonFX(kLRollersMotorID);
    private final TalonFXConfiguration lRollersMotorConfig = new TalonFXConfiguration();
    

    private final TalonFX m_RollersR = new TalonFX(kRRollersMotorID);
    private final TalonFXConfiguration rRollersMotorConfig = new TalonFXConfiguration();

    private final TalonFX m_RollersR2 = new TalonFX(kR2RollersMotorID);
    private final TalonFXConfiguration r2RollersMotorConfig = new TalonFXConfiguration();

    private final TalonFX m_Pivot = new TalonFX(kIntakePivotID);
    private final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

    private final CANcoder m_IntakeCANcoder = new CANcoder(kIntakePivotCANcoderID);

    private double pivotSetpoint = kStowSetpoint;

    @Getter private boolean intakeDown = false;

    // Pivot motor would use PID, maybe feedforward with arm angle

    public Intake() {

        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotMotorConfig.Slot0.kS = kPivotkS;
        pivotMotorConfig.Slot0.kV = kPivotkV;
        pivotMotorConfig.Slot0.kA = kPivotkA;
        pivotMotorConfig.Slot0.kP = kPivotkP;
        pivotMotorConfig.Slot0.kI = kPivotkI;
        pivotMotorConfig.Slot0.kD = kPivotkD;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = kPivotMMCruiseVelocity;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = kPivotMMAcceleration;
        pivotMotorConfig.MotionMagic.MotionMagicJerk = kPivotMMJerk;

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = kIntakePivotCANcoderID;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        //pivotMotorConfig.Feedback.SensorToMechanismRatio =-1;
        
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =-0.5;
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =0;
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        CANcoderConfigurator intakeCANcoderConfigurator = m_IntakeCANcoder.getConfigurator();
        retryConfigApply(() -> intakeCANcoderConfigurator.apply(kIntakeCANcoderMagnetSensorConfigs));


        // Apply things to the configuration here
        lRollersMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        lRollersMotorConfig.CurrentLimits.StatorCurrentLimit = 25;
        lRollersMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rRollersMotorConfig.CurrentLimits.StatorCurrentLimit = 25;
        rRollersMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        r2RollersMotorConfig.CurrentLimits.StatorCurrentLimit = 25;
        r2RollersMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        setupTalonFx(m_RollersL, lRollersMotorConfig);
        setupTalonFx(m_RollersR, rRollersMotorConfig);
        setupTalonFx(m_RollersR2, r2RollersMotorConfig);
        setupTalonFx(m_Pivot, pivotMotorConfig);
    
        m_RollersR.setControl(new Follower(m_RollersL.getDeviceID(), MotorAlignmentValue.Opposed));
        m_RollersR2.setControl(new Follower(m_RollersL.getDeviceID(),MotorAlignmentValue.Opposed));

        m_IntakeCANcoder.setPosition(m_IntakeCANcoder.getAbsolutePosition().getValue());
    } 

    @Override
    public void periodic() {
        SmartDashboard.putData(this);

        // setControl needs to run periodically at all times or the motor will disable I think        
        
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

    }

}
