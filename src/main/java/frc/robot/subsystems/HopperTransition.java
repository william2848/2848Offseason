package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import lombok.Getter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.RangerHelpers.setupTalonFx;

import java.util.function.BooleanSupplier;

public class HopperTransition extends SubsystemBase {
    private final TalonFX m_SidewaysBelt = new TalonFX(kSidewaysBeltMotorID);
    private final TalonFXConfiguration sidewaysBeltMotorConfig = new TalonFXConfiguration();
    
    private final TalonFX m_ForwardBelt = new TalonFX(kForwardBeltMotorID);
    private final TalonFXConfiguration forwardBeltMotorConfig = new TalonFXConfiguration();
    

    public HopperTransition() {
        // Apply things to the configurations here

        setupTalonFx(m_SidewaysBelt, sidewaysBeltMotorConfig);
        setupTalonFx(m_ForwardBelt, forwardBeltMotorConfig);

        this.register();

    }

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Example!", ()-> "This is what it looks like", null);
    }

}
