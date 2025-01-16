package frc.robot.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.*; 

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    //TalonFX
        public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration coralFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();
        //remove below
        public final TalonFXConfiguration shooterFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration aimerFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration armFXConfig = new TalonFXConfiguration();
    //TalonFXS
        public final TalonFXSConfiguration intakeFXSConfig = new TalonFXSConfiguration();
    //CANcoder
        public final CANcoderConfiguration coralCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration climberCCConfig = new CANcoderConfiguration();
        //remove below
        public final CANcoderConfiguration aimerCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration armCCConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        //Intake Configuration
        //CANcoder
        coralCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        coralCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        coralCCConfig.MagnetSensor.MagnetOffset = CoralConstants.kMagnetOffset;

        Slot0Configs coralSlot0Configs = new Slot0Configs()
            .withKP(CoralConstants.kP)
            .withKI(CoralConstants.kI)
            .withKD(CoralConstants.kD)
            .withKS(CoralConstants.kS)
            .withKV(CoralConstants.kV)
            .withKA(CoralConstants.kA);
        coralFXConfig.Slot0 = coralSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs coralCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CoralConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(CoralConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(CoralConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(CoralConstants.kCurrentLimitEnable);
        coralFXConfig.CurrentLimits = coralCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs coralMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(CoralConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(CoralConstants.kMotionMagicAccel)
            .withMotionMagicJerk(CoralConstants.kMotionMagicJerk);
        coralFXConfig.MotionMagic = coralMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs coralSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(CoralConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(CoralConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(CoralConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(CoralConstants.kSoftForwardLimit);
        coralFXConfig.SoftwareLimitSwitch = coralSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs coralHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // coralFXConfig.HardwareLimitSwitch = coralHardwareLimitsConfigs;
        //Encoder
        if(CoralConstants.kUseCANcoder) {
            coralFXConfig.Feedback.FeedbackRemoteSensorID = CoralConstants.kCANcoderID;
            coralFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            coralFXConfig.Feedback.RotorToSensorRatio = CoralConstants.kGearRatio;
            coralFXConfig.Feedback.SensorToMechanismRatio = CoralConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            coralFXConfig.Feedback.SensorToMechanismRatio = CoralConstants.kGearRatio;
        }
        //Neutral and Direction
        coralFXConfig.MotorOutput.NeutralMode = CoralConstants.kNeutralMode;
        coralFXConfig.MotorOutput.Inverted = (CoralConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        coralFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);


        //Shooter Configuration
        Slot0Configs shooterSlot0Configs = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD)
            .withKV(ShooterConstants.kV);
        shooterFXConfig.Slot0 = shooterSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLimit(ShooterConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ShooterConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ShooterConstants.kCurrentLimitEnable);
        shooterFXConfig.CurrentLimits = shooterCurrentLimitsConfigs;
        // shooterFXConfig.Voltage.PeakForwardVoltage = Shooter.kPeakFwdVoltage;
        // shooterFXConfig.Voltage.PeakReverseVoltage = Shooter.kPeakRevVoltage;
        //Ramping (spinup/spindown)
        // shooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Shooter.kOpenLoopRamp;
        // shooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Shooter.kClosedLoopRamp;
        //Motion Magic
        MotionMagicConfigs shooterMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ShooterConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(ShooterConstants.kMotionMagicAccel)
            .withMotionMagicJerk(ShooterConstants.kMotionMagicJerk);
        shooterFXConfig.MotionMagic = shooterMotionMagicConfigs;
        //Neutral and Direction
        shooterFXConfig.MotorOutput.NeutralMode = ShooterConstants.kNeutralMode;
        shooterFXConfig.MotorOutput.Inverted = (ShooterConstants.isInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Rotational rate modifiers
        shooterFXConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kGearRatio;
        //Audio
        shooterFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Aimer Configuration
        //CANcoder
        aimerCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        aimerCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        aimerCCConfig.MagnetSensor.MagnetOffset = AimerConstants.kMagnetOffset;
        
        Slot0Configs aimerSlot0Configs = new Slot0Configs()
            .withKP(AimerConstants.kP)
            .withKI(AimerConstants.kI)
            .withKD(AimerConstants.kD)
            .withKS(AimerConstants.kS)
            .withKV(AimerConstants.kV)
            .withKA(AimerConstants.kA);
        aimerFXConfig.Slot0 = aimerSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs aimerCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(AimerConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(AimerConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(AimerConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(AimerConstants.kCurrentLimitEnable);
        aimerFXConfig.CurrentLimits = aimerCurrentLimitsConfigs;
        // aimerFXConfig.Voltage.PeakForwardVoltage = Aimer.kPeakFwdVoltage;
        // aimerFXConfig.Voltage.PeakReverseVoltage = Aimer.kPeakRevVoltage;
        //Ramping (spinup/spindown) //not needed or wanted for MM
        // aimerFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Aimer.kOpenLoopRamp;
        // aimerFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Aimer.kClosedLoopRamp;
        //Motion Magic
        MotionMagicConfigs aimerMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(AimerConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(AimerConstants.kMotionMagicAccel)
            .withMotionMagicJerk(AimerConstants.kMotionMagicJerk);
        aimerFXConfig.MotionMagic = aimerMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs aimerSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(AimerConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(AimerConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(AimerConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(AimerConstants.kSoftForwardLimit);
        aimerFXConfig.SoftwareLimitSwitch = aimerSoftwareLimitSwitchConfigs;
         
        // HardwareLimitSwitchConfigs aimerHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // aimerFXConfig.HardwareLimitSwitch = aimerHardwareLimitsConfigs;
        //Encoder
        if(AimerConstants.kUseCANcoder) {
            aimerFXConfig.Feedback.FeedbackRemoteSensorID = AimerConstants.kCANcoderID;
            aimerFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
            aimerFXConfig.Feedback.RotorToSensorRatio = AimerConstants.kGearRatio;
            aimerFXConfig.Feedback.SensorToMechanismRatio = 1.0; //CANcoder is the same as mechanism
        } else {
            aimerFXConfig.Feedback.SensorToMechanismRatio = AimerConstants.kGearRatio;
        }
        //Neutral and Direction
        aimerFXConfig.MotorOutput.NeutralMode = AimerConstants.kNeutralMode;
        aimerFXConfig.MotorOutput.Inverted = (AimerConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        aimerFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Arm
        //CANcoder
        armCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        armCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armCCConfig.MagnetSensor.MagnetOffset = ArmConstants.kMagnetOffset;

        Slot0Configs armSlot0Configs = new Slot0Configs()
            .withKP(ArmConstants.kP)
            .withKI(ArmConstants.kI)
            .withKD(ArmConstants.kD)
            .withKS(ArmConstants.kS)
            .withKV(ArmConstants.kV)
            .withKA(ArmConstants.kA);
        armFXConfig.Slot0 = armSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs armCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ArmConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ArmConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ArmConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ArmConstants.kCurrentLimitEnable);
        armFXConfig.CurrentLimits = armCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ArmConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(ArmConstants.kMotionMagicAccel)
            .withMotionMagicJerk(ArmConstants.kMotionMagicJerk);
        armFXConfig.MotionMagic = armMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs armSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(ArmConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(ArmConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(ArmConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(ArmConstants.kSoftForwardLimit);
        armFXConfig.SoftwareLimitSwitch = armSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs armHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // armFXConfig.HardwareLimitSwitch = armHardwareLimitsConfigs;
        //Encoder
        if(ArmConstants.kUseCANcoder) {
            armFXConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.kCANcoderID;
            armFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
            armFXConfig.Feedback.RotorToSensorRatio = ArmConstants.kGearRatio;
            armFXConfig.Feedback.SensorToMechanismRatio = 1.0; //CANcoder is the same as mechanism
        } else {
            armFXConfig.Feedback.SensorToMechanismRatio = ArmConstants.kGearRatio;
        }
        //Neutral and Direction
        armFXConfig.MotorOutput.NeutralMode = ArmConstants.kNeutralMode;
        armFXConfig.MotorOutput.Inverted = (ArmConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        armFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);


        //Climber
        //CANcoder
        climberCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        climberCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        climberCCConfig.MagnetSensor.MagnetOffset = ClimberConstants.kMagnetOffset;

        Slot0Configs climberSlot0Configs = new Slot0Configs()
            .withKP(ClimberConstants.kP)
            .withKI(ClimberConstants.kI)
            .withKD(ClimberConstants.kD)
            .withKS(ClimberConstants.kS)
            .withKV(ClimberConstants.kV)
            .withKA(ClimberConstants.kA);
        climberFXConfig.Slot0 = climberSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs climberCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ClimberConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ClimberConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ClimberConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ClimberConstants.kCurrentLimitEnable);
        climberFXConfig.CurrentLimits = climberCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ClimberConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(ClimberConstants.kMotionMagicAccel)
            .withMotionMagicJerk(ClimberConstants.kMotionMagicJerk);
        climberFXConfig.MotionMagic = climberMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs climberSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(ClimberConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(ClimberConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(ClimberConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(ClimberConstants.kSoftForwardLimit);
        climberFXConfig.SoftwareLimitSwitch = climberSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs climberHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // climberFXConfig.HardwareLimitSwitch = climberHardwareLimitsConfigs;
        //Encoder
        if(ClimberConstants.kUseCANcoder) {
            climberFXConfig.Feedback.FeedbackRemoteSensorID = ClimberConstants.kCANcoderID;
            climberFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            climberFXConfig.Feedback.RotorToSensorRatio = ClimberConstants.kGearRatio;
            climberFXConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            climberFXConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kGearRatio;
        }
        //Neutral and Direction
        climberFXConfig.MotorOutput.NeutralMode = ClimberConstants.kNeutralMode;
        climberFXConfig.MotorOutput.Inverted = (ClimberConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        climberFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

    }

    public void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = 5;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }
}