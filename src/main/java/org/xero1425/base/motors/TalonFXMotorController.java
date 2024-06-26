package org.xero1425.base.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/// \file
/// This file contains the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonFX,
/// the TalonSRX, and the VictorSPX.
///


/// \brief This class is MotorController class that supports the TalonFX motors.   This class is a
/// wrapper for the TalonFX class that provides the interface that meets the requirements of the
/// MotorController base class.
public class TalonFXMotorController extends MotorController
{
    private final static int kApplyTries = 5 ;
    private final static int kTicksPerRevolution = 2048 ;

    private TalonFX ctrl_ ;
    private TalonFXConfiguration cfg_ ;
    private IMotorController leader_ ;
    private String bus_ ;
    private boolean voltage_compensation_enabled_ ;
    private double nominal_voltage_ ;
    private int major_ ;
    private int minor_ ;
    private int bugfix_ ;
    private int build_ ;
    private String type_ ;

    public TalonFXMotorController(String name, String bus, int canid, boolean leader, String type) throws MotorRequestFailedException, BadMotorRequestException {
        super(name) ;

        bus_ = bus ;
        type_ = type ;

        ctrl_ = new TalonFX(canid, bus_);
        cfg_ = new TalonFXConfiguration() ;

        major_ = -1 ;
        minor_ = -1 ;
        bugfix_ = -1 ;
        build_ = -1 ;

        checkError("TalonFXMotorController - apply configuration", () -> ctrl_.getConfigurator().apply(cfg_));
        checkError("TalonFXMotorController - optimize bus", () -> ctrl_.optimizeBusUtilization()) ;
        setPosition(0.0);
    }

    private void checkError(String msg, Supplier<StatusCode> toApply) throws MotorRequestFailedException {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = kApplyTries ;
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            msg = msg + " - " + code.toString() ;
            throw new MotorRequestFailedException(this, msg, code) ;
        }
    }

    public void setPIDv(double v) throws MotorRequestFailedException {
        cfg_.Slot0.kV = v ;
        checkError("setPID()", () -> ctrl_.getConfigurator().apply(cfg_.Slot0));        
    }

    public void setPIDp(double v) throws MotorRequestFailedException {
        cfg_.Slot0.kP = v ;
        checkError("setPID()", () -> ctrl_.getConfigurator().apply(cfg_.Slot0));        
    }    

    /// \brief Returns the CAN ID of the motor
    /// \returns the CAN ID of the motor    
    public int getCanID() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getDeviceID() ;
    }

    /// \brief Returns the name of the CAN bus.  An empty string means roborio
    /// \return the name of the CAN bus.  An empty string means roborio    
    public String getBus() throws BadMotorRequestException, MotorRequestFailedException {
        return bus_ ;
    }

    /// \brief Return list of faults detected by the motor controller
    /// \returns list of faults detected by the motor controller
    public List<String> getFaults() {
        List<String> results = new ArrayList<String>() ;

        if (ctrl_.getFault_Hardware().getValue()) {
            results.add("hardware");
        }

        if (ctrl_.getStickyFault_Hardware().getValue()) {
            results.add("sticky-hardware");
        }

        if (ctrl_.getFault_ProcTemp().getValue()) {
            results.add("proctemp") ;
        }

        if (ctrl_.getStickyFault_ProcTemp().getValue()) {
            results.add("proctemp") ;
        }        

        if (ctrl_.getFault_DeviceTemp().getValue()) {
            results.add("devicetemp") ;
        }        

        if (ctrl_.getStickyFault_DeviceTemp().getValue()) {
            results.add("sticky-devicetemp") ;
        }

        if (ctrl_.getFault_Undervoltage().getValue()) {
            results.add("undervoltage") ;
        }     
        
        if (ctrl_.getStickyFault_Undervoltage().getValue()) {
            results.add("sticky-undervoltage") ;
        }

        if (ctrl_.getFault_BootDuringEnable().getValue()) {
            results.add("boot-during-enable") ;
        }     
        
        if (ctrl_.getStickyFault_BootDuringEnable().getValue()) {
            results.add("sticky-boot-during-enable") ;
        }

        if (ctrl_.getFault_UnlicensedFeatureInUse().getValue()) {
            results.add("unlicensed-feature") ;
        }     

        if (ctrl_.getStickyFault_UnlicensedFeatureInUse().getValue()) {
            results.add("sticky-unlicensed-feature") ;
        }          

        if (ctrl_.getFault_BridgeBrownout().getValue()) {
            results.add("bridge-brownout") ;
        }     
        
        if (ctrl_.getStickyFault_BridgeBrownout().getValue()) {
            results.add("sticky-bridge-brownout") ;
        }        

        if (ctrl_.getFault_RemoteSensorReset().getValue()) {
            results.add("remote-sensor-reset") ;
        }            

        if (ctrl_.getStickyFault_RemoteSensorReset().getValue()) {
            results.add("sticky-remote-sensor-reset") ;
        }

        if (ctrl_.getFault_MissingDifferentialFX().getValue()) {
            results.add("missing-differential-fx") ;
        }            

        if (ctrl_.getStickyFault_MissingDifferentialFX().getValue()) {
            results.add("sticky-missing-differential-fx") ;
        }

        if (ctrl_.getFault_RemoteSensorPosOverflow().getValue()) {
            results.add("remote-sensor-pos-overflow") ;
        }    
        
        if (ctrl_.getStickyFault_RemoteSensorPosOverflow().getValue()) {
            results.add("sticky-remote-sensor-pos-overflow") ;
        }

        if (ctrl_.getFault_OverSupplyV().getValue()) {
            results.add("over-supply") ;
        }

        if (ctrl_.getStickyFault_OverSupplyV().getValue()) {
            results.add("sticky-over-supply") ;
        }

        if (ctrl_.getFault_UnstableSupplyV().getValue()) {
            results.add("unstable-supply") ;
        }

        if (ctrl_.getStickyFault_UnstableSupplyV().getValue()) {
            results.add("sticky-unstable-supply") ;
        }

        if (ctrl_.getFault_ReverseHardLimit().getValue()) {
            results.add("reverse-hard-limit") ;
        }

        if (ctrl_.getStickyFault_ReverseHardLimit().getValue()) {
            results.add("sticky-reverse-hard-limit") ;
        }

        if (ctrl_.getFault_ForwardHardLimit().getValue()) {
            results.add("forward-hard-limit") ;
        }

        if (ctrl_.getStickyFault_ForwardHardLimit().getValue()) {
            results.add("sticky-forward-hard-limit") ;
        }

        if (ctrl_.getFault_ReverseSoftLimit().getValue()) {
            results.add("reverse-soft-limit") ;
        }

        if (ctrl_.getStickyFault_ReverseSoftLimit().getValue()) {
            results.add("sticky-reverse-soft-limit") ;
        }

        if (ctrl_.getFault_ForwardSoftLimit().getValue()) {
            results.add("forward-soft-limit") ;
        }

        if (ctrl_.getStickyFault_ForwardSoftLimit().getValue()) {
            results.add("sticky-forward-soft-limit") ;
        }        

        if (ctrl_.getFault_RemoteSensorDataInvalid().getValue()) {
            results.add("remote-sensor-data-invalid") ;
        }

        if (ctrl_.getStickyFault_RemoteSensorDataInvalid().getValue()) {
            results.add("sticky-remote-sensor-data-invalid") ;
        }

        if (ctrl_.getFault_FusedSensorOutOfSync().getValue()) {
            results.add("fused-sensor-out-of-sync") ;
        }

        if (ctrl_.getStickyFault_FusedSensorOutOfSync().getValue()) {
            results.add("sticky-fused-sensor-out-of-sync") ;
        }

        if (ctrl_.getFault_StatorCurrLimit().getValue()) {
            results.add("stator-curr-limit") ;
        }

        if (ctrl_.getStickyFault_StatorCurrLimit().getValue()) {
            results.add("sticky-stator-curr-limit") ;
        }

        if (ctrl_.getFault_SupplyCurrLimit().getValue()) {
            results.add("supply-curr-limit") ;
        }

        if (ctrl_.getStickyFault_SupplyCurrLimit().getValue()) {
            results.add("sticky-supply-curr-limit") ;
        }

        if (ctrl_.getFault_UsingFusedCANcoderWhileUnlicensed().getValue()) {
            results.add("using-fused-cancoder-while-unlicensed") ;
        }

        if (ctrl_.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue()) {
            results.add("sticky-using-fused-cancoder-while-unlicensed") ;
        }

        return results ;
    }

    public void clearStickyFaults() {
        ctrl_.clearStickyFaults() ;
    }

    /// \brief Have the current motor follow another motor.
    /// \param leader if true, the leader is inverted versus normal operation
    /// \param invert if true, follow the other motor but with the power inverted.    
    public void follow(IMotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException {
        if (!TalonFX.class.isInstance(ctrl.getNativeController())) {
            throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
        }
        
        leader_ = ctrl ;
        TalonFX other = (TalonFX)ctrl.getNativeController() ;
        boolean i = (invert != leader) ;
        checkError("could not follow another robot", () -> ctrl_.setControl(new Follower(other.getDeviceID(), i))) ;
    }    

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position    
    public boolean hasEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        return true ;
    }

    /// \Brief Return the native motor controller for this controller.
    /// \returns the native motor controller for the motor    
    public Object getNativeController() {
        return ctrl_ ;
    }

    /// \brief Return a human readable string giving the physical motor controller type (e.g. TalonFX, SparkMaxBrushless, etc.)
    /// \returns a human readable string giving the physical motor controller type
    public String getType() throws BadMotorRequestException, MotorRequestFailedException {
        return type_ ;
    }    

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller        
    public String getFirmwareVersion() throws BadMotorRequestException, MotorRequestFailedException {
        StatusSignal<Integer> sig ;
        
        if (major_ == -1) {
            sig = ctrl_.getVersion() ;
            sig.setUpdateFrequency(1000) ;
            int value = sig.waitForUpdate(0.1).getValue() ;
            sig.setUpdateFrequency(0) ;

            major_ = (value >> 24) & 0xff ;
            minor_ = (value >> 16) & 0xff ;
            bugfix_ = (value >> 8) & 0xff ;
            build_ = (value >> 0) & 0xff ;
        }

        return major_ + "." + minor_ + "." + bugfix_ + "." + build_ ;
    }    

    /// \brief Return the number of encoder ticks per revolution for this motor.  If this motor does not
    /// have an encoder, a BadMotorEquestException is thrown.
    /// \returns the number of encoder ticks per revolution for this motor.
    public int ticksPerRevolution() throws BadMotorRequestException, MotorRequestFailedException {
        return kTicksPerRevolution ;
    }    

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given
    public void setCurrentLimit(double limit) throws BadMotorRequestException, MotorRequestFailedException {
        CurrentLimitsConfigs cfgs = cfg_.CurrentLimits ;
        cfgs.SupplyCurrentLimit = limit ;
        cfgs.SupplyCurrentLimitEnable = true ;
        cfgs.SupplyCurrentThreshold = limit ;
        cfgs.SupplyTimeThreshold = 0.1 ;
        checkError("setCurrentLimit", () -> ctrl_.getConfigurator().apply(cfgs)) ;
    }

    /// \brief Returns the current limit for the current supplied to the motor
    /// \returns the current limit for the current supplied to the motor
    public double getCurrentLimit() throws BadMotorRequestException, MotorRequestFailedException {
        double ret = Double.MAX_VALUE ;

        if (cfg_.CurrentLimits.SupplyCurrentLimitEnable) {
            ret = cfg_.CurrentLimits.SupplyCurrentLimit ;
        }
        return ret ;
    }

    /// \brief set the deadband for the motor.  If any power value is assigned to the motor that is less
    /// than this value, zero is assumed.
    /// \param value the deadband value for this motor
    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException {
        MotorOutputConfigs cfgs = cfg_.MotorOutput ;
        cfgs.DutyCycleNeutralDeadband = value ;
        checkError("setNeutralDeadband", () -> ctrl_.getConfigurator().apply(cfgs)) ;
    } 

    /// \brief Get the deadband value for the motor
    /// \returns the deadband value for the motor
    public double getNeutralDeadband() throws BadMotorRequestException, MotorRequestFailedException {
        return cfg_.MotorOutput.DutyCycleNeutralDeadband ;
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public void setNeutralMode(XeroNeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        MotorOutputConfigs cfgs = cfg_.MotorOutput ;
        cfgs.NeutralMode = (mode == XeroNeutralMode.Brake) ? NeutralModeValue.Brake : NeutralModeValue.Coast ;
        checkError("setNeutralMode", () -> ctrl_.getConfigurator().apply(cfgs)) ;        
    }
    
    /// \brief Get the neutral mode for the motor
    /// \returns the neutral mode for the motor
    public XeroNeutralMode getNeutralMode() throws BadMotorRequestException, MotorRequestFailedException {
        XeroNeutralMode ret = XeroNeutralMode.Coast ;

        if (cfg_.MotorOutput.NeutralMode == NeutralModeValue.Brake) {
            ret = XeroNeutralMode.Brake ;
        }
        return ret ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not
    public void setInverted(boolean inverted) throws BadMotorRequestException , MotorRequestFailedException {
        MotorOutputConfigs cfgs = cfg_.MotorOutput ;
        cfgs.Inverted = (inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive) ;
        checkError("setInverted", () -> ctrl_.getConfigurator().apply(cfgs)) ;  
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted
    public boolean isInverted()  throws BadMotorRequestException , MotorRequestFailedException {
        return cfg_.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive ;
    }

    /// \brief Return the leader for the current motor.  If the current motor is not following, return null.
    /// \returns the leader for the current motor.
    public IMotorController getLeader() {
        return leader_ ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID(XeroPidType type) throws BadMotorRequestException , MotorRequestFailedException {
        return true ;
    }    

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param v the velocity feed forward parameter for the PID controller
    /// \param a the acceleration feed forward parameter for the PID controller
    /// \param g the gravity feed forward parameter for the PID controller
    /// \param s the static friction feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller 
    public void setPID(XeroPidType type, double p, double i, double d, double v, double a, double g, double s, double outmax) throws BadMotorRequestException , MotorRequestFailedException {
        Slot0Configs cfg = cfg_.Slot0 ;

        cfg.kP = p * kTicksPerRevolution ;
        cfg.kI = i * kTicksPerRevolution ;
        cfg.kD = d * kTicksPerRevolution ;
        cfg.kV = v * kTicksPerRevolution ;
        cfg.kA = a * kTicksPerRevolution ;
        cfg.kG = g ;
        cfg.kS = s ;
        checkError("setPID()", () -> ctrl_.getConfigurator().apply(cfg));

        VoltageConfigs mo = cfg_.Voltage ;
        mo.PeakForwardVoltage = outmax * 12.0 ;
        mo.PeakReverseVoltage = -outmax * 12.0 ;
        checkError("setPID()", () -> ctrl_.getConfigurator().apply(mo));
    }

    /// \brief Set the parameters for motion magic
    /// \param v the max velocity for the motion
    /// \param a the max acceleration for the motion
    /// \param j the max jerk for the motion
    public void setMotionMagicParams(double v, double a, double j) throws BadMotorRequestException, MotorRequestFailedException {
        MotionMagicConfigs cfg = cfg_.MotionMagic ;
        cfg.MotionMagicAcceleration = a / kTicksPerRevolution;
        cfg.MotionMagicCruiseVelocity = v  / kTicksPerRevolution;
        cfg.MotionMagicJerk = j / kTicksPerRevolution;

        checkError("setMotionMagicParams()", () -> ctrl_.getConfigurator().apply(cfg));
    }        

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type    
    public void set(XeroPidType type, double target) throws BadMotorRequestException, MotorRequestFailedException {
        ControlRequest req = null ;
        switch(type) {
            case Power:
                if (voltage_compensation_enabled_) {
                    req = new VoltageOut(target * nominal_voltage_) ;
                }
                else {
                    req = new DutyCycleOut(target) ;
                }
                break ;
            case Position:
                if (voltage_compensation_enabled_) {
                    req = new PositionVoltage(target / kTicksPerRevolution) ;
                }
                else {
                    req = new PositionDutyCycle(target / kTicksPerRevolution) ;
                }
                break ;
            case Velocity:
                if (voltage_compensation_enabled_) {
                    req = new VelocityVoltage(target / kTicksPerRevolution) ;
                }
                else {
                    req = new VelocityDutyCycle(target / kTicksPerRevolution) ;
                }
                break ;
            case MotionMagic:
                if (voltage_compensation_enabled_) {
                    req = new MotionMagicVoltage(target / kTicksPerRevolution) ;
                }
                else {
                    req = new MotionMagicDutyCycle(target / kTicksPerRevolution) ;
                }
                break ;
        }

        final ControlRequest freq = req ;
        checkError("set()", () -> ctrl_.setControl(freq)) ;
    }

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(ImportantType value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = 0.0 ;

        switch(value) {
            case Off:
                freq = 0.0 ;
                break ;
            case Low:
                freq = 4.0 ;
                break ;
            case High:
                freq = 100;
                break; 
            case Invalid:
                freq = 1.0 ;
                break ;                  
        }
        final double tfreq = freq ;
        checkError("setPositionImportant()", () -> ctrl_.getPosition().setUpdateFrequency(tfreq)) ;
    }
    
    /// \brief If value is true, the motor controller will consider velocity data as important and update
    /// the data a quickly as possible.
    public void setVelocityImportant(ImportantType value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = 0.0 ;

        switch(value) {
            case Off:
                freq = 0.0 ;
                break ;
            case Low:
                freq = 4.0 ;
                break ;
            case High:
                freq = 100;
                break; 
            case Invalid:
                freq = 1.0 ;
                break ;                
        }
        final double tfreq = freq ;        
        checkError("setVelocityImportant error", () -> ctrl_.getVelocity().setUpdateFrequency(tfreq)) ;
    }
    
    /// \brief If value is true, the motor controller will consider acceleration data as important and update
    /// the data a quickly as possible.
    public void setAccelerationImportant(ImportantType value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = 0.0 ;

        switch(value) {
            case Off:
                freq = 0.0 ;
                break ;
            case Low:
                freq = 4.0 ;
                break ;
            case High:
                freq = 100;
                break; 
            case Invalid:
                freq = 1.0 ;
                break ;
        }
        final double tfreq = freq ;        
        checkError("setAccelerationImportant error", () -> ctrl_.getAcceleration().setUpdateFrequency(tfreq)) ;
    }

    /// \brief Returns the position of the motor in motor units from the motor controller.  If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the position of the motor in encoder ticks
    public double getPosition() throws BadMotorRequestException, MotorRequestFailedException {          
        return ctrl_.getPosition().getValue() * kTicksPerRevolution ;
    }
    
    /// \brief Return the velocity of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the velocity of the motor in ticks per second
    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        double ret = 0.0 ;

        ret = ctrl_.getVelocity().getValue() * kTicksPerRevolution ;

        return ret;
    }

    /// \brief Return the acceleration of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the acceleration of the motor in ticks per second squared
    public double getAcceleration() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getAcceleration().getValue() * kTicksPerRevolution ;
    }

    public boolean hasAcceleration() throws BadMotorRequestException, MotorRequestFailedException {
        return true ;
    }

    /// \brief Reset the encoder values to zero    
    public void resetEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        checkError("resetEncoder", () -> ctrl_.setPosition(0.0)) ;
    }

     /// \brief Set the encoder to a specific value in ticks
     /// \param pos the new value for the encoder in ticks
     public void setPosition(double value) throws BadMotorRequestException, MotorRequestFailedException {
        checkError("setPosition", () -> ctrl_.setPosition(value / kTicksPerRevolution)) ;
     }    

    /// \brief Enable voltage compensation for the given motor
    /// \param enabled if true voltage compensation is enabled
    /// \param nominal if enabled is true, this is the nominal voltage for compensation
    public void enableVoltageCompensation(boolean enabled, double nominal) throws BadMotorRequestException, MotorRequestFailedException {
        voltage_compensation_enabled_ = enabled ;
        nominal_voltage_ = nominal ;
    }

    /// \brief Return the closed loop target
    /// \returns  the closed loop target
    public double getClosedLoopTarget() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getClosedLoopReference().getValue() * kTicksPerRevolution ;
    }

    /// \brief Return the closed loop error
    /// \returns  the closed loop error
    public double getClosedLoopError() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getClosedLoopError().getValue() * kTicksPerRevolution ;
    }

    public double getVoltage() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getMotorVoltage().getValue() ;
    }    

    public double getCurrentTargetPosition() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getClosedLoopReference().getValue() ;
    }

    public double getCurrentTargetVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getClosedLoopReferenceSlope().getValue() ;
    }

    public void enableSoftForwardLimit(double value) throws BadMotorRequestException, MotorRequestFailedException {
        cfg_.SoftwareLimitSwitch.ForwardSoftLimitEnable = true ;
        cfg_.SoftwareLimitSwitch.ForwardSoftLimitThreshold = value / kTicksPerRevolution ;
        checkError("enableSoftForwardLimit", () -> ctrl_.getConfigurator().apply(cfg_.SoftwareLimitSwitch)) ;
    }

    public void disableSoftForwardLimit() throws BadMotorRequestException, MotorRequestFailedException {
        cfg_.SoftwareLimitSwitch.ForwardSoftLimitEnable = false ;
        checkError("disableSoftForwardLimit", () -> ctrl_.getConfigurator().apply(cfg_.SoftwareLimitSwitch)) ;
    }

    public void enableSoftReverseLimit(double value) throws BadMotorRequestException, MotorRequestFailedException {
        cfg_.SoftwareLimitSwitch.ReverseSoftLimitEnable = true ;
        cfg_.SoftwareLimitSwitch.ReverseSoftLimitThreshold = value / kTicksPerRevolution ;
        checkError("enableSoftReverseLimit", () -> ctrl_.getConfigurator().apply(cfg_.SoftwareLimitSwitch)) ;
    }

    public void disableSoftReverseLimit() throws BadMotorRequestException, MotorRequestFailedException {
        cfg_.SoftwareLimitSwitch.ReverseSoftLimitEnable = false ;
        checkError("disableSoftReverseLimit", () -> ctrl_.getConfigurator().apply(cfg_.SoftwareLimitSwitch)) ;
    }
} ;
