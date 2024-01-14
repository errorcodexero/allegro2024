package org.xero1425.base.motors;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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
    private final static int kTicksPerRevolution = 2048 ;
    private final static String kRobotType = "TalonFX" ;

    private TalonFX ctrl_ ;
    private TalonFXConfiguration cfg_ ;
    private IMotorController leader_ ;
    private String bus_ ;

    public TalonFXMotorController(String name, String bus, int canid, boolean leader) throws MotorRequestFailedException {
        super(name) ;

        bus_ = bus ;
        ctrl_ = new TalonFX(canid, bus_);
        cfg_ = new TalonFXConfiguration() ;

        checkError("TalonFXMotorController - apply configuration", ctrl_.getConfigurator().apply(cfg_));
        checkError("TalonFXMotorController - optimize bus", ctrl_.optimizeBusUtilization()) ;

    }

    private void checkError(String msg, StatusCode err) throws MotorRequestFailedException {
        if (err != StatusCode.OK) {
            throw new MotorRequestFailedException(this, msg, err) ;            
        }
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

        if (ctrl_.getFault_BridgeBrownout().getValue()) {
            results.add("bridge-brownout") ;
        }     
        
        if (ctrl_.getStickyFault_BridgeBrownout().getValue()) {
            results.add("sticky-bridge-brownout") ;
        }        

        if (ctrl_.getFault_RemoteSensorReset().getValue()) {
            results.add("undervoltage") ;
        }   
        
        if (ctrl_.getStickyFault_RemoteSensorReset().getValue()) {
            results.add("undervoltage") ;
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
            results.add("stick-remote-sensor-pos-overflow") ;
        }      
        
        if (ctrl_.getFault_OverSupplyV().getValue()) {
            results.add("over-supply") ;
        }           

        if (ctrl_.getStickyFault_OverSupplyV().getValue()) {
            results.add("sticky-over-supply") ;
        }    
        
        if (ctrl_.getFault_UnstableSupplyV().getValue()) {
            results.add("over-supply") ;
        }          

        if (ctrl_.getStickyFault_UnstableSupplyV().getValue()) {
            results.add("sticky-over-supply") ;
        }          

        return results ;
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
        checkError("could not follow another robot", ctrl_.setControl(new Follower(other.getDeviceID(), i))) ;
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
        return kRobotType ;
    }    

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller        
    public String getFirmwareVersion() throws BadMotorRequestException, MotorRequestFailedException {
        return "0.0.0.0" ;
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
        checkError("setCurrentLimit", ctrl_.getConfigurator().apply(cfgs)) ;
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
        checkError("setNeutralDeadband", ctrl_.getConfigurator().apply(cfgs)) ;
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
        checkError("setNeutralMode", ctrl_.getConfigurator().apply(cfgs)) ;        
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
        checkError("setInverted", ctrl_.getConfigurator().apply(cfgs)) ;  
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

        cfg.kP = p ;
        cfg.kI = i ;
        cfg.kD = d ;
        cfg.kV = v ;
        cfg.kA = a ;
        cfg.kG = g ;
        cfg.kS = s ;
        checkError("setPID()", ctrl_.getConfigurator().apply(cfg));

        VoltageConfigs mo = cfg_.Voltage ;
        mo.PeakForwardVoltage = outmax * 12.0 ;
        mo.PeakReverseVoltage = -outmax * 12.0 ;
        checkError("setPID()", ctrl_.getConfigurator().apply(mo));
    }

    /// \brief Set the parameters for motion magic
    /// \param v the max velocity for the motion
    /// \param a the max acceleration for the motion
    /// \param j the max jerk for the motion
    public void setMotionMagicParams(double v, double a, double j) throws BadMotorRequestException, MotorRequestFailedException {
        MotionMagicConfigs cfg = cfg_.MotionMagic ;
        cfg.MotionMagicAcceleration = a ;
        cfg.MotionMagicCruiseVelocity = v ;
        cfg.MotionMagicJerk = j ;

        checkError("setMotionMagicParams()", ctrl_.getConfigurator().apply(cfg));
    }        

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type    
    public void set(XeroPidType type, double target) throws BadMotorRequestException, MotorRequestFailedException {
        ControlRequest req = null ;
        switch(type) {
            case Voltage:
                req = new VoltageOut(target * 12.0) ;
                break ;
            case Position:
                req = new PositionVoltage(0).withPosition(target / kTicksPerRevolution) ;
                break ;
            case Velocity:
                req = new VelocityVoltage(0).withVelocity(target / kTicksPerRevolution) ;
                break ;
            case MotionMagic:
                req = new MotionMagicVoltage(0).withPosition(target / kTicksPerRevolution) ;
                break ;
        }

        checkError("set()", ctrl_.setControl(req)) ;
    }

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = (value ? 100 : 1) ;
        checkError("setPositionImportant()", ctrl_.getPosition().setUpdateFrequency(freq)) ;
    }
    
    /// \brief If value is true, the motor controller will consider velocity data as important and update
    /// the data a quickly as possible.
    public void setVelocityImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = (value ? 100 : 1) ;
        checkError("setPositionImportant error", ctrl_.getVelocity().setUpdateFrequency(freq)) ;
    }
    
    /// \brief If value is true, the motor controller will consider acceleration data as important and update
    /// the data a quickly as possible.
    public void setAccelerationImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = (value ? 100 : 1) ;
        checkError("setPositionImportant error", ctrl_.getAcceleration().setUpdateFrequency(freq)) ;
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
    public double getVelocity()  throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getVelocity().getValue() * kTicksPerRevolution ;
    }

    /// \brief Return the acceleration of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the acceleration of the motor in ticks per second squared
    public double getAcceleration() throws BadMotorRequestException, MotorRequestFailedException {
        return ctrl_.getAcceleration().getValue() * kTicksPerRevolution ;
    }

    /// \brief Reset the encoder values to zero    
    public void resetEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        checkError("resetEncoder" ,ctrl_.setPosition(0.0)) ;
    }

     /// \brief Set the encoder to a specific value in ticks
     /// \param pos the new value for the encoder in ticks
     public void setPosition(double value) throws BadMotorRequestException, MotorRequestFailedException {
        checkError("setPosition", ctrl_.setPosition(value / kTicksPerRevolution)) ;
     }    
} ;
