package org.xero1425.base.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

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
    private double current_limit_ ;
    private double deadband_ ;
    private NeutralMode neutral_mode_ ;
    private boolean inverted_ ;
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

    /// \brief Have the current motor follow another motor.
    /// \param leader if true, the leader is inverted versus normal operation
    /// \param invert if true, follow the other motor but with the power inverted.    
    public void follow(IMotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException {
        if (!TalonFX.class.isInstance(ctrl.getNativeController())) {
            throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
        }
        
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
        int v = ctrl_.
        return String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;        
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
        current_limit_ = limit ;
    }

    /// \brief Returns the current limit for the current supplied to the motor
    /// \returns the current limit for the current supplied to the motor
    public double getCurrentLimit() throws BadMotorRequestException, MotorRequestFailedException {
        return current_limit_ ;
    }

    /// \brief set the deadband for the motor.  If any power value is assigned to the motor that is less
    /// than this value, zero is assumed.
    /// \param value the deadband value for this motor
    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException {
        deadband_ = value ;
    } 

    /// \brief Get the deadband value for the motor
    /// \returns the deadband value for the motor
    public double getNeutralDeadband() throws BadMotorRequestException, MotorRequestFailedException {
        return deadband_ ;
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        neutral_mode_ = mode ;
    }
    
    /// \brief Get the neutral mode for the motor
    /// \returns the neutral mode for the motor
    public NeutralMode getNeutralMode() throws BadMotorRequestException, MotorRequestFailedException {
        return neutral_mode_ ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not
    public void setInverted(boolean inverted) throws BadMotorRequestException , MotorRequestFailedException {
        inverted_ = inverted ;
        ctrl_.setInverted(inverted_);
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted
    public boolean isInverted()  throws BadMotorRequestException , MotorRequestFailedException {
        return inverted_ ;
    }

    /// \brief Return the leader for the current motor.  If the current motor is not following, return null.
    /// \returns the leader for the current motor.
    public IMotorController getLeader() {
        return leader_ ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID(PidType type) throws BadMotorRequestException , MotorRequestFailedException {
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
    public void setPID(PidType type, double p, double i, double d, double v, double a, double g, double s, double outmax) throws BadMotorRequestException , MotorRequestFailedException {
        Slot0Configs cfg = cfg_.Slot0 ;

        cfg.kP = p ;
        cfg.kI = i ;
        cfg.kD = d ;
        cfg.kV = v ;
        cfg.kA = a ;
        cfg.kG = g ;
        cfg.kS = s ;

        checkError("setPID()", ctrl_.getConfigurator().apply(cfg));
    }

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type    
    public void set(PidType type, double target) throws BadMotorRequestException, MotorRequestFailedException {
        ControlRequest req = null ;
        switch(type) {
            case None:
                req = new VoltageOut(target) ;
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

        checkError("set the target for the motor", ctrl_.setControl(req)) ;
    }

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        double freq = (value ? 100 : 1) ;
        checkError("setPositionImportant error", ctrl_.getPosition().setUpdateFrequency(freq)) ;
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
    public void resetEncoder() throws BadMotorRequestException {
    }
} ;
