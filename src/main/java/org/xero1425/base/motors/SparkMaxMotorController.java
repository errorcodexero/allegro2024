package org.xero1425.base.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

/// \file

/// \brief This class is MotorController class that supports the SparkMax motor controller.   This class is a 
/// wrapper for the CANSparkMax class that provides the interface that meets the requirements of the 
/// MotorController base class.  This class supports both brushless and brushed motors.
public class SparkMaxMotorController extends MotorController
{
    static final int kTicksPerRevolution = 42 ;
    static final String kRobotType = "SparkMaxBrushless" ;

    private int canid_ ;
    private CANSparkMax ctrl_ ;
    private SparkPIDController pid_ ;
    private RelativeEncoder encoder_ ;
    private MotorType mtype_ ;
    private boolean inverted_ ;
    private IMotorController leader_ ;
    private NeutralMode neutral_mode_ ;
    private double deadband_ ;
    private double current_limit_ ;
    private boolean pos_important_ ;
    private boolean vel_important_ ;

    public SparkMaxMotorController(String name, int canid, boolean brushless, boolean leader) throws MotorRequestFailedException {
        super(name) ;

        canid_ = canid ;

        if (brushless) {
            mtype_ = MotorType.kBrushless;
        }
        else {
            mtype_ = MotorType.kBrushed ;
        }

        ctrl_ = new CANSparkMax(canid, mtype_);
        ctrl_.restoreFactoryDefaults() ;

        pid_ = ctrl_.getPIDController() ;
        encoder_ = ctrl_.getEncoder() ;

        // Output the position in ticks
        checkError("constructor", encoder_.setPositionConversionFactor(kTicksPerRevolution)) ;

        // Output velocity in ticks per second
        encoder_.setVelocityConversionFactor(kTicksPerRevolution / 60.0) ;

        inverted_ = false ;
        pos_important_ = false ;
        vel_important_ = false ;
        current_limit_ = Double.MAX_VALUE ;
    }

    private void checkError(String msg, REVLibError err) throws MotorRequestFailedException {
        if (err != REVLibError.kOk) {
            throw new MotorRequestFailedException(this, msg, err) ;            
        }
    }

    /// \brief Returns the CAN ID of the motor
    /// \returns the CAN ID of the motor    
    public int getCanID() throws BadMotorRequestException, MotorRequestFailedException {
        return canid_ ;
    }

    /// \brief Returns the name of the CAN bus.  An empty string means roborio
    /// \return the name of the CAN bus.  An empty string means roborio    
    public String getBus() throws BadMotorRequestException, MotorRequestFailedException {
        return "" ;
    }    

    /// \brief Have the current motor follow another motor.
    /// \param leader if true, the leader is inverted versus normal operation
    /// \param invert if true, follow the other motor but with the power inverted.    
    public void follow(IMotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException {
        if (!CANSparkMax.class.isInstance(ctrl.getNativeController())) {
            throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
        }
        
        CANSparkMax other = (CANSparkMax)ctrl.getNativeController() ;
        boolean i = (invert != leader) ;
        checkError("could not follow another robot", ctrl_.follow(other, i)) ;
    }    

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position    
    public boolean hasEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        return mtype_ == MotorType.kBrushed ;
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
        int v = ctrl_.getFirmwareVersion() ;

        return String.valueOf((v >> 24) & 0xff) + "." + String.valueOf((v >> 16) & 0xff) + "." + 
                String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;
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
        checkError("could not set current limit", ctrl_.setSmartCurrentLimit((int)limit)) ;
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
        ctrl_.setIdleMode((mode == NeutralMode.Brake) ? IdleMode.kBrake : IdleMode.kCoast);
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
        if (type == PidType.MotionMagic)
            return false ;

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
        checkError("could not set PID controller P value", pid_.setP(p));
        checkError("could not set PID controller I value", pid_.setI(i));
        checkError("could not set PID controller D value", pid_.setD(d));
        checkError("could not set PID controller V value", pid_.setFF(v));
        checkError("could not set PID controller OUTMAX value", pid_.setOutputRange(-outmax, outmax));
    }

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type    
    public void set(PidType type, double target) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushless && type != PidType.None) {
            throw new BadMotorRequestException(this, "brushed motor does not support PID") ;
        }

        switch(type) {
            case None:
                checkError("could not set voltage", pid_.setReference(target, CANSparkMax.ControlType.kVoltage)) ;
                break ;            
            case Position:
                checkError("could not set position", pid_.setReference(target, CANSparkMax.ControlType.kPosition)) ;
                break ;            
            case Velocity:
                checkError("could not set velocity", pid_.setReference(target, CANSparkMax.ControlType.kVelocity)) ;
                break ;            
            case MotionMagic:
                throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support motion magic") ;
        }
    }

    private void updateStatusFreqs() throws MotorRequestFailedException {
        int k1Period = (pos_important_ ? 10 : 500) ; 
        int k2Period = (vel_important_ ? 10 : 500) ;

        checkError("could not set status frame frequency",ctrl_.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1Period)) ;
        checkError("could not set status frame frequency",ctrl_.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2Period)) ;
    }

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushless)
            throw new BadMotorRequestException(this, "brushed motor does not support position") ;

        pos_important_ = true ;
        updateStatusFreqs() ;
    }
    
    /// \brief If value is true, the motor controller will consider velocity data as important and update
    /// the data a quickly as possible.
    public void setVelocityImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushless)
            throw new BadMotorRequestException(this, "brushed motor does not support velocity") ;

        vel_important_ = true ;
        updateStatusFreqs() ;
    }
    
    /// \brief If value is true, the motor controller will consider acceleration data as important and update
    /// the data a quickly as possible.
    public void setAccelerationImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support acceleration") ;       
    }

    /// \brief Returns the position of the motor in motor units from the motor controller.  If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the position of the motor in encoder ticks
    public double getPosition() throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushless)
            throw new BadMotorRequestException(this, "brushed motor does not support position") ;

        return encoder_.getPosition() ;
    }
    
    /// \brief Return the velocity of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the velocity of the motor in ticks per second
    public double getVelocity()  throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushless)
            throw new BadMotorRequestException(this, "brushed motor does not support velocity") ;

        return encoder_.getVelocity() ;
    }

    /// \brief Return the acceleration of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the acceleration of the motor in ticks per second squared
    public double getAcceleration() throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support acceleration") ; 
    }

    /// \brief Reset the encoder values to zero    
    public void resetEncoder() throws BadMotorRequestException {
        if (mtype_ == MotorType.kBrushless)
            throw new BadMotorRequestException(this, "brushed motor does not support resetEncoder()") ;

        encoder_.setPosition(0.0) ;
    }
} ;
