package org.xero1425.base.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.RobotBase;

/// \file

/// \brief This class is MotorController class that supports the SparkMax motor controller.   This class is a 
/// wrapper for the CANSparkMax class that provides the interface that meets the requirements of the 
/// MotorController base class.  This class supports both brushless and brushed motors.
public class SparkMaxMotorController extends MotorController
{
    public class SimState {
        private double voltage_ ;
        private double position_ ;
        private double velocity_ ;

        private boolean inverted_ ;

        public SimState() {
            voltage_ = 0.0 ;
            inverted_ = false ;
        }

        public double getMotorVoltage() {
            return voltage_ ;
        }

        public void setMotorVoltage(double v) {
            voltage_ = v ;
        }

        public void setInterved(boolean i) {
            inverted_ = i ;
        }

        public boolean getInverted() {
            return inverted_ ;
        }

        public double getPosition() {
            return position_ ;
        }

        public void setPosition(double v) {
            position_ = v ;
        }

        public double getVelocity() {
            return velocity_ ;
        }

        public void setVelocity(double v) {
            velocity_ = v ;
        }
    }

    static public final String SimDeviceNameBrushed = "SparkMaxBrushed" ;
    static public final String SimDeviceNameBrushless = "SparkMax" ;
    static final int kTicksPerRevolution = 42 ;
    static private final int kApplyTries = 5 ;
    static final String kRobotType = "SparkMaxBrushless" ;

    private int canid_ ;
    private CANSparkMax ctrl_ ;
    private SparkPIDController pid_ ;
    private RelativeEncoder encoder_ ;
    private MotorType mtype_ ;
    private boolean inverted_ ;
    private IMotorController leader_ ;
    private XeroNeutralMode neutral_mode_ ;
    private double deadband_ ;
    private double current_limit_ ;
    private ImportantType pos_important_ ;
    private ImportantType vel_important_ ;
    private double target_ ;

    private SimState simstate_ ;

    public SparkMaxMotorController(String name, int canid, boolean brushless, boolean leader) throws MotorRequestFailedException {
        super(name) ;

        canid_ = canid ;
        inverted_ = false ;
        pos_important_ = ImportantType.Off ;
        vel_important_ = ImportantType.Off ;
        current_limit_ = Double.MAX_VALUE ;
        target_ = 0.0 ;

        if (brushless) {
            mtype_ = MotorType.kBrushless;
        }
        else {
            mtype_ = MotorType.kBrushed ;
        }        

        if (RobotBase.isSimulation()) {
            simstate_ = new SimState() ;
        }
        else {
            ctrl_ = new CANSparkMax(canid, mtype_);
            checkError("restoreFactoryDefaults - restoreFactoryDefaults", () -> ctrl_.restoreFactoryDefaults());

            pid_ = ctrl_.getPIDController() ;
            encoder_ = ctrl_.getEncoder() ;
            
            // Output the position in ticks
            checkError("constructor - setPositionConversionFactor", () -> encoder_.setPositionConversionFactor(kTicksPerRevolution)) ;

            // Output velocity in ticks per second
            checkError("constructor - setVelocityConversionFactor", () -> encoder_.setVelocityConversionFactor(kTicksPerRevolution / 60.0)) ;
        }
    }

    public SimState getSimState() {
        return simstate_ ;
    }

    private void checkError(String msg, Supplier<REVLibError> toApply) throws MotorRequestFailedException {
        REVLibError code = REVLibError.kInvalid ;
        int tries = kApplyTries ;
        do {
            code = toApply.get() ;
        } while (code != REVLibError.kOk && --tries > 0)  ;

        if (code != REVLibError.kOk) {
            throw new MotorRequestFailedException(this, msg, code) ;
        }
    }

    /// \brief Return list of faults detected by the motor controller
    /// \returns list of faults detected by the motor controller
    public List<String> getFaults() throws BadMotorRequestException, MotorRequestFailedException {
        ArrayList<String> faults = new ArrayList<String>() ;

        if (simstate_ == null) {
            if (ctrl_.getFault(FaultID.kBrownout)) {
                faults.add("kBrownout") ;
            }

            if (ctrl_.getFault(FaultID.kOvercurrent)) {
                faults.add("kOvercurrent") ;
            }

            if (ctrl_.getFault(FaultID.kIWDTReset)) {
                faults.add("kIWDTReset") ;
            }

            if (ctrl_.getFault(FaultID.kMotorFault)) {
                faults.add("kMotorFault") ;
            }

            if (ctrl_.getFault(FaultID.kSensorFault)) {
                faults.add("kSensorFault") ;
            }    
            
            if (ctrl_.getFault(FaultID.kStall)) {
                faults.add("kStall") ;
            }    
            
            if (ctrl_.getFault(FaultID.kEEPROMCRC)) {
                faults.add("kEEPROMCRC") ;
            }    
            
            if (ctrl_.getFault(FaultID.kCANTX)) {
                faults.add("kCANTX") ;
            }
        
            if (ctrl_.getFault(FaultID.kCANRX)) {
                faults.add("kCANRX") ;
            }  

            if (ctrl_.getFault(FaultID.kHasReset)) {
                faults.add("kHasReset") ;
            }  
            
            if (ctrl_.getFault(FaultID.kDRVFault)) {
                faults.add("kDRVFault") ;
            }  

            if (ctrl_.getFault(FaultID.kOtherFault)) {
                faults.add("kOtherFault") ;
            }  

            if (ctrl_.getFault(FaultID.kSoftLimitFwd)) {
                faults.add("kSoftLimitFwd") ;
            }  
            
            if (ctrl_.getFault(FaultID.kSoftLimitRev)) {
                faults.add("kSoftLimitRev") ;
            }  
            
            if (ctrl_.getFault(FaultID.kHardLimitFwd)) {
                faults.add("kHardLimitFwd") ;
            }  
            
            if (ctrl_.getFault(FaultID.kHardLimitRev)) {
                faults.add("kHardLimitRev") ;
            }
        }

        return faults ;
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
        leader_ = ctrl ;

        if (simstate_ == null) {
            if (!CANSparkMax.class.isInstance(ctrl.getNativeController())) {
                throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
            }

            CANSparkMax other = (CANSparkMax)ctrl.getNativeController() ;
            boolean i = (invert != leader) ;
            checkError("could not follow another robot", () -> ctrl_.follow(other, i)) ;
        }
    }    

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position    
    public boolean hasEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        return mtype_ == MotorType.kBrushless;
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
        String ret = "42.42.42.42" ;

        if (simstate_ == null) {
            int v = ctrl_.getFirmwareVersion() ;
            ret = String.valueOf((v >> 24) & 0xff) + "." + String.valueOf((v >> 16) & 0xff) + "." + 
                    String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;
        }

        return ret;
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

        if (simstate_ == null) {
            checkError("could not set current limit", () -> ctrl_.setSmartCurrentLimit((int)limit)) ;
        }
    }

    /// \brief Returns the current limit for the current supplied to the motor
    /// \returns the current limit for the current supplied to the motor
    public double getCurrentLimit() throws BadMotorRequestException, MotorRequestFailedException {
        return current_limit_ ;
    }

    /// \brief set the deadband for the motor.  If any power value is assigned to the motor that is less
    /// than this value, zero is assumed.  This is not supported for NEO motors
    /// \param value the deadband value for this motor
    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException {
        deadband_ = value ;

        throw new BadMotorRequestException(this, "setNeutralDeadband is not supported for SparkMax controllers") ;
    } 

    /// \brief Get the deadband value for the motor
    /// \returns the deadband value for the motor
    public double getNeutralDeadband() throws BadMotorRequestException, MotorRequestFailedException {
        return deadband_ ;
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public void setNeutralMode(XeroNeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        neutral_mode_ = mode ;

        if (simstate_ == null) {
            ctrl_.setIdleMode((mode == XeroNeutralMode.Brake) ? IdleMode.kBrake : IdleMode.kCoast);
        }
    }
    
    /// \brief Get the neutral mode for the motor
    /// \returns the neutral mode for the motor
    public XeroNeutralMode getNeutralMode() throws BadMotorRequestException, MotorRequestFailedException {
        return neutral_mode_ ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not
    public void setInverted(boolean inverted) throws BadMotorRequestException , MotorRequestFailedException {
        inverted_ = inverted ;

        if (simstate_ != null) {
            simstate_.setInterved(inverted) ;
        }
        else {
            ctrl_.setInverted(inverted_);
        }
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
    public boolean hasPID(XeroPidType type) throws BadMotorRequestException , MotorRequestFailedException {
        if (type == XeroPidType.MotionMagic)
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
    public void setPID(XeroPidType type, double p, double i, double d, double v, double a, double g, double s, double outmax) throws BadMotorRequestException , MotorRequestFailedException {
        if (simstate_ != null) {
            checkError("could not set PID controller P value", () -> pid_.setP(p));
            checkError("could not set PID controller I value", () -> pid_.setI(i));
            checkError("could not set PID controller D value", () -> pid_.setD(d));
            checkError("could not set PID controller V value", () -> pid_.setFF(v));
            checkError("could not set PID controller OUTMAX value", () -> pid_.setOutputRange(-outmax, outmax));
        }
        else {
            throw new BadMotorRequestException(this, "setPID is not supported for SparkMax when simulating") ;
        }
    }

    /// \brief Set the parameters for motion magic
    /// \param v the max velocity for the motion
    /// \param a the max acceleration for the motion
    /// \param j the max jerk for the motion
    public void setMotionMagicParams(double v, double a, double j) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support motion magic") ;        
    }    

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type    
    public void set(XeroPidType type, double target) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushed && type != XeroPidType.Power) {
            throw new BadMotorRequestException(this, "brushed motor does not support PID") ;
        }

        target_ = target ;        
        if (simstate_ != null) {
            if (type == XeroPidType.Power) {
                this.simstate_.setMotorVoltage(target * 12.0) ;
            }
            else {
                throw new BadMotorRequestException(this, "this control type not supported in simulation mode");
            }
        }
        else {
            switch(type) {
                case Power:
                    checkError("could not set voltage", () -> pid_.setReference(target * 12.0, CANSparkMax.ControlType.kVoltage)) ;
                    break ;            
                case Position:
                    checkError("could not set position", () -> pid_.setReference(target, CANSparkMax.ControlType.kPosition)) ;
                    break ;            
                case Velocity:
                    checkError("could not set velocity", () -> pid_.setReference(target, CANSparkMax.ControlType.kVelocity)) ;
                    break ;            
                case MotionMagic:
                    throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support motion magic") ;
            }
        }
    }

    private int importantValueToMS(ImportantType value) {
        int period = 5000 ;

        switch(value) {
            case Off:
                period = 5000 ;
                break ;

            case Low:
                period = 500 ;
                break ;

            case High:
                period = 10;
                break ;

            case Invalid:
                period = 5000 ;
                break;
        }

        return period ;
    }

    private void updateStatusFreqs() throws MotorRequestFailedException {

        if (simstate_ == null) {

            int k1Period = importantValueToMS(pos_important_) ;
            int k2Period = importantValueToMS(vel_important_);

            checkError("could not set status frame frequency",() -> ctrl_.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1Period)) ;
            checkError("could not set status frame frequency",() -> ctrl_.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2Period)) ;
        }
    }

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(ImportantType value) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushed)
            throw new BadMotorRequestException(this, "brushed motor does not support position") ;

        pos_important_ = value ;
        updateStatusFreqs() ;
    }
    
    /// \brief If value is true, the motor controller will consider velocity data as important and update
    /// the data a quickly as possible.
    public void setVelocityImportant(ImportantType value) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushed)
            throw new BadMotorRequestException(this, "brushed motor does not support velocity") ;

        vel_important_ = value ;
        updateStatusFreqs() ;
    }
    
    /// \brief If value is true, the motor controller will consider acceleration data as important and update
    /// the data a quickly as possible.
    public void setAccelerationImportant(ImportantType value) throws BadMotorRequestException, MotorRequestFailedException {
        // throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support acceleration") ;       
    }

    /// \brief Returns the position of the motor in motor units from the motor controller.  If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the position of the motor in encoder ticks
    public double getPosition() throws BadMotorRequestException, MotorRequestFailedException {
        double ret = 0.0 ;

        if (mtype_ == MotorType.kBrushed)
            throw new BadMotorRequestException(this, "brushed motor does not support position") ;

        if (simstate_ != null) {
            ret = simstate_.getPosition() ;
        }
        else {      
            ret = encoder_.getPosition() ;
        }

        return ret;
    }
    
    /// \brief Return the velocity of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the velocity of the motor in ticks per second
    public double getVelocity()  throws BadMotorRequestException, MotorRequestFailedException {
        double ret = 0.0 ;

        if (mtype_ == MotorType.kBrushed)
            throw new BadMotorRequestException(this, "brushed motor does not support velocity") ;

        if(simstate_ != null) {
            ret = simstate_.getVelocity() ;
        }
        else {
            ret = encoder_.getVelocity() ;
        }

        return ret;
    }

    /// \brief Return the acceleration of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the acceleration of the motor in ticks per second squared
    public double getAcceleration() throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "the SparkMaxMotorController does not support acceleration") ; 
    }

    /// \brief Reset the encoder values to zero    
    public void resetEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushed)
            throw new BadMotorRequestException(this, "brushed motor does not support resetEncoder()") ;

        if (simstate_ == null) {
            checkError("resetEncoder", () -> encoder_.setPosition(0.0)) ;
        }
        else {
            simstate_.setPosition(0);
        }
    }

     /// \brief Set the encoder to a specific value in ticks
     /// \param pos the new value for the encoder in ticks
     public void setPosition(double value) throws BadMotorRequestException, MotorRequestFailedException {
        if (mtype_ == MotorType.kBrushed)
            throw new BadMotorRequestException(this, "brushed motor does not support resetEncoder()") ;

        if (simstate_ == null) {
            checkError("setPosition", () -> encoder_.setPosition(value)) ;
        }
        else {
            simstate_.setPosition(value);
        }
    }        

    /// \brief Enable voltage compensation for the given motor
    /// \param enabled if true voltage compensation is enabled
    /// \param nominal if enabled is true, this is the nominal voltage for compensation
    public void enableVoltageCompensation(boolean enabled, double nominal) throws BadMotorRequestException, MotorRequestFailedException {
        if (simstate_ == null) {
            ctrl_.enableVoltageCompensation(nominal);
        }
    }


    /// \brief Return the closed loop target
    /// \returns  the closed loop target
    public double getClosedLoopTarget() throws BadMotorRequestException, MotorRequestFailedException {
        return target_ ;
    }

    /// \brief Return the closed loop error
    /// \returns  the closed loop error
    public double getClosedLoopError() throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "the SparkMax does not support returning closed loop error") ;
    }    

    public double getVoltage() throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "the SparkMax does not support returning voltage") ;
    }        
} ;
