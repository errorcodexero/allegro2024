package org.xero1425.base.motors ;

import java.util.List ;
import java.util.ArrayList ;

/// \file

/// \brief This class acts like a MotorController but in reality is a group of motors that are mechanically
/// coupled and setup such that all but the first motor follows the first motor.  For the most part, calls to this
/// object are referred to the first motor controller in the group.
public class MotorGroupController extends MotorController
{ 
    // The set of motors that are grouped
    private List<IMotorController> motors_ ;

    /// \brief Create a new MotorGroupController
    /// \param name the name of the group
    public MotorGroupController(String name) {
        super(name) ;
        motors_ = new ArrayList<IMotorController>() ;
    }

    public void setMotionMagicParams(double v, double a, double j) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setMotionMagicParams(v, a, j) ;
    }

    public int getCanID() throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "cannot get CAN ID for motor group") ;
    }

    public String getBus() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getBus() ;
    }

    public List<String> getFaults() throws BadMotorRequestException, MotorRequestFailedException {
        ArrayList<String> faults = new ArrayList<String>() ;

        for(IMotorController motor: motors_) {
            List<String> mfaults = motor.getFaults() ;

            for(String fault: mfaults) {
                String item = String.valueOf(motor.getCanID()) + ":" + fault ;
                faults.add(item) ;
            }
        }

        return faults ;
    }

    public Object getNativeController() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getNativeController() ;
    } 

    public int[] getPDPChannels() throws BadMotorRequestException, MotorRequestFailedException {
        int [] channels = new int[motors_.size()] ;
        for(int i = 0 ; i < motors_.size() ; i++) {
            int [] c = motors_.get(i).getPDPChannels() ;
            if (c.length != 1) {
                throw new BadMotorRequestException(this, "invalid MotorGroupController - internal error - includes something that is not a single motor") ;
            }
            channels[i] = c[0] ;
        }

        return channels ;
    }

    public IMotorController getLeader() {
        return null ;
    }

    /// \brief Return a specific motor from the group.  This is used for debugging issues.
    /// \param index the index of the motor to retreive
    /// \returns a specific motor controller from the group
    public IMotorController getMotor(int index) {
        return motors_.get(index) ;
    }

    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException {
        for(IMotorController ctrl : motors_) {
            ctrl.setNeutralDeadband(value);
        }
    }

    public double getNeutralDeadband() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getNeutralDeadband() ;
    }    

    public int ticksPerRevolution() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).ticksPerRevolution() ;
    }       

    public void follow(IMotorController ctrl, boolean leader, boolean inverted) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "a motor group cannot follow another motor") ;
    }

    /// \brief Add a new motor to the group
    /// \param ctrl the motor to add to the group
    /// \param leader if true, the leader is inverted
    /// \param inverted if true, the new motor is inverted with respect to the first motor
    public void addFollowerMotor(IMotorController ctrl, boolean leader, boolean inverted) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() > 0 && !motors_.get(0).getType().equals(ctrl.getType()))
            throw new BadMotorRequestException(this, "cannot add motor to group with existing motors unless the are the same type") ;

        motors_.add(ctrl) ;

        if (motors_.size() > 1)
            ctrl.follow(motors_.get(0), leader, inverted) ;
    }

    public void addLeaderMotor(IMotorController ctrl) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() > 0 && !motors_.get(0).getType().equals(ctrl.getType()))
            throw new BadMotorRequestException(this, "cannot add motor to group with existing motors unless the are the same type") ;

        motors_.add(ctrl) ;
    }    

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller    
    public boolean hasPID(PidType type) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).hasPID(type) ;
    }

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type    
    public void set(IMotorController.PidType type, double target) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).set(type, target);
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
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setPID(type, p, i, d, v, a, g, s, outmax) ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not     
    public void setInverted(boolean inverted)  throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "for groups the inverted state for motors can only be set via the settings file") ;
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted      
    public boolean isInverted() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
            
        return motors_.get(0).isInverted() ;
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor      
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        for(IMotorController ctrl : motors_)
            ctrl.setNeutralMode(mode);
    }

    public NeutralMode getNeutralMode() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getNeutralMode() ;
    } 

    /// \brief Set the current motor to follow another motor.  Note a MotorGroupController cannot follow anything else.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader is inverted
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.      
    public void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "a motor group cannot follow other motors") ;
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type        
    public String getType() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getType() ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position      
    public boolean hasEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).hasEncoder() ;
    }

    /// \brief Returns the position of the motor in motor units.
    public double getPosition() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getPosition() ;  
    }

    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getVelocity() ;  
    }    

    public double getAcceleration() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getAcceleration() ;  
    }      

    /// \brief Reset the encoder values to zero      
    public void resetEncoder() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).resetEncoder();
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given   
    public void setCurrentLimit(double limit) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
                    
        for(IMotorController ctrl : motors_)
            ctrl.setCurrentLimit(limit);
    }      

    public double getCurrentLimit() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getCurrentLimit() ;
    }      

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller       
    public String getFirmwareVersion() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        StringBuilder result = new StringBuilder() ;

        for(int i = 0 ; i < motors_.size() ; i++) {
            if (result.length() > 0)
                result.append(",") ;
            
            result.append(motors_.get(i).getFirmwareVersion()) ;
        }

        return result.toString() ;
    }

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setPositionImportant(value);
    }
    
    /// \brief If value is true, the motor controller will consider velocity data as important and update
    /// the data a quickly as possible.
    public void setVelocityImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setVelocityImportant(value);            
    }
    
    /// \brief If value is true, the motor controller will consider acceleration data as important and update
    /// the data a quickly as possible.
    public void setAccelerationImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setAccelerationImportant(value);             
    }
} ;
