package org.xero1425.base.motors;

import java.util.List;

public interface IMotorController {
    /// \brief the type of PID control to run on the motor controller
    public enum PidType {
        Voltage,                    ///< No PID type has been set
        Position,                   ///< Position PID control
        Velocity,                   ///< Velocity PID control
        MotionMagic,                ///< Motion Magic
    }

    /// \brief the mode to apply when zero power is applied to the motor
    public enum XeroNeutralMode { 
        Coast,              ///< Coast mode
        Brake               ///< Brake mode
    } ;

    /// \brief Returns the user assigned name of the motor controller
    /// \returns the user assigned name of the motor controller
    public String getName() ;

    /// \brief Returns the CAN ID of the motor
    /// \returns the CAN ID of the motor
    public int getCanID() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns the name of the CAN bus.  An empty string means roborio
    /// \return the name of the CAN bus.  An empty string means roborio
    public String getBus() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return a human readable string giving the physical motor controller type (e.g. TalonFX, SparkMaxBrushless, etc.)
    /// \returns a human readable string giving the physical motor controller type
    public String getType() throws BadMotorRequestException, MotorRequestFailedException ;    

    /// \brief Return list of faults detected by the motor controller
    /// \returns list of faults detected by the motor controller
    public List<String> getFaults() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \Brief Return the native motor controller for this controller.  This is used like
    /// TalonFX fx = (TalonFX)ctrl.getNativeController() ;
    /// This will return null if the motor controller is not a talon FX, or it will return the controller if it is
    /// \returns the native motor controller for the motor
    public Object getNativeController() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller
    public String getFirmwareVersion() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position    
    public boolean hasEncoder() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return the number of encoder ticks per revolution for this motor.  If this motor does not
    /// have an encoder, a BadMotorEquestException is thrown.
    /// \returns the number of encoder ticks per revolution for this motor.
    public int ticksPerRevolution() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not
    public void setInverted(boolean inverted)  throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted
    public boolean isInverted() throws BadMotorRequestException , MotorRequestFailedException ;

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public void setNeutralMode(XeroNeutralMode coast) throws BadMotorRequestException, MotorRequestFailedException ;  
    
    /// \brief Get the neutral mode for the motor
    /// \returns the neutral mode for the motor
    public XeroNeutralMode getNeutralMode() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given
    public void setCurrentLimit(double limit) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns the current limit for the current supplied to the motor
    /// \returns the current limit for the current supplied to the motor
    public double getCurrentLimit() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief set the PDP channel for this motor
    /// \param channel the channel on the PDP supplying this motor
    public void setPDPChannel(int channel) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief get the PDP channels associated with a given motor
    /// For most motors there will be a single entry.  For a motor group, there will be an entry
    /// per motor in the group.
    /// \returns the PDP channels assocaited with a given motor
    public int[] getPDPChannels() throws BadMotorRequestException, MotorRequestFailedException ; 

    /// \brief set the deadband for the motor.  If any power value is assigned to the motor that is less
    /// than this value, zero is assumed.
    /// \param value the deadband value for this motor
    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException ;    

    /// \brief Get the deadband value for the motor
    /// \returns the deadband value for the motor
    public double getNeutralDeadband() throws BadMotorRequestException, MotorRequestFailedException ; 

    /// \brief If value is true, the motor controller will consider position data as important and update
    /// the data a quickly as possible.
    public void setPositionImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException ;
    
    /// \brief If value is true, the motor controller will consider velocity data as important and update
    /// the data a quickly as possible.
    public void setVelocityImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException ;
    
    /// \brief If value is true, the motor controller will consider acceleration data as important and update
    /// the data a quickly as possible.
    public void setAccelerationImportant(boolean value) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Have the current motor follow another motor.
    /// \param leader if true, the leader is inverted versus normal operation
    /// \param invert if true, follow the other motor but with the power inverted.    
    public void follow(IMotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return the leader for the current motor.  If the current motor is not following, return null.
    /// \returns the leader for the current motor.
    public IMotorController getLeader() throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID(PidType type) throws BadMotorRequestException , MotorRequestFailedException ;

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
    public void setPID(PidType type, double p, double i, double d, double v, double a, double g, double s, double outmax) throws BadMotorRequestException , MotorRequestFailedException ; 

    /// \brief Set the parameters for motion magic
    /// \param v the max velocity for the motion
    /// \param a the max acceleration for the motion
    /// \param j the max jerk for the motion
    public void setMotionMagicParams(double v, double a, double j) throws BadMotorRequestException, MotorRequestFailedException;

    /// \brief Set the motor target.  What the target is depends on the mode.
    /// \param type the type of target to set (position PID, velocity PID, MotionMagic, or percent power)
    /// \param target the target value, depends on the type
    public void set(PidType type, double target) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Reset the encoder values to zero
    public void resetEncoder() throws BadMotorRequestException, MotorRequestFailedException ;

     /// \brief Set the encoder to a specific value in ticks
     /// \param pos the new value for the encoder in ticks
     public void setPosition(int pos) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns the position of the motor in motor units from the motor controller.  If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the position of the motor in encoder ticks
    public double getPosition() throws BadMotorRequestException, MotorRequestFailedException ;    
    
    /// \brief Return the velocity of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the velocity of the motor in ticks per second
    public double getVelocity()  throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return the acceleration of the motor if there is PID control in the motor controller.   If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the acceleration of the motor in ticks per second squared
    public double getAcceleration()  throws BadMotorRequestException, MotorRequestFailedException ;


}
