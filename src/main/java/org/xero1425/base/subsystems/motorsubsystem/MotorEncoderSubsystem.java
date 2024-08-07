package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

/// \file

/// \brief A subsystem that includes a single motor, or group of motors mechanically coupled and an encoder for closed loop control
public class MotorEncoderSubsystem extends MotorSubsystem
{
    // The encoder attached to the motor output
    private XeroEncoder encoder_ ;

    // If true, the measured output is angular
    private boolean angular_ ;

    private double max_value_ ;
    private double min_value_ ;

    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angular) throws Exception {
        this(parent, name, angular, 2) ;
    }

    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angular, int samples) throws Exception {
        super(parent, name) ;

        angular_ = angular ;

        getMotorController().enableVoltageCompensation(true, 11.0);

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angular, getMotorController()) ;

        if (isSettingDefined("props:maxpos"))
            max_value_ = getSettingsValue("props:maxpos").getDouble() ;
        else
            max_value_ = Double.POSITIVE_INFINITY ;

        if (isSettingDefined("props:minpos"))
            min_value_ = getSettingsValue("props:minpos").getDouble() ;
        else
            min_value_ = Double.NEGATIVE_INFINITY ;

        if (Double.isFinite(max_value_)) {
            double v = encoder_.mapPhysicalToMotor(max_value_) ;
            getMotorController().enableSoftForwardLimit(v);
        }

        if (Double.isFinite(min_value_)) {
            double v = encoder_.mapPhysicalToMotor(min_value_) ;            
            getMotorController().enableSoftReverseLimit(v);
        }
    }

    public XeroEncoder getEncoder() {
        return encoder_ ;
    }

    public String[] getUnits() {
        return encoder_.getUnits();
    }

    public double getMaxPos() {
        return max_value_ ;
    }

    public double getMinPos() {
        return min_value_ ;
    }

    /// \brief Returns true if the motor has a hardware PID loop in the motor controller
    /// \returns true if the motor has a hardware PID loop in the motor controller
    public boolean hasHWPID(XeroPidType type) {
        boolean ret = false ;

        try {
            ret = getMotorController().hasPID(type) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret;
    }

    /// \brief Returns true if measuring an angular quantity
    /// \returns true if measuring an angular quantity
    public boolean isAngular() {
        return angular_ ;
    }

    /// \brief Returns the position of the motor output, as measured by the speedometer
    /// \returns the position of the motor output, as measured by the speedometer
    public double getPosition() {
        double ret = 0.0 ;

        try {
            ret = encoder_.mapMotorToPhysical(getMotorController().getPosition());
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("getPosition() threw an exception") ;
            logger.add("subsystem name", getName()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }

        return ret;
    }

    /// \brief Returns the velocity of the motor output, as measured by the speedometer
    /// \returns the velocity of the motor output, as measured by the speedometer
    public double getVelocity() {
        double ret = 0.0 ;

        try {
            ret = getMotorController().getVelocity() ;
            ret = encoder_.mapMotorToPhysical(ret) ;
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("getVelocity() threw an exception") ;
            logger.add("subsystem name", getName()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }
        return ret ;
    }

    public double getAcceleration() {
        double ret = 0.0 ;

        try {
            if (getMotorController().hasAcceleration()) {
                ret = getMotorController().getAcceleration() ;
                ret = encoder_.mapMotorToPhysical(ret) ;
            }
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("getVelocity() threw an exception") ;
            logger.add("subsystem name", getName()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }

        return ret ;
    }

    /// \brief Calibrates the motor encoder with the given position
    /// \param pos the current real world position of the motor output
    public void calibrate(double pos) {
        encoder_.calibrate(pos) ;
    }

    @Override
    public void postHWInit() throws Exception {
        super.postHWInit();
        encoder_.reset() ;
    }

    /// \brief Reset the motor and attacd encoder.  This will reset the encoder value to
    /// zero and set the motor power to off.
    public void reset() {
        super.reset() ;
    }

    /// \brief Returns the encoder raw count
    /// \returns the encoder raw count
    public double getEncoderRawCount() {
        return encoder_.getRawCount() ;
    }

    public double getTotalCurrent() {
        double total = 0.0 ;

        try {
            int [] channels = getMotorController().getPDPChannels() ;

            for(int i = 0 ; i < channels.length ; i++) {
                total += getRobot().getCurrent(channels[i]);
            }
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception thrown in getTotalCurrent - " + ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());

            total = -1.0 ;
        }

        return total ;
    }

    /// \brief Called once per robot loop by the Xero Framework to compute the position, velocity, and
    /// acceleration of the motor in real world units.  It also displays the position and velocity on the
    /// SmartDashboard if verbose output is enabled for this subsystem.
    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        double pos = encoder_.getPosition() ;

        putDashboard(getName() + "-pos", DisplayType.Verbose, pos);
    }
}
