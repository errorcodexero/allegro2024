package org.xero1425.base.motors ;

/// \file

/// \brief This class is an abstract base class that defines a contract that all supported motors
/// must meet.  There are specific derived classes for Talon SRX, Victor SPX, Talon Fx, SparkMax, and Romi
/// motor controller.  There are also derived classes where a group of mechanically connected motors
/// are represented by a single MotorController derived object (MotorGroupController).
public abstract class MotorController implements IMotorController
{
    // The name of the motor
    private String name_ ;

    // The PDP channel for the motor
    private int pdp_channel_ ;

    /// \brief Property name for property used for motor power in a simulation
    public final static String SimPowerParamName = "Power" ;

    /// \brief Property name for property used for the encoder value in a simulation    
    public final static String SimEncoderParamName = "Encoder" ;

    /// \brief Property name for property used for the stores ticks in a simulation (i.e. motor contains encoder)
    public final static String SimEncoderStoresTicksParamName = "StoresTicks" ;

    /// \brief Property name for property used for to indicate if the power should be inverted in a simulation
    public final static String SimInvertedParamName = "Inverted" ;

     /// \brief Property name for property used for to indicate the neutral mode in a simulation  
    public final static String SimNeutralParamName = "Neutral" ;

    /// \brief Create a new motor controller
    /// \param name the name of the motor controller
    protected MotorController(String name) {
        name_ = name ;
        pdp_channel_ = -1 ;
    }

    public void setPDPChannel(int channel) throws BadMotorRequestException, MotorRequestFailedException {
        pdp_channel_ = channel ;
    }

    public int[] getPDPChannels() throws BadMotorRequestException, MotorRequestFailedException {
        int [] ret  = new int[1] ;
        ret[0] = pdp_channel_ ;
        return ret;
    }

    /// \brief Returns the name of the motor controller
    /// \returns the name of the motor controller
    public String getName() {
        return name_  ;
    }
}
