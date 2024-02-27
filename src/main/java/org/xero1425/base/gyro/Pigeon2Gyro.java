package org.xero1425.base.gyro;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon2Gyro implements XeroGyro {
    static final private int kApplyTries = 5 ;

    // The instance of the NavX Gyro
    private Pigeon2 gyro_ ;
    private Pigeon2Configuration cfg_ ;

    /// \brief Create the Gyro class using the default NavX Port on the MXP bus
    public Pigeon2Gyro(String bus, int canid) throws Exception {
        gyro_ = new Pigeon2(canid, bus) ;
        cfg_ = new Pigeon2Configuration() ;

        checkError("setCofiguration", () -> gyro_.getConfigurator().apply(cfg_)) ;
    }

    @Override
    public double getRate() {
        return gyro_.getRate() ;
    }

    public Pigeon2 getPigeon2() {
        return gyro_ ;
    }

    /// \brief Returns true if the NavX is connected
    /// \returns true if the NavX is connected, othewise false
    public boolean isConnected() {
        return true ;
    }

    /// \brief Returns the current effective YAW angle for the NavX.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective YAW angle for the NavX
    public double getYaw() {
        return gyro_.getYaw().getValue() ;
    }

    public double getPitch() {
        return gyro_.getPitch().getValue() ;
    }

    public double getRoll() {
        return gyro_.getRoll().getValue() ;
    }

    /// \brief Returns the total angle for the NavX
    /// \returns the total angle for the NavX    
    public double getAngle() {
        return gyro_.getAccumGyroZ().getValue() ;
    }

    private void checkError(String msg, Supplier<StatusCode> toApply) throws Exception {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = kApplyTries ;
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            throw new Exception(msg + ":configuration of Pigeon 2 gyro failed")  ;
        }
    }    
}

