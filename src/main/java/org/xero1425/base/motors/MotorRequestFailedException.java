package org.xero1425.base.motors;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

/// \file

/// \brief This class is an exception that is thrown when a motor request fails with an error code
/// from the underlying motor controller
public class MotorRequestFailedException extends Exception {
    // The motor controller that had a failed request        
    private MotorController motor_ ;
    private StatusCode ctre_code_ ;
    private REVLibError rev_code_ ;

    /// \brief The UID for the class for serialization
    static final long serialVersionUID = 42 ;

    /// \brief This method creates a new BadMotorRequestException
    /// \param motor the motor that causeed the exception
    /// \param msg a string describing the cause of the exception
    /// \param code the error code that occurred
    public MotorRequestFailedException(MotorController motor, String msg, REVLibError code) {
        super("motor '" + motor.getName() + "' - " + code.toString() + ":" + msg) ;

        motor_ = motor ;
        ctre_code_ = StatusCode.OK ;
        rev_code_ = code ;
    }

    /// \brief This method creates a new BadMotorRequestException
    /// \param motor the motor that causeed the exception
    /// \param msg a string describing the cause of the exception
    /// \param code the error code that occurred
    public MotorRequestFailedException(MotorController motor, String msg, StatusCode code) {
        super("motor '" + motor.getName() + "' - " + code.toString() + ":" + msg) ;

        motor_ = motor ;
        ctre_code_ = code ;
        rev_code_ = REVLibError.kOk ;
    }

    /// \brief Return the motor that caused the exception
    /// \returns the motor that caused the exception
    public MotorController getMotor() {
        return motor_ ;
    }

    public boolean isCTRE() {
        return ctre_code_ != StatusCode.OK ;
    }

    public StatusCode getCTRE() {
        return ctre_code_ ;
    }

    public boolean isREV() {
        return rev_code_ != REVLibError.kOk ;
    }

    public REVLibError getREV() {
        return rev_code_ ;
    }
}
