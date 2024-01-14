package org.xero1425.swervelib;

import org.xero1425.base.motors.MotorFactory;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.PidType;
import org.xero1425.misc.ISettingsSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.IMotorController;

public class SwerveModule {
    private double ticksToMeters_ ;
    private double ticksToDegrees_ ;
    private IMotorController steer_ ;
    private IMotorController drive_ ;
    private CANcoder absolute_encoder_ ;
    private CANcoderConfiguration absolute_cfg_ ;

    public SwerveModule(MotorFactory factory, ISettingsSupplier settings, String name, String id) throws Exception {
        steer_ = factory.createMotor(name + "-steer", id + "-steer");
        steer_ = factory.createMotor(name + "-drive", id + "-drive");

        int encoderId = settings.get(name + "-encoder:canid").getInteger() ;
        absolute_encoder_ = new CANcoder(encoderId) ;
        absolute_cfg_ = new CANcoderConfiguration();

        double offset = settings.get(name + "-encoder:offset").getDouble() ;

        absolute_cfg_.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1 ;
        absolute_cfg_.MagnetSensor.MagnetOffset = offset;
        absolute_cfg_.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive ;
        checkError("SwerveModule constructor()", absolute_encoder_.getConfigurator().apply(absolute_cfg_));

        ticksToMeters_ = 1.0 ;
    }

    private void checkError(String msg, StatusCode err) throws Exception {
        if (err != StatusCode.OK) {
            throw new Exception(msg) ;
        }
    }

    public double getDriveVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        return drive_.getVelocity() * ticksToMeters_ ;
    }

    public double getDistance() throws BadMotorRequestException, MotorRequestFailedException {
        return drive_.getPosition() * ticksToMeters_ ;
    }

    public double getSteerAngle() throws BadMotorRequestException, MotorRequestFailedException {
        return steer_.getPosition() * ticksToDegrees_ ;
    }

    public void set(double velocity, double angle) throws BadMotorRequestException, MotorRequestFailedException {
        steer_.set(PidType.Position, angle);
        drive_.set(PidType.Velocity, velocity) ;
    }

    public void synchronizeEncoders() {

    }
}
