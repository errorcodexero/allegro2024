package org.xero1425.base.subsystems.swerve;

import org.xero1425.base.motors.MotorFactory;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.XeroNeutralMode;
import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.misc.ISettingsSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.IMotorController;

public class SwerveModule {
    private String name_ ;
    private double ticksToMeters_ ;
    private double ticksToRadians_ ;
    private IMotorController steer_ ;
    private IMotorController drive_ ;
    private CANcoder absolute_encoder_ ;
    private CANcoderConfiguration absolute_cfg_ ;
    private double target_angle_ ;

    public SwerveModule(MotorFactory factory, ISettingsSupplier settings, SwerveModuleConfig cfg, ShuffleboardContainer container, String name, String id) throws Exception {
        //
        // Create and initialize the steering motor
        //
        name_ = name ;
        steer_ = factory.createMotor(name + "-steer", id + ":motors:steer");
        steer_.setInverted(cfg.steer_inverted);
        steer_.setNeutralMode(XeroNeutralMode.Brake);
        steer_.resetEncoder();
        ticksToRadians_ = 2.0 * Math.PI / steer_.ticksPerRevolution() * cfg.steer_reduction ;
        double p = settings.get(id + ":motors:steer:p").getDouble() ;
        double i = settings.get(id + ":motors:steer:i").getDouble() ;
        double d = settings.get(id + ":motors:steer:d").getDouble() ;
        steer_.setPID(XeroPidType.Position, p, i, d, 0, 0, 0, 0, 0.1);
        steer_.setPositionImportant(true);
        target_angle_ = 0.0 ;

        //
        // Create and initialize the drive motor
        //
        drive_ = factory.createMotor(name + "-drive", id + ":motors:drive");
        drive_.setInverted(cfg.drive_inverted);
        drive_.resetEncoder();
        ticksToMeters_ = Math.PI * cfg.wheel_diameter * cfg.drive_reduction / drive_.ticksPerRevolution() ;
        drive_.setVelocityImportant(true);
        drive_.setPositionImportant(true);

        //
        // Create and initialize the absolute cancoder encoder
        //
        int encoderId = settings.get(id + ":encoder:canid").getInteger() ;
        String bus = settings.get(id + ":encoder:bus").getString() ;
        double offset = settings.get(id + ":encoder:offset").getDouble() ;        

        absolute_encoder_ = new CANcoder(encoderId, bus);
        absolute_cfg_ = new CANcoderConfiguration();

        absolute_cfg_.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1 ;
        absolute_cfg_.MagnetSensor.MagnetOffset = offset / 360.0 ;
        absolute_cfg_.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive ;
        checkError("SwerveModule constructor()", absolute_encoder_.getConfigurator().apply(absolute_cfg_));

        addDashBoardEntries(container);
    }

    public void setNeutralMode(IMotorController.XeroNeutralMode mode) {
        try {
            steer_.setNeutralMode(mode);
            drive_.setNeutralMode(mode);
        }
        catch(Exception ex) {
        }
    }

    public String getName() {
        return name_ ;
    }

    private void addDashBoardEntries(ShuffleboardContainer container) {
        container.addNumber("Velocity", () -> {
            double ret = 0 ;
            try {
                ret = getDriveVelocity();
            } catch (Exception e) {
                ret = Double.MAX_VALUE ;
            }
            return ret;
        });

        container.addNumber("Current Angle", () -> {
            double ret = 0 ;
            try {
                ret = Math.toDegrees(getStateAngle());
            } catch (Exception e) {
                ret = Double.MAX_VALUE ;
            }
            return ret;
        }) ;

        container.addNumber("Steer Angle Raw", () -> {
            double ret = 0 ;
            try {
                ret = steer_.getPosition() ;
            } catch (Exception e) {
                ret = Double.MAX_VALUE ;
            }
            return ret;
        }) ;        

        container.addNumber("Target Angle", () -> Math.toDegrees(getTargetAngle())) ;
        container.addNumber("Absolute Encoder Degrees", () -> Math.toDegrees(getAbsoluteEncoderAngle())) ;
        container.addNumber("Absolute Encoder Raw", () -> absolute_encoder_.getAbsolutePosition().getValue()) ;
    }

    public double getDriveVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        return drive_.getVelocity() * ticksToMeters_ ;
    }

    public double getDistance() throws BadMotorRequestException, MotorRequestFailedException {
        return drive_.getPosition() * ticksToMeters_ ;
    }

    public double getStateAngle() throws BadMotorRequestException, MotorRequestFailedException {
        double ticks = steer_.getPosition() ;
        double angle = ticks * ticksToRadians_ ;
        angle %= 2.0 * Math.PI ;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI ;
        }

        return angle ;
    }    

    public double getTargetAngle() {
        return target_angle_ ;
    }

    public void set(double voltage, double angle) throws BadMotorRequestException, MotorRequestFailedException {
        angle %= 2.0 * Math.PI ;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI ;
        }

        double diff = angle - getTargetAngle() ;
        if (diff >= Math.PI) {
            angle -= 2.0 * Math.PI ;
        }
        else if (diff < -Math.PI) {
            angle += 2.0 * Math.PI ;
        }
        diff = angle - getTargetAngle() ;

        if (diff > Math.PI / 2.0 || diff < -Math.PI / 2.0) {
            angle += Math.PI ;
            voltage *= -1.0 ;
        }

        angle %= 2.0 * Math.PI ;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI ;
        }

        target_angle_ = angle ;
        steer_.set(XeroPidType.Position, angle / ticksToRadians_);
        drive_.set(XeroPidType.Voltage, voltage) ;
    }

    /// \brief Return the absolute encoder angle in radians
    /// \returns the absolute encoder angle in radians
    public double getAbsoluteEncoderAngle() {
        double v = absolute_encoder_.getAbsolutePosition().getValue() ;
        return v * 2.0 * Math.PI ;
    }


    public void synchronizeEncoders() throws BadMotorRequestException, MotorRequestFailedException {
        double encoder = getAbsoluteEncoderAngle() ;
        double value = (encoder/ ticksToRadians_);
        steer_.setPosition(value) ;
    }

    private void checkError(String msg, StatusCode err) throws Exception {
        if (err != StatusCode.OK) {
            throw new Exception(msg) ;
        }
    }    
}
