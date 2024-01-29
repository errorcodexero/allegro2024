package org.xero1425.base.subsystems.swerve;

import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.XeroNeutralMode;
import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.base.subsystems.Subsystem;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.IMotorController;

public class SwerveModule {
    private String name_ ;
    private Subsystem subsys_ ;
    private double ticksToMeters_ ;
    private double ticksToRadians_ ;
    private IMotorController steer_ ;
    private IMotorController drive_ ;
    private CANcoder absolute_encoder_ ;
    private CANcoderConfiguration absolute_cfg_ ;
    private double target_angle_ ;
    private boolean synchronized_ ;

    static String[] plot_values_ = new String[] { "a" };

    public SwerveModule(Subsystem subsys, SwerveModuleConfig cfg, ShuffleboardContainer container, String name, String id) throws Exception {
        //
        // Create and initialize the steering motor
        //
        name_ = name ;
        subsys_ = subsys ;
        synchronized_ = false ;

        steer_ = subsys_.getRobot().getMotorFactory().createMotor(name + "-steer", id + ":motors:steer");
        steer_.setInverted(cfg.steer_inverted);
        steer_.setNeutralMode(XeroNeutralMode.Brake);
        steer_.resetEncoder();
        steer_.enableVoltageCompensation(true, 11.0);
        ticksToRadians_ = 2.0 * Math.PI / steer_.ticksPerRevolution() * cfg.steer_reduction ;
        double p = subsys_.getRobot().getSettingsSupplier().get(id + ":motors:steer:p").getDouble() ;
        double i = subsys_.getRobot().getSettingsSupplier().get(id + ":motors:steer:i").getDouble() ;
        double d = subsys_.getRobot().getSettingsSupplier().get(id + ":motors:steer:d").getDouble() ;
        steer_.setPID(XeroPidType.Position, p, i, d, 0, 0, 0, 0, 0.1);
        steer_.setPositionImportant(IMotorController.ImportantType.High);
        steer_.setVelocityImportant(IMotorController.ImportantType.High);
        target_angle_ = 0.0 ;

        //
        // Create and initialize the drive motor
        //
        drive_ = subsys_.getRobot().getMotorFactory().createMotor(name + "-drive", id + ":motors:drive");
        drive_.setInverted(cfg.drive_inverted);
        drive_.resetEncoder();
        drive_.enableVoltageCompensation(true, 11.0);
        ticksToMeters_ = Math.PI * cfg.wheel_diameter * cfg.drive_reduction / drive_.ticksPerRevolution() ;
        drive_.setVelocityImportant(IMotorController.ImportantType.High);
        drive_.setPositionImportant(IMotorController.ImportantType.High);

        //
        // Create and initialize the absolute cancoder encoder
        //
        int encoderId = subsys_.getRobot().getSettingsSupplier().get(id + ":encoder:canid").getInteger() ;
        String bus = subsys_.getRobot().getSettingsSupplier().get(id + ":encoder:bus").getString() ;
        double offset ;       

        if (RobotBase.isSimulation()) {
            offset = 90.0 ;
        }
        else {
            offset = subsys_.getRobot().getSettingsSupplier().get(id + ":encoder:offset").getDouble() ;  
        }

        absolute_encoder_ = new CANcoder(encoderId, bus);
        absolute_cfg_ = new CANcoderConfiguration();

        absolute_cfg_.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1 ;
        absolute_cfg_.MagnetSensor.MagnetOffset = offset / 360.0 ;
        absolute_cfg_.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive ;
        checkError("SwerveModule constructor()", absolute_encoder_.getConfigurator().apply(absolute_cfg_));

        if (RobotBase.isSimulation()) {
            Timer.delay(0.1) ;
        }
        addDashBoardEntries(container);
    }

    public double ticks2Angle(double ticks) {
        double angle = ticks * ticksToRadians_ ;
        angle %= 2.0 * Math.PI ;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI ;
        }

        return angle ;        
    }

    public CANcoder getCANCoder() {
        return absolute_encoder_ ;
    }

    public boolean isEncoderSynchronized() {
        return synchronized_ ;
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

        container.addNumber("Target Angle", () -> Math.toDegrees(getTargetAngle())) ;
        container.addNumber("Absolute Encoder Degrees", () -> Math.toDegrees(getAbsoluteEncoderAngle())) ;
    }

    public double getDriveVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        return drive_.getVelocity() * ticksToMeters_ ;
    }

    public double getDistance() throws BadMotorRequestException, MotorRequestFailedException {
        return drive_.getPosition() * ticksToMeters_ ;
    }

    public double getStateAngle() throws BadMotorRequestException, MotorRequestFailedException {
        return ticks2Angle(steer_.getPosition());

    }    

    public double getTargetAngle() {
        return target_angle_ ;
    }

    public void set(double voltage, double angle) throws BadMotorRequestException, MotorRequestFailedException {

        angle %= 2.0 * Math.PI ;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI ;
        }    

        double diff = angle - getStateAngle() ;
        if (diff >= Math.PI) {
            angle -= 2.0 * Math.PI ;
        }
        else if (diff < -Math.PI) {
            angle += 2.0 * Math.PI ;
        }
        diff = angle - getStateAngle() ;          

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
        drive_.set(XeroPidType.Power, voltage) ;
    }

    /// \brief Return the absolute encoder angle in radians
    /// \returns the absolute encoder angle in radians
    public double getAbsoluteEncoderAngle() {
        double v = absolute_encoder_.getAbsolutePosition().getValue() ;
        return v * 2.0 * Math.PI ;
    }


    public void synchronizeEncoders(boolean force) throws BadMotorRequestException, MotorRequestFailedException {
        if (!synchronized_ || force) {
            StatusSignal<Double> val = absolute_encoder_.getPosition().waitForUpdate(0.2);
            if (val.getStatus() == StatusCode.OK) {
                double value = (val.getValue() * 2.0 * Math.PI) / ticksToRadians_;
                steer_.setPosition(value) ;
                synchronized_ = true ;
            }
        }
    }

    private void checkError(String msg, StatusCode err) throws Exception {
        if (err != StatusCode.OK) {
            throw new Exception(msg) ;
        }
    }    
}
