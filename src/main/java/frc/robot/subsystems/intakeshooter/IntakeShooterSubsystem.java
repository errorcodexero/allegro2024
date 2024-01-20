package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class IntakeShooterSubsystem extends Subsystem {

    private MotorEncoderSubsystem spinner_ ;
    private MotorEncoderSubsystem updown_ ; 
    private MotorEncoderSubsystem feeder_ ;
    private MotorEncoderSubsystem shooter_ ;
    private MotorEncoderSubsystem tilt_ ;

    public IntakeShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, "intake-shooter") ;

        spinner_ = new MotorEncoderSubsystem(this, "spinner", false) ;
        addChild(spinner_) ;

        updown_ = new MotorEncoderSubsystem(this, "updown", false) ;
        addChild(updown_) ;

        feeder_ = new MotorEncoderSubsystem(this, "feeder", false);
        addChild(feeder_) ;

        shooter_ = new MotorEncoderSubsystem(this, "shooter", false) ;
        addChild(shooter_) ;

        tilt_ = new MotorEncoderSubsystem(this, "tilt", false) ;
        addChild(tilt_);
    }

    public MotorEncoderSubsystem spinner() {
        return spinner_ ;
    }

    public MotorEncoderSubsystem updown() {
        return updown_;
    }

    public MotorEncoderSubsystem feeder() {
        return feeder_ ;
    }

    public MotorEncoderSubsystem tilt() {
        return tilt_ ;
    }    

    public MotorEncoderSubsystem shooter() {
        return shooter_ ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("spinner-velocity")) {
            v = new SettingsValue(spinner_.getVelocity()) ;
        }

        return v ;
    }    
}
