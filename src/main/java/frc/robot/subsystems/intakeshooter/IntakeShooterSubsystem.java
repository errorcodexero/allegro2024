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

        //
        // Spins the wheels at the entry to the intake
        //
        spinner_ = new MotorEncoderSubsystem(this, "spinner", false) ;
        addChild(spinner_) ;

        //
        // Spins feeder wheels that receive the note from the spinner wheels
        //
        feeder_ = new MotorEncoderSubsystem(this, "feeder", false);
        addChild(feeder_) ;        

        //
        // Rotates the pivot arm that contains the intake/shooter assembly up and down
        //
        updown_ = new MotorEncoderSubsystem(this, "updown", false) ;
        addChild(updown_) ;

        //
        // Tilts the shooter/intake mechanism on top of the pivot arm
        //
        tilt_ = new MotorEncoderSubsystem(this, "tilt", false) ;
        addChild(tilt_);        

        //
        // Spins the shooter wheels used to shoot the game
        //
        shooter_ = new MotorEncoderSubsystem(this, "shooter", false) ;
        addChild(shooter_) ;
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
        else if (name.equals("shooter-velocity")) {
            v = new SettingsValue(shooter_.getVelocity());
        }
        else if (name.equals("feeder-velocity")) {
            v = new SettingsValue(feeder_.getVelocity());
        }
        else if (name.equals("updown-position")) {
            v = new SettingsValue(updown_.getPosition());
        }
        else if (name.equals("tilt")) {
            v = new SettingsValue(tilt_.getPosition());
        }

        return v ;
    }    
}
