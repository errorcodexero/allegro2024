package frc.robot.subsystems.intakeshooter;

import java.util.function.Supplier;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeShooterSubsystem extends Subsystem {

    private EncoderMapper mapper_ ;
    private CANcoder cancoder_ ;
    private MotorEncoderSubsystem spinner_feeder_ ;
    private MotorEncoderSubsystem updown_ ; 
    private MotorEncoderSubsystem shooter1_ ;
    private MotorEncoderSubsystem shooter2_ ;
    private MotorEncoderSubsystem tilt_ ;
    private DigitalInput note_sensor_ ;
    private boolean note_present_ ;
    private boolean note_inverted_ ;
    private boolean tilt_inited_ ;

    public IntakeShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, "intake-shooter") ;

        note_present_ = false ;
        tilt_inited_ = false ;

        //
        // Spins the wheels at the entry to the intake
        //
        spinner_feeder_ = new MotorEncoderSubsystem(this, "spinner-feeder", false) ;
        addChild(spinner_feeder_) ;

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
        shooter1_ = new MotorEncoderSubsystem(this, "shooter1", false) ;
        addChild(shooter1_) ;
        shooter2_ = new MotorEncoderSubsystem(this, "shooter2", false) ;
        addChild(shooter2_) ;        

        //
        // The sensor for detecting the note
        //
        int channel = getSettingsValue("hw:note-sensor:io").getInteger() ;
        note_sensor_ = new DigitalInput(channel) ;
        note_inverted_ = getSettingsValue("hw:note-sensor:inverted").getBoolean() ;

        int canid = getSettingsValue("hw:tilt-sensor:io").getInteger() ;
        cancoder_ = new CANcoder(canid) ;

        CANcoderConfiguration cfg = new CANcoderConfiguration() ;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive ;
        cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1 ;
        checkError("IntakeShooterSubsystem()", () -> cancoder_.getConfigurator().apply(cfg));
        
        double robotv = getSettingsValue("hw:tilt-sensor:robotv").getDouble() ;
        double encoderv = getSettingsValue("hw:tilt-sensor:encoderv").getDouble() ;
        mapper_ = new EncoderMapper(360.0, 0.0, 1.0, 0.0);
        mapper_.calibrate(robotv, encoderv) ;
    }

    private void checkError(String msg, Supplier<StatusCode> toApply) throws Exception {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = 5 ;
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            throw new Exception("cannot initialize CANcoder - " + msg);
        }
    }    

    public boolean isNotePresent() {
        return note_present_ ;
    }

    public MotorEncoderSubsystem spinner_feeder() {
        return spinner_feeder_ ;
    }

    public MotorEncoderSubsystem updown() {
        return updown_;
    }

    public MotorEncoderSubsystem tilt() {
        return tilt_ ;
    }    

    public MotorEncoderSubsystem shooter1() {
        return shooter1_ ;
    }

    public MotorEncoderSubsystem shooter2() {
        return shooter2_ ;
    }

    @Override
    public void computeMyState() {
        if (!tilt_inited_) {
            copyAngleFromCancoderToMotor();
        }

        note_present_ = note_sensor_.get() ^ note_inverted_ ;
        putDashboard("note", DisplayType.Always, note_present_);
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("spinner-feeder-velocity")) {
            v = new SettingsValue(spinner_feeder_.getVelocity()) ;
        }
        else if (name.equals("shooter1-velocity")) {
            v = new SettingsValue(shooter1_.getVelocity());
        }
        else if (name.equals("shooter2-velocity")) {
            v = new SettingsValue(shooter2_.getVelocity());
        }        
        else if (name.equals("updown-position")) {
            v = new SettingsValue(updown_.getPosition());
        }
        else if (name.equals("tilt")) {
            v = new SettingsValue(tilt_.getPosition());
        }

        return v ;
    }

    @Override
    public void postHWInit() {
        copyAngleFromCancoderToMotor() ;
    }

    private void copyAngleFromCancoderToMotor() {
        StatusSignal<Double> sig = cancoder_.getPosition().waitForUpdate(0.1) ;
        if (sig.getStatus().isOK()) {
            try {
                double robotv = mapper_.toRobot(sig.getValue()) ;
                tilt().getMotorController().setPosition(robotv);
                tilt_inited_ = true ;
            }
            catch(Exception ex) {
            }
        }
    }
}
