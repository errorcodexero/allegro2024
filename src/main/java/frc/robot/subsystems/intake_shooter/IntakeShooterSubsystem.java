package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.Speedometer;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeShooterSubsystem extends Subsystem {

    // moves the intake roller straight up (0 degrees) to flat to the ground (90 degrees), actual range 15-75 degrees   
    private MotorEncoderSubsystem updown_;

    // holds the note in place before it is shot
    private MotorEncoderSubsystem feeder_;
    
    // adjusts the angle of the shot, actual range is -45 to 45, resting posititon is 0 degrees
    private MotorEncoderSubsystem tilt_;
    
    // intakes the note into the robot
    private MotorEncoderSubsystem shooter1_;
    
    // shoots the note out of the robot
    private MotorEncoderSubsystem shooter2_;
    
    private DigitalInput noteSensor_; 
    private boolean noteSensorInverted_;
    private AnalogInput absoluteEncoder_;
    private EncoderMapper encoderMapper_;

    // This is true if the sensor is currently detecting a note
    private boolean is_note_present_;

    private double angle_;

    // This is true if a note has previously been detected and has not been
    // transferred to the amp/trap manipulator or been shot.  The note sensor may
    // not be detecting a note as th e sensor may be over the hole in the note.
    private boolean note_present_ ;

    public IntakeShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, "intake-shooter");

        note_present_ = false ;

        updown_ = new MotorEncoderSubsystem(this,"intake-updown", false);
        addChild(updown_);
        updown_.getMotorController().setNeutralDeadband(0.0);

        feeder_ = new MotorEncoderSubsystem(this,"intake-feeder", false);
        addChild(feeder_);

        tilt_ = new MotorEncoderSubsystem(this,"intake-tilt", false);
        addChild(tilt_);
        tilt_.getMotorController().setNeutralDeadband(0.0);

        shooter1_ = new MotorEncoderSubsystem(this,"intake-shooter1", false);
        addChild(shooter1_);

        shooter2_ = new MotorEncoderSubsystem(this,"intake-shooter2", false);
        addChild(shooter2_);

        int channel = getSettingsValue("hw:sensor:channel").getInteger();
        noteSensor_ = new DigitalInput(channel);

        channel = getSettingsValue("hw:encoder:channel").getInteger();
        absoluteEncoder_ = new AnalogInput(channel);

        double rmax = getSettingsValue("hw:encoder:rmax").getDouble();
        double rmin = getSettingsValue("hw:encoder:rmin").getDouble();
        double emax = getSettingsValue("hw:encoder:emax").getDouble();
        double emin = getSettingsValue("hw:encoder:emin").getDouble();
        double rcval = getSettingsValue("hw:encoder:rcval").getDouble();
        double ecval = getSettingsValue("hw:encoder:ecval").getDouble();

        encoderMapper_ = new EncoderMapper(rmax,rmin,emax,emin);
        encoderMapper_.calibrate(rcval, ecval);

        noteSensorInverted_ = getSettingsValue("hw:sensor:inverted").getBoolean();

        //
        // Get the value in degrees from the settings file
        //
        double initpos = updown_.getSettingsValue("targets:shoot").getDouble() ;

        //
        // Using the encoder in the motor/encoder subsystem, map the degrees to ticks
        //
        initpos = updown_.getEncoder().mapPhysicalToMotor(initpos) ;

        //
        // Set the motor position in ticks
        //
        updown_.getMotorController().setPosition(initpos);
    }

    public MotorEncoderSubsystem getUpDown() {
            return updown_ ;
    }

    public MotorEncoderSubsystem getFeeder() {
            return feeder_ ;
    }

    public MotorEncoderSubsystem getTilt() {
            return tilt_ ;
    }

    public MotorEncoderSubsystem getShooter1() {
            return shooter1_ ;
    }

    public MotorEncoderSubsystem getShooter2() {
            return shooter2_ ;
    }

    public boolean isNoteCurrentlyDetected() {
        return is_note_present_;
    }

    public boolean isHoldingNote() {
        return note_present_ ;
    }

    public void setHoldingNote(boolean b) {
        note_present_ = b ;
    }

    public double getAngle() {
        return angle_;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();
        
        is_note_present_ = noteSensor_.get() ^ noteSensorInverted_;
        
        double eval = absoluteEncoder_.getVoltage();
        angle_ = encoderMapper_.toRobot(eval);

        putDashboard("RawEnc", DisplayType.Verbose, eval);
        putDashboard("Angle", DisplayType.Verbose, angle_);
        putDashboard("note", DisplayType.Always, is_note_present_) ;
        putDashboard("hasnote", DisplayType.Always, note_present_);
    }

    @Override
    public void postHWInit() throws BadMotorRequestException, MotorRequestFailedException {
        updateMotorPosition();
    }
    
    private void updateMotorPosition() throws BadMotorRequestException, MotorRequestFailedException {
        double eval = absoluteEncoder_.getVoltage();
        angle_ = encoderMapper_.toRobot(eval);

        double m = tilt_.getEncoder().mapPhysicalToMotor(angle_) ;
        tilt_.getMotorController().setPosition(m);
    }
}
