package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.EncoderMapper;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeShooterSubsystem extends Subsystem{
    private MotorEncoderSubsystem updown_;
    private MotorEncoderSubsystem feeder_;
    private MotorEncoderSubsystem tilt_;
    private MotorEncoderSubsystem shooter1_;
    private MotorEncoderSubsystem shooter2_;
    private DigitalInput noteSensor_; 
    private boolean noteSensorInverted_;
    private AnalogInput absoluteEncoder_;
    private EncoderMapper encoderMapper_;
    private boolean is_note_present_;
    private boolean invert_note_sensor_;
    private double angle_;

    public IntakeShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, "intake-shooter");

        updown_ = new MotorEncoderSubsystem(this,"intake-updown", false);
        addChild(updown_);

        feeder_ = new MotorEncoderSubsystem(this,"intake-feeder", false);
        addChild(feeder_);

        tilt_ = new MotorEncoderSubsystem(this,"intake-tilt", false);
        addChild(tilt_);

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

        invert_note_sensor_ = getSettingsValue("hw:sensor:inverted").getBoolean();

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

    public boolean isNotePresent() {
        return is_note_present_;
    }

    public double getAngle() {
        return angle_;
    }

    @Override
    public void computeMyState() {
        is_note_present_ = noteSensor_.get() ^ noteSensorInverted_;
        
        double eval = absoluteEncoder_.getVoltage();
        angle_ = encoderMapper_.toRobot(eval);
    }
}
