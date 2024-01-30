package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeShooterSubsystem extends Subsystem{
    private MotorEncoderSubsystem updown_;
    private MotorEncoderSubsystem feeder_;
    private MotorEncoderSubsystem tilt_;
    private MotorEncoderSubsystem shooter1_;
    private MotorEncoderSubsystem shooter2_;
    private DigitalInput noteSensor_; 
    private boolean noteSensorInverted_;

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
        if (noteSensor_.get() ^ noteSensorInverted_) {
            return true;
        }
        else {
            return false;    
        }
    }
}
