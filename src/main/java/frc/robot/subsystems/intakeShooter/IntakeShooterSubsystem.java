package frc.robot.subsystems.intakeShooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class IntakeShooterSubsystem extends Subsystem{
    private MotorEncoderSubsystem updown_;
    private MotorEncoderSubsystem feeder_;
    private MotorEncoderSubsystem tilt_;
    private MotorEncoderSubsystem shooter1_;
    private MotorEncoderSubsystem shooter2_;

    public IntakeShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, "intakeShooter");

        updown_ = new MotorEncoderSubsystem(this,"intake-updown", true);
        addChild(updown_);

        feeder_ = new MotorEncoderSubsystem(this,"intake-feeder", false);
        addChild(feeder_);

        tilt_ = new MotorEncoderSubsystem(this,"intake-tilt", true);
        addChild(tilt_);

        shooter1_ = new MotorEncoderSubsystem(this,"intake-shooter1", false);
        addChild(shooter1_);

        shooter2_ = new MotorEncoderSubsystem(this,"intake-shooter2", false);
        addChild(shooter2_);

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
}
