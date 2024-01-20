package frc.robot.subsystems.protointake;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;

public class ProtoTypeIntakeSubsystem extends Subsystem {
    private boolean running_ ;
    private MotorEncoderSubsystem m1_ ;
    private MotorEncoderSubsystem m2_ ;
    private double p1_ = 0.8 ;
    private double p2_ = 0.8 ;

    public ProtoTypeIntakeSubsystem(Subsystem parent) throws Exception {
        super(parent, "ProtoTypeIntake") ;

        m1_ = new MotorEncoderSubsystem(this, "m1", false) ;
        addChild(m1_) ;

        m1_ = new MotorEncoderSubsystem(this, "m2", false) ;
        addChild(m2_) ;

        running_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        SwerveDriveGamepad gp = (SwerveDriveGamepad)getRobot().getRobotSubsystem().getOI().getGamePad() ;
        if (gp.isRTriggerPressed() && !running_) {
            m1_.setPower(p1_);
            m2_.setPower(p2_);
        }
        else if (!gp.isRTriggerPressed() && running_) {
            m1_.setPower(0.0) ;
            m2_.setPower(0.0) ;            
        }
    }
}
