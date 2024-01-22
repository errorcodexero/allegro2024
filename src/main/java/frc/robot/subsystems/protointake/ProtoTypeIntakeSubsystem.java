package frc.robot.subsystems.protointake;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;

public class ProtoTypeIntakeSubsystem extends Subsystem {
    private boolean running_ ;
    private MotorEncoderSubsystem m1_ ;         // Lower Intake
    private MotorEncoderSubsystem m2_ ;         // Upper Intake
    private MotorEncoderSubsystem m3_ ;         // Feeder Roller
    private double p1_ = 0.3375 ;
    private double p2_ = -0.2 ;
    private double p3_ = 0.3 ;

    public ProtoTypeIntakeSubsystem(Subsystem parent) throws Exception {
        super(parent, "ProtoTypeIntake") ;

        m1_ = new MotorEncoderSubsystem(this, "m1", false) ;
        addChild(m1_) ;

        m2_ = new MotorEncoderSubsystem(this, "m2", false) ;
        addChild(m2_) ;

        m3_ = new MotorEncoderSubsystem(this, "m3", false) ;
        addChild(m3_) ;

        running_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        SwerveDriveGamepad gp = (SwerveDriveGamepad)getRobot().getRobotSubsystem().getOI().getGamePad() ;
        if (gp.isRTriggerPressed() && !running_) {
            m1_.setPower(p1_);
            m2_.setPower(p2_);
            m3_.setPower(p3_) ;
            running_ = true ;
        }
        else if (!gp.isRTriggerPressed() && running_) {
            m1_.setPower(0.0) ;
            m2_.setPower(0.0) ;    
            m3_.setPower(0.0) ;        
            running_ = false ;
        }
    }
}
