package frc.robot.subsystems.elevatorpivot;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ElevatorPivotSubsystem extends Subsystem {
    private MotorEncoderSubsystem elevator_ ;
    private MotorEncoderSubsystem pivot_ ;
    private MotorEncoderSubsystem tilt_ ;
    private MotorEncoderSubsystem hooks_ ;
    private MotorEncoderSubsystem manipulator_ ;

    public ElevatorPivotSubsystem(Subsystem parent) throws Exception {
        super(parent, "elevator-pivot") ;

        elevator_ = new MotorEncoderSubsystem(this, "elevator", false) ;
        addChild( elevator_ ) ;

        pivot_ = new MotorEncoderSubsystem(this, "pivot", false) ;
        addChild(pivot_) ;

        tilt_ = new MotorEncoderSubsystem(this, "mtilt", false) ;
        addChild(tilt_) ;

        hooks_ = new MotorEncoderSubsystem(this, "hooks", false) ;
        addChild(hooks_) ;
        
        manipulator_ = new MotorEncoderSubsystem(this, "manipulator", false) ;
        addChild(manipulator_) ;
    }

    public MotorEncoderSubsystem getElevator() {
        return elevator_ ;
    }

    public MotorEncoderSubsystem getPivot() {
        return pivot_ ;
    }

    public MotorEncoderSubsystem getTilt() {
        return tilt_ ;
    }

    public MotorEncoderSubsystem getHooks() {
        return hooks_ ;
    }

    public MotorEncoderSubsystem getManipulator() {
        return manipulator_ ;
    }
}
