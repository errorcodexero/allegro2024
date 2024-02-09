package frc.robot.subsystems.superstructure;

import org.xero1425.base.subsystems.Subsystem;

import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;

public class SuperStructureSubsystem extends Subsystem {
    private IntakeShooterSubsystem is_ ;
    private AmpTrapSubsystem at_ ;

    public SuperStructureSubsystem(Subsystem parent) throws Exception {
        super(parent, "superstructure");

        is_ = new IntakeShooterSubsystem(this);
        addChild(is_);        

        at_ = new AmpTrapSubsystem(this);
        addChild(at_);             
    }

    public IntakeShooterSubsystem getIntakeShooter() {
        return is_;
    }

    public AmpTrapSubsystem getAmpTrap() {
        return at_;
    }
}
