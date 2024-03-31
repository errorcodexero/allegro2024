package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.intake_shooter.IntakeManualShootAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class JustShootAutoMode extends AllegroGameAutoMode {

    public JustShootAutoMode(AutoController ctrl, String pos) throws Exception {
        super(ctrl, "JustShoot") ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        IntakeManualShootAction action = new IntakeManualShootAction(robot.getIntakeShooter(), pos, true) ;
        addSubActionPair(robot.getIntakeShooter(), action, true);
    }
}
