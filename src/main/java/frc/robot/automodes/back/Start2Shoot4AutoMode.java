package frc.robot.automodes.back;

import org.xero1425.base.actions.Action;
import org.xero1425.base.controllers.AutoController;

import frc.robot.automodes.AllegroGameAutoMode;
import frc.robot.automodes.actions.Start2Shoot4Action;
import frc.robot.automodes.actions.Start2Shoot4ActionFast;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot4AutoMode extends AllegroGameAutoMode {

    public Start2Shoot4AutoMode(AutoController ctrl, boolean mirror, double mvalue, boolean fast) throws Exception {
        super(ctrl, "Center Four Note" + (fast ? "-fast" : "-slow")) ;

        Action action ;
        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        if (fast) {
            action = new Start2Shoot4ActionFast(robot, mirror, mvalue) ;
        }
        else {
            action = new Start2Shoot4Action(robot, mirror, mvalue) ;
        }
        addSubActionPair(robot, action, true);
    }    
}
