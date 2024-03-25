package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.controllers.AutoController;

import frc.robot.automodes.Start2Shoot4DynamicAction.FinishStrategy;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot4DynamicAutoMode extends AllegroGameAutoMode {
    public Start2Shoot4DynamicAutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "FourNote-Dynamic") ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Action action = new Start2Shoot4DynamicAction(robot, mirror, mvalue, FinishStrategy.NearSide) ;
        addSubActionPair(robot, action, true);
    }    
}
