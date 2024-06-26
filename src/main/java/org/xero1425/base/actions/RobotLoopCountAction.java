package org.xero1425.base.actions;

import org.xero1425.base.XeroRobot;

public class RobotLoopCountAction extends Action {
    private int count_ ;
    private int current_ ;

    public RobotLoopCountAction(XeroRobot robot, int count) {
        super(robot) ;

        count_ = count ;
    }    

    @Override
    public void start() {
        current_ = 0 ;
        checkDone() ;
    }

    @Override
    public void run() {
        current_++ ;
        checkDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "RobotLoopCountAction " + count_ ;
    }

    private void checkDone() {
        if (current_ >= count_)
            setDone() ;
    }
}
