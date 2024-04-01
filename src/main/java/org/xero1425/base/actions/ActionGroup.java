package org.xero1425.base.actions;

import java.util.List;

import org.xero1425.base.XeroRobot;

/// \file

/// \brief This class is the base class for any action that executes multiple other actions.
public abstract class ActionGroup extends Action {
    /// \brief create a new ActionGroup object
    /// \param logger the message logger for the robot
    public ActionGroup(XeroRobot robot) {
        super(robot) ;
    }

    /// \brief return all of the child actions that are contained by this ActionGroup
    /// \param output a list to contain all of the child actions
    public abstract void getAllChildren(List<Action> output) ;
}
