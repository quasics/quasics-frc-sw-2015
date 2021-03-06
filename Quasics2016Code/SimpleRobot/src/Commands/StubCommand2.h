// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef STUB_COMMAND_2_H
#define STUB_COMMAND_2_H

#include "CloneableCommand.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class StubCommand2: public CloneableCommand {
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    StubCommand2();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    virtual void Initialize();
    virtual void Execute();
    virtual bool IsFinished();
    virtual void End();
    virtual void Interrupted();

    virtual std::unique_ptr<Command> clone() const {
        return std::unique_ptr<Command>(new StubCommand2());
    }

private:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
    bool isFinished;
    int autoStage;

    //enum Autonomous{
    //kforward, kturnright, kturnleft, kstop
//  };
//  Autonomous firstattempt;
};

#endif
