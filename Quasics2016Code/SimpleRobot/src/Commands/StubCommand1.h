// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef STUB_COMMAND_1_H
#define STUB_COMMAND_1_H

#include "CloneableCommand.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class StubCommand1: public CloneableCommand {
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    StubCommand1();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    virtual void Initialize();
    virtual void Execute();
    virtual bool IsFinished();
    virtual void End();
    virtual void Interrupted();

    virtual std::unique_ptr<Command> clone() const {
        return std::unique_ptr<Command>(new StubCommand1());
    }

private:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
//  int timer = 0;
//  int test = 0;
    int autoStage;
    bool isFinished;

//  enum octojohn{
//      kTurnRight, kForward, kStop
//  };
//  octojohn otacon;
};

#endif
