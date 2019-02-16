#ifndef ELEVATOR_STAGE_H
#define ELEVATOR_STAGE_H

#include <frc/commands/Subsystem.h>
#include "ConfigurationFlags.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class ElevatorStage : public frc::Subsystem {
 public:
  ElevatorStage(const char* name);
  virtual ~ElevatorStage();

  void InitDefaultCommand() override;

  virtual bool atTop() = 0;
  virtual bool atBottom() = 0;
  virtual bool atPositionOne() = 0;
  virtual bool atPositionTwo() = 0;

  virtual void moveUp() = 0;
  virtual void moveDown() = 0;
  virtual void moveSlowlyUp() = 0;
  virtual void moveSlowlyDown() = 0;
  virtual void stop() = 0;
};

#endif  // ELEVATOR_STAGE_H
