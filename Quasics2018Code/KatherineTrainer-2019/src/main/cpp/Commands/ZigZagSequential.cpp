/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ZigZagSequential.h"
#include "Commands/RightTurnN.h"
#include "Commands/LeftTurnN.h"
#include "Commands/MoveForDuration.h"

ZigZagSequential::ZigZagSequential() {
  
  //Nurfadil: Created a Zig Zag
  AddSequential(new MoveForDuration(1.00));
  AddSequential(new LeftTurnN(1.00));
  AddSequential(new MoveForDuration(1.00));
  AddSequential(new RightTurnN(1.00));
  AddSequential(new MoveForDuration(1.00));
  AddSequential(new RightTurnN(1.00));
  AddSequential(new MoveForDuration(1.00));
  AddSequential(new LeftTurnN(1.00));
  AddSequential(new MoveForDuration(1.00)); 

}
