#include "AutoLineCross.h"
#include "AutoForward.h"
AutoLineCross::AutoLineCross() {

	AddSequential(new AutoForward(10, .4));


}
