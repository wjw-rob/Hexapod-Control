 
#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

class ControlFrame{
public:
	ControlFrame(CtrlComponents *ctrlComp);
	~ControlFrame(){
		delete _FSMController;
	}
	void run();
	CtrlComponents *_ctrlComp;
// private:
	FSM* _FSMController;
	// CtrlComponents *_ctrlComp;
};

#endif  //CONTROLFRAME_H