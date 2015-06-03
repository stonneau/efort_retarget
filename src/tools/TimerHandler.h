
#ifndef _CLASS_TIMERHANDLER
#define _CLASS_TIMERHANDLER

#include "Timer.h"

#include <vector>


struct TimerHandled_ABC
{
	typedef std::vector<TimerHandled_ABC*>	T_TimerHandled_ABC;
	typedef T_TimerHandled_ABC::iterator	IT_TimerHandled_ABC;

	 TimerHandled_ABC(){};
	~TimerHandled_ABC(){};
	virtual void Update(const Timer::t_time /*time*/, const Timer::t_time /*dt*/) = 0;
	virtual void Reset(){};
};

class TimerHandler {

public:
	  TimerHandler();
	 ~TimerHandler();

public:
	void	Start(Timer::t_time initTime = 0);
	void	Register(TimerHandled_ABC* timer);
	void    Update();
	void    Reset(Timer::t_time initTime = 0);

public:
	Timer& GetTimer();

private:
	Timer timer_;
	Timer::t_time lastUpdateTime_;
	TimerHandled_ABC::T_TimerHandled_ABC listeners_;
}; // Timer

#endif //_CLASS_TIMERHANDLER