#ifndef TIMER_H
#define TIMER_H

#ifdef _WIN32 

#include <windows.h>

//  -------------------------------------------------  //
/** \brief Classe de timer système
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class TimerPerf {

private:

	LARGE_INTEGER ticksPerSecond;
	LARGE_INTEGER tick;   // A point in time
	LARGE_INTEGER time_start;   // For converting tick into real time	
	float		  time;
	bool		  pause;

public:

	TimerPerf::TimerPerf() {
		pause = false;
		time = 0.0f ;
		// get the high resolution counter's accuracy
		QueryPerformanceFrequency(&ticksPerSecond);
	}

	virtual void restart() {
		// what time is it?
		QueryPerformanceCounter(&time_start);
	}

	//in milliseconds
	virtual float elapsedTime() {
		// what time is it?
		QueryPerformanceCounter(&tick);

		if ( ! pause )
		{		
			return time + ((float)(tick.QuadPart - time_start.QuadPart)*1000.0f/(float)ticksPerSecond.QuadPart);
		}

		//deduce time in milliseconds
		return time;
	}

	void wait()
	{ 
		pause = !pause; 
		if (pause)
		{
			QueryPerformanceCounter(&tick);
			time = time + ((float)(tick.QuadPart - time_start.QuadPart)*1000.0f/(float)ticksPerSecond.QuadPart);			
		}else 
		{
			QueryPerformanceCounter(&time_start);				
		}
	}

	void increase(float value)
	{
		time += value*1000.0f;
	}

};

#else

#include <cstddef>
#include <sys/time.h>                // for gettimeofday()

class TimerPerf {

private:
		
	timeval  time_start;   // For converting tick into real time	
	timeval  current_time;   // For converting tick into real time	
	float	 time;
	bool	 pause;

public:

	TimerPerf() {
		pause = false;
		time = 0.0f ;
		// get the high resolution counter's accuracy		
	}

	virtual void restart() {
		// what time is it?
		gettimeofday(&time_start, NULL);
	}

	//in milliseconds
	virtual float elapsedTime() {
		// what time is it?
		gettimeofday(&current_time, NULL);
		double elapsedTime ; 

		if ( ! pause )
		{		
			elapsedTime = (current_time.tv_sec - time_start.tv_sec) * 1000.0;      // sec to ms
			elapsedTime += (current_time.tv_usec - time_start.tv_usec) / 1000.0;   // us to ms
			return time + elapsedTime ;			
		}

		//deduce time in milliseconds
		return time;
	}

	void wait()
	{ 
		double elapsedTime ; 
		pause = !pause; 
		if (pause)
		{
			gettimeofday(&current_time, NULL);
			elapsedTime = (current_time.tv_sec - time_start.tv_sec) * 1000.0;      // sec to ms
			elapsedTime += (current_time.tv_usec - time_start.tv_usec) / 1000.0;   // us to ms
			time = time + 	elapsedTime ; 		
		}else 
		{
			gettimeofday(&time_start, NULL);		
		}
	}

	void increase(float value)
	{
		time += value*1000.0f;
	}

};


#endif

#endif

