#ifndef __ARMOR_TIMER
#define __ARMOR_TIMER

#include <chrono>

namespace armor {

struct TimeStats
{
	double lastProcessTime = 0.0f;  // ms
	double avgProcessTime  = 0.0f;  // ms
	double fps             = 0.0f;
	long   frameCount      = 0;
	double totalTime       = 0.0f;  // ms

	/** @param duration time in ms */
	void update(double duration)
	{
		lastProcessTime = duration;
		totalTime += duration;

		frameCount++;

		avgProcessTime = totalTime / frameCount;
		fps            = 1000.0f / avgProcessTime;
	}

	void reset()
	{
		lastProcessTime = 0.0f;
		avgProcessTime  = 0.0f;
		fps             = 0.0f;
		frameCount      = 0;
		totalTime       = 0.0f;
	}
};

// Simple RAII timer to measure the time of a scope
class SimpleTimer
{
  private:
	std::chrono::high_resolution_clock::time_point startTime;
	TimeStats&                                     stats;

  public:
	SimpleTimer(TimeStats& _stats) : stats(_stats)
	{
		startTime = std::chrono::high_resolution_clock::now();
	}

	~SimpleTimer()
	{
		using namespace std::chrono;

		auto end      = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(end - startTime).count();

		double processTime = static_cast<double>(duration / 1000.0f);  // ms
		stats.update(processTime);
	}
};

}  // namespace armor

#endif  // __ARMOR_TIMER