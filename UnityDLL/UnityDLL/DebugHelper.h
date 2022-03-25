#pragma once
#include <vector>
#include "Types.h"

class DebugHelper
{
private:
	std::vector<std::chrono::steady_clock::time_point> timePoints;
	std::vector<std::string> timePointNames;

	std::map<std::string, std::chrono::steady_clock::time_point> timePointMap;

	std::chrono::steady_clock::time_point lastTimepoint;
	std::string lastName;
	std::map<std::string, double> durations;
	std::map<std::string, int> occurances;

public:
	DebugHelper() {
		timePoints = std::vector<std::chrono::steady_clock::time_point>();
		timePointNames = std::vector<std::string>();
	};
	~DebugHelper() {};
	bool enabled = true;

	void RecordTime(std::string name);
	void Wait();
	void PrintTimes(std::string fileName = "times", std::string description = "");

	void PrintValue(std::string value, std::string fileName = "value");

	void PrintMat(SpMat mat, std::string name = "test");
};

