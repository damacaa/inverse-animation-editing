// Portable C API bridge between the physics engine and Unity (P/Invoke).
// Replaces the old Windows-only dllmain.cpp: no DllMain, no __declspec,
// works as .dll on Windows, .so on Linux and .dylib on macOS.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include "PhysicsManager.h"
#include "Types.h"

#include <nlohmann/json.hpp>

#if defined(_WIN32)
#define DPE_API __declspec(dllexport)
#else
#define DPE_API __attribute__((visibility("default")))
#endif

namespace {
bool initialized = false;

float simulationTime = 0;
float delta = 0.01f;

MyCounter* counter = nullptr;
MyCounter* threadCounter = nullptr;

int nUpdates = 0;
std::mutex physManMutex;

std::thread simThread;

bool running = false;
bool updating = false;

PhysicsManager* physicsManager = nullptr;

void UpdateInternal() {
	if (updating)
		return;
	updating = true;
	simulationTime += delta;

	/*This may take a long time, depending on your simulation.*/
	if (physicsManager)
		physicsManager->UpdatePhysics(simulationTime, delta);

	nUpdates++;
	updating = false;
}

/*Function executed using a separate thread*/
void StartFunction() {
	using namespace std::chrono;

	/*Sleep for 1 s*/
	std::this_thread::sleep_for(milliseconds(1000));

	while (running) {
		auto t1 = high_resolution_clock::now();
		UpdateInternal();
		auto t2 = high_resolution_clock::now();

		duration<double, std::milli> ms_double = t2 - t1;
		double duration = ms_double.count();

		int sleepTime = (int)std::ceil(std::max(0.0, (1000.0 * (double)delta) - duration));

		std::this_thread::sleep_for(milliseconds(sleepTime));
	}
}
} // namespace

extern "C" {

	DPE_API void Update() {
		UpdateInternal();
	}

	DPE_API void Initialize(int integrationMethod, float timeStep, float tolerance) {
		if (!initialized) {
			physicsManager = new PhysicsManager((PhysicsManager::Integration)integrationMethod, tolerance);
			delta = timeStep;

			counter = new MyCounter();
			threadCounter = new MyCounter();

			initialized = true;
		}
	}

	DPE_API void InitializeFromJSON(char* info) {
		if (!initialized) {
			nlohmann::json js = nlohmann::json::parse(info);
			delta = js["delta"];

			physicsManager = new PhysicsManager(std::string(info));

			counter = new MyCounter();
			threadCounter = new MyCounter();

			initialized = true;
		}
	}

	DPE_API void StartSimulation(bool multithreading) {
		if (running)
			return;

		running = true;
		if (multithreading) {
			simThread = std::thread(StartFunction); /*Create new thread that is executing StartFunction*/
			simThread.detach(); /*Allow thread to run independently*/
		}
	}

	DPE_API int AddObject(Vector3f* vertPos, float vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, float damping)
	{
		return -1;
	}

	DPE_API void AddFixer(Vector3f position, Vector3f scale) {
		if (physicsManager)
			physicsManager->AddFixer(position, scale);
	}

	DPE_API void Destroy() {
		if (initialized) {

			running = false; /*Stop main loop of thread function*/

			while (updating)
			{
				std::this_thread::yield();
			}

			if (simThread.joinable()) {
				simThread.join(); /*Wait for thread to finish*/
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			std::lock_guard<std::mutex> lock(physManMutex);
			delete counter;
			delete threadCounter;
			delete physicsManager;
			physicsManager = nullptr;

			counter = nullptr;
			threadCounter = nullptr;
			/*Since the variables below are global variables, they exist as long the library is loaded. In case of Unity,
			the library remains loaded until we close Unity. Therefore we need to reset these variables in order to
			allow Unity to correctly run the simulation again.*/

			initialized = false;
			updating = false;

			nUpdates = 0;
		}
	}

	DPE_API void IncreaseCounter() {
		if (counter) {
			counter->IncreaseCounter();
		}
	}

	DPE_API int GetCounter() {
		if (counter) {
			return counter->GetCounter();
		}
		return 0;
	}

	DPE_API int GetThreadCounter() {
		if (threadCounter) {
			return threadCounter->GetCounter();
		}
		return 0;
	}

	DPE_API Vector3f* GetVertices(int id, int* count) {

		if (!running) {
			*count = 0;
			return nullptr;
		}

		return physicsManager->GetVertices(id, count);
	}
}
