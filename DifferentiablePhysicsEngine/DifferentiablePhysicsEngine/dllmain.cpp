// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include "types.h"
#include <math.h>   
#include <vector>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

#include "DLLMAIN.H"
#include "PhysicsManager.h"


bool initialized = false;
bool newFixers = false;

float simulationTime = 0;
float delta = 0.01f;

MyCounter* counter = 0;

int nUpdates = 0;
std::mutex physManMutex;

MyCounter* threadCounter = 0;
std::thread myThread;

bool running = false;
bool updating = false;

PhysicsManager* physicsManager;

#ifdef __cplusplus
extern "C" {
#endif

	BOOL APIENTRY DllMain(HMODULE hModule,
		DWORD  ul_reason_for_call,
		LPVOID lpReserved
	)
	{
		switch (ul_reason_for_call)
		{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
		}
		return TRUE;
	}



	__declspec(dllexport) void Update() {
		if (updating)
			return;
		updating = true;
		simulationTime += delta;

		//!!!!
		/*This may take a long time, depending on your simulation.*/
		if (physicsManager)
			physicsManager->UpdatePhysics(simulationTime, delta);

		nUpdates++;
		updating = false;
	}

	/*Function executed using a separate thread*/
	void StartFunction() {
		/*Sleep for 10 ms*/
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		double elapsedTime = 0;

		while (running) {
			auto t1 = high_resolution_clock::now();
			Update();
			auto t2 = high_resolution_clock::now();

			duration<double, std::milli> ms_double = t2 - t1;
			double duration = ms_double.count();

			int sleepTime = ceil(max(0.0, (1000.0 * (double)delta) - duration));

			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			//std::this_thread::yield();
		}
	}

	__declspec(dllexport) void Initialize(int integrationMethod, float timeStep, float tolerance) {
		if (!initialized) {
			physicsManager = new PhysicsManager((PhysicsManager::Integration)integrationMethod, tolerance);
			delta = timeStep;

			counter = new MyCounter();
			threadCounter = new MyCounter();

			initialized = true;
		}
	}

	__declspec(dllexport) void InitializeFromJSON(char* info) {
		if (!initialized) {

			json js = json::parse(info);

			delta = js["delta"];

			physicsManager = new PhysicsManager(info);

			counter = new MyCounter();
			threadCounter = new MyCounter();

			initialized = true;
		}
	}

	__declspec(dllexport) void StartSimulation(bool multithreading) {
		if (running)
			return;

		running = true;
		if (multithreading) {
			myThread = std::thread(StartFunction); /*Create new thread that is executing StartFunction*/
			myThread.detach(); /*Allow thread to run independently*/
		}
	}

	__declspec(dllexport) int AddObject(Vector3f* vertPos, float vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, float damping)
	{
		return -1;
	}

	__declspec(dllexport) void AddFixer(Vector3f position, Vector3f scale) {
		if (physicsManager)
			physicsManager->AddFixer(position, scale);
	}

	__declspec(dllexport) void Destroy() {
		if (initialized) {

			running = false; /*Stop main loop of thread function*/

			while (updating)
			{
				std::this_thread::yield();
			}

			if (myThread.joinable()) {
				myThread.join(); /*Wait for thread to finish*/
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			std::lock_guard<std::mutex> lock(physManMutex);
			delete counter;
			delete threadCounter;
			delete physicsManager;
			physicsManager = nullptr;

			counter = 0;
			threadCounter = 0;
			/*Since the variables below are global variables, they exist as long the DLL is loaded. In case of Unity,
			the DLL remains loaded until we close Unity. Therefore we need to reset these variables in order to
			allow Unity to correctly run the simulation again.*/

			initialized = false;
			updating = false;

			nUpdates = 0;
		}
	}

	__declspec(dllexport) void IncreaseCounter() {
		if (counter) {
			counter->IncreaseCounter();
		}
	}

	__declspec(dllexport) int GetCounter() {
		if (counter) {
			return counter->GetCounter();
		}
		return 0;
	}

	__declspec(dllexport) int GetThreadCounter() {
		return threadCounter->GetCounter();
	}


	__declspec(dllexport) Vector3f* GetVertices(int id, int* count) {

		if (!running) {
			*count = 0;
			return nullptr;
		}

		return physicsManager->GetVertices(id, count);
	}
#ifdef __cplusplus
}
#endif


