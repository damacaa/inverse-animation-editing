// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include "types.h"
#include <math.h>   
#include <vector>

#include "PhysicsManager.h"

bool initialized = false;
bool newFixers = false;

MyCounter* counter = 0;

int nUpdates = 0;
std::mutex vertexMutex;
std::mutex vertexMutex2;

MyCounter* threadCounter = 0;
bool running = false;
std::thread myThread;
float simulationTime = 0;
float delta = 0.01f;

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

	/*Function executed using a separate thread*/
	void StartFunction() {
		while (running) {
			threadCounter->IncreaseCounter();

			std::this_thread::sleep_for(std::chrono::milliseconds(10)); /*Sleep for 10 ms*/
		}
	}

	__declspec(dllexport) void Initialize() {
		if (!initialized) {
			physicsManager = new PhysicsManager();

			counter = new MyCounter();
			threadCounter = new MyCounter();


			running = true;
			myThread = std::thread(&StartFunction); /*Create new thread that is executing StartFunction*/
			myThread.detach(); /*Allow thread to run independently*/

			initialized = true;
		}
	}

	__declspec(dllexport) int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass) {//, int* triangles, int* nTriangles) {
		std::lock_guard<std::mutex> lock(vertexMutex); /*Locks mutex and releases mutex once the guard is (implicitly) destroyed*/
		return physicsManager->AddObject(position, vertices, nVertices, triangles, nTriangles, stiffness, mass);
	}

	__declspec(dllexport) void AddFixer(Vector3f position, Vector3f scale) {
		std::lock_guard<std::mutex> lock(vertexMutex);
		physicsManager->AddFixer(position, scale);
	}

	__declspec(dllexport) void Destroy() {
		if (initialized) {

			running = false; /*Stop main loop of thread function*/
			if (myThread.joinable()) {
				myThread.join(); /*Wait for thread to finish*/
			}

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


			nUpdates = 0;

			running = false;
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

	__declspec(dllexport) void Update() {


		simulationTime += delta;

		std::lock_guard<std::mutex> lock(vertexMutex); /*Locks mutex and releases mutex once the guard is (implicitly) destroyed*/
		/*This may take a long time, depending on your simulation.*/
		physicsManager->Update(simulationTime, delta);

		nUpdates++;

	}

	__declspec(dllexport) Vector3f* GetVertices(int id, int* count) {

		/*Depending on how Update is being executed, vertexArray might being updated at the same time. To prevent race conditions, we have to wait
		for the lock of vertexArray and copy all data to a second array. Unity/C# will process the data in the second array. In the meantime
		the data in the first array can be updated.*/

		std::lock_guard<std::mutex> lock(vertexMutex);
		if (physicsManager->Updated) {
			std::lock_guard<std::mutex> lock(vertexMutex2);
			return physicsManager->GetVertices(id, count);
		}

		return physicsManager->GetVertices(id, count);
	}
#ifdef __cplusplus
}
#endif


