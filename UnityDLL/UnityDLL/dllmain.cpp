// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include "types.h"
#include "Node.h"
#include <math.h>   
#include <vector>
#include "Object.h"
#include "Fixer.h"

bool initialized = false;
bool newFixers = false;

MyCounter* counter = 0;

std::vector<Object*> objects;
std::vector<Fixer*> fixers;

int nUpdates = 0;
std::mutex vertexMutex;
std::mutex vertexMutex2;

MyCounter* threadCounter = 0;
bool running = false;
std::thread myThread;
float simulationTime = 0;
float delta = 0.01f;


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
			counter = new MyCounter();
			threadCounter = new MyCounter();
			objects = std::vector<Object*>();

			running = true;
			myThread = std::thread(&StartFunction); /*Create new thread that is executing StartFunction*/
			myThread.detach(); /*Allow thread to run independently*/

			initialized = true;
		}
	}

	__declspec(dllexport) int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles) {//, int* triangles, int* nTriangles) {
		std::lock_guard<std::mutex> lock(vertexMutex); /*Locks mutex and releases mutex once the guard is (implicitly) destroyed*/

		Object* o = new Object(position, vertices, nVertices, triangles, nTriangles);
		o->id = (int)objects.size();
		objects.push_back(o);

		for (size_t i = 0; i < fixers.size(); i++)
		{
			o->FixNodes(fixers[i]);
		}

		return o->id;
	}

	__declspec(dllexport) void AddFixer(Vector3f position, Vector3f scale) {
		std::lock_guard<std::mutex> lock(vertexMutex);
		Fixer* f = new Fixer(position, scale);
		fixers.push_back(f);

		for (size_t i = 0; i < objects.size(); i++)
		{
			objects[i]->FixNodes(f);
		}
	}

	__declspec(dllexport) void Destroy() {
		if (initialized) {

			running = false; /*Stop main loop of thread function*/
			if (myThread.joinable()) {
				myThread.join(); /*Wait for thread to finish*/
			}

			delete counter;
			delete threadCounter;

			for (int i = 0; i < objects.size(); i++)
			{
				delete objects[i];
			}
			objects.clear();

			for (int i = 0; i < fixers.size(); i++)
			{
				delete fixers[i];
			}
			fixers.clear();


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
		for (int i = 0; i < objects.size(); i++)
		{
			objects[i]->Update(simulationTime, delta);
		}

		nUpdates++;

	}

	__declspec(dllexport) Vector3f* GetVertices(int id, int* count) {

		if (id >= objects.size())
			return new Vector3f((float)id, (float)objects.size(), 0.0f);

		if (count) {
			*count = objects[id]->nVertices;
		}

		/*Depending on how Update is being executed, vertexArray might being updated at the same time. To prevent race conditions, we have to wait
		for the lock of vertexArray and copy all data to a second array. Unity/C# will process the data in the second array. In the meantime
		the data in the first array can be updated.*/

		std::lock_guard<std::mutex> lock(vertexMutex);
		if (objects[id]->updated) {
			std::lock_guard<std::mutex> lock(vertexMutex2);
			return objects[id]->GetVertices();
		}

		return objects[id]->vertexArray2;
	}
#ifdef __cplusplus
}
#endif


