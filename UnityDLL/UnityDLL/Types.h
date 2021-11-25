#pragma once

#include <mutex>

#pragma pack(push, 1) /*Force compiler to remove 4 or 8 byte memory alignment for all structs below, i.e., pack all variables without unused spaces. */
template<typename T>
struct Vector3 {
	Vector3() :x(0), y(0), z(0) {}
	Vector3(T _x, T _y, T _z) :x(_x), y(_y), z(_z) {}
	T& operator[](int i) {
		T* ptr = &x;
		return ptr[i];
	}
	const T& operator[](int i) const {
		const T* ptr = &x;
		return ptr[i];
	}
	T x, y, z;
};

typedef Vector3<float>   Vector3f;
typedef Vector3<double>  Vector3d;

#pragma pack(pop) /*Remove last #pragma, i.e., disable packing -> enable compiler optimization for memory alignment.*/

#pragma pack(push, 1) /*Force compiler to remove 4 or 8 byte memory alignment for all structs below, i.e., pack all variables without unused spaces. */
struct Edge {
	Edge() :a(-1), b(-1), other(-1) {}
	Edge(int _a, int _b) :a(_a), b(_b), other(-1) {}
	Edge(int _a, int _b, int _other) :a(_a), b(_b), other(_other) {}

	int a, b, other;
};
#pragma pack(pop) /*Remove last #pragma, i.e., disable packing -> enable compiler optimization for memory alignment.*/

class MyCounter {
public:
	void IncreaseCounter() {
		std::lock_guard<std::mutex> lock(counterMutex);
		counter++;
	}

	int GetCounter() {
		std::lock_guard<std::mutex> lock(counterMutex);
		return counter;
	}
private:
	int counter = 0;

	std::mutex counterMutex;
};
