#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Types.h"

class PhysicsManager;

class SimWorker {
public:
    SimWorker(const std::string& sceneJson, float timestep);
    ~SimWorker();

    void Start();
    void Stop();

    // Thread-safe snapshot of current vertex positions
    std::vector<Vector3f> GetVertices();

    void SetParam(Eigen::VectorXd params, const std::string& settings);
    int  GetNumDoFs() const { return numDoF_; }

private:
    void Run();

    std::unique_ptr<PhysicsManager> pm_;
    float timestep_;
    int numDoF_ = 0;

    std::atomic<bool> running_{false};
    std::atomic<bool> started_{false};
    std::thread thread_;
    std::mutex vertexMutex_;
    std::vector<Vector3f> vertices_;
};
