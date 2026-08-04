#include "sim_worker.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "PhysicsManager.h"

SimWorker::SimWorker(const std::string& sceneJson, float timestep)
    : timestep_(timestep)
{
    pm_ = std::make_unique<PhysicsManager>(sceneJson);
    pm_->Start();
    numDoF_ = pm_->GetInitialState().x.size();
}

SimWorker::~SimWorker() {
    Stop();
}

void SimWorker::Start() {
    if (running_.load())
        return;
    running_ = true;
    thread_ = std::thread(&SimWorker::Run, this);
}

void SimWorker::Stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

std::vector<Vector3f> SimWorker::GetVertices() {
    std::lock_guard<std::mutex> lock(vertexMutex_);
    return vertices_;
}

void SimWorker::SetParam(Eigen::VectorXd params, const std::string& settings) {
    pm_->SetParam(params, settings);
}

void SimWorker::Run() {
    using namespace std::chrono;
    float simTime = 0;

    while (running_) {
        auto t1 = high_resolution_clock::now();

        pm_->UpdatePhysics(simTime, timestep_);

        // Copy vertex data under lock
        {
            std::lock_guard<std::mutex> lock(vertexMutex_);
            int count;
            Vector3f* v = pm_->GetVertices(0, &count);
            if (count > 0) {
                vertices_.resize(count);
                std::copy(v, v + count, vertices_.begin());
            }
        }

        simTime += timestep_;
        started_ = true;

        auto t2 = high_resolution_clock::now();
        double elapsed = duration<double, std::milli>(t2 - t1).count();
        double target = (double)timestep_ * 1000.0;
        // always sleep at least 1 ms so the CPU core isn't spinning
        auto sleepFor = std::max(1, (int)(target - elapsed));
        std::this_thread::sleep_for(milliseconds(sleepFor));
    }
}
