#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include <Eigen/Core>

#include <nlohmann/json.hpp>

#include "renderer.hpp"
#include "scene_loader.hpp"
#include "sim_worker.hpp"

struct OrbitCamera {
    float yaw = 0.0f, pitch = -0.3f, distance = 3.0f;
    Eigen::Vector3f target = Eigen::Vector3f::Zero();

    Eigen::Matrix4f View() const {
        Eigen::Vector3f eye(
            target.x() + distance * cosf(pitch) * sinf(yaw),
            target.y() + distance * sinf(pitch),
            target.z() + distance * cosf(pitch) * cosf(yaw));

        Eigen::Vector3f up(0, 1, 0);
        Eigen::Vector3f f = (target - eye).normalized();
        Eigen::Vector3f s = f.cross(up).normalized();
        Eigen::Vector3f u = s.cross(f);

        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        view(0,0) =  s.x(); view(0,1) =  s.y(); view(0,2) =  s.z(); view(0,3) = -s.dot(eye);
        view(1,0) =  u.x(); view(1,1) =  u.y(); view(1,2) =  u.z(); view(1,3) = -u.dot(eye);
        view(2,0) = -f.x(); view(2,1) = -f.y(); view(2,2) = -f.z(); view(2,3) =  f.dot(eye);
        return view;
    }

    void Orbit(float dx, float dy) {
        yaw   -= dx * 0.01f;
        pitch += dy * 0.01f;
        if (pitch >  1.5f) pitch =  1.5f;
        if (pitch < -1.5f) pitch = -1.5f;
    }

    void Zoom(float dz) {
        distance += dz;
        if (distance < 0.2f) distance = 0.2f;
    }
};

static Eigen::Matrix4f Projection(float fov, float aspect, float near, float far) {
    // Vulkan: depth [0,1], Y flipped, RH clip space
    float t = tanf(fov * 0.5f);
    Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
    p(0,0) = 1.0f / (aspect * t);
    p(1,1) = -1.0f / t;
    p(2,2) = far / (near - far);
    p(2,3) = far * near / (near - far);
    p(3,2) = -1.0f;
    return p;
}

int main(int argc, char** argv) {
    std::string scenePath;
    if (argc < 2) {
        printf("Scene file path: ");
        std::getline(std::cin, scenePath);
        if (scenePath.empty()) return 1;
    } else {
        scenePath = argv[1];
    }

    // Load scene
    SceneData scene = LoadScene(scenePath);
    fprintf(stderr, "Loaded scene: %zu vertices, %zu indices\n",
           scene.vertices.size(), scene.indices.size());

    // Read scene JSON for the simulation
    std::ifstream f(scenePath);
    std::stringstream buf;
    buf << f.rdbuf();
    std::string sceneJson = buf.str();
    float delta = (float)nlohmann::json::parse(sceneJson)["delta"];

    // GLFW window
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Inverse Animation Viewer", nullptr, nullptr);
    glfwSetScrollCallback(window, [](GLFWwindow* w, double, double yoff) {
        static_cast<OrbitCamera*>(glfwGetWindowUserPointer(w))->Zoom((float)-yoff * 0.5f);
    });

    // Vulkan renderer
    VulkanRenderer renderer;
    if (!renderer.Init(window)) {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // Upload initial mesh
    int nVerts = (int)scene.vertices.size();
    Eigen::MatrixXf positions(nVerts, 3);
    for (int i = 0; i < nVerts; ++i) {
        positions(i, 0) = scene.vertices[i].x;
        positions(i, 1) = scene.vertices[i].y;
        positions(i, 2) = scene.vertices[i].z;
    }
    renderer.UploadMesh(positions, scene.indices);

    // Simulation worker thread
    SimWorker sim(sceneJson, delta);
    sim.Start();

    // Camera — target at mesh center
    OrbitCamera camera;
    {
        Eigen::Vector3f center(0,0,0);
        for (int i = 0; i < nVerts; ++i)
            center += Eigen::Vector3f(scene.vertices[i].x, scene.vertices[i].y, scene.vertices[i].z);
        center /= (float)nVerts;
        camera.target = center;
    }
    camera.distance = 3.0f;
    camera.pitch = -0.3f;
    glfwSetWindowUserPointer(window, &camera);

    double lastMouseX = 0, lastMouseY = 0;
    bool mouseDown = false;

    auto lastFrame = std::chrono::high_resolution_clock::now();
    int frameCount = 0;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Camera input
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

        if (left && !mouseDown) { mouseDown = true; lastMouseX = mx; lastMouseY = my; }
        if (!left) mouseDown = false;
        if (mouseDown) {
            camera.Orbit((float)(mx - lastMouseX), (float)(my - lastMouseY));
            lastMouseX = mx; lastMouseY = my;
        }

        // Keyboard
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
            camera = OrbitCamera{};
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camera.Zoom(-0.05f);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camera.Zoom(0.05f);

        // Pull vertex data from simulation
        auto simVerts = sim.GetVertices();
        if (!simVerts.empty() && (int)simVerts.size() == nVerts) {
            Eigen::MatrixXf newPos(nVerts, 3);
            for (int i = 0; i < nVerts; ++i) {
                newPos(i, 0) = simVerts[i].x;
                newPos(i, 1) = simVerts[i].y;
                newPos(i, 2) = simVerts[i].z;
            }
            renderer.UpdateVertices(newPos);
        }

        // Draw
        float aspect = (float)renderer.GetExtent().width / (float)renderer.GetExtent().height;
        Eigen::Matrix4f proj = Projection(0.8f, aspect, 0.1f, 100.0f);
        Eigen::Matrix4f mvp = proj * camera.View();

        renderer.BeginFrame();
        renderer.EndFrame(mvp, Eigen::Vector3f(0.7f, 0.8f, 1.0f));

        // FPS counter
        frameCount++;
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFrame).count();
        if (elapsed >= 1.0) {
            char title[128];
            snprintf(title, sizeof(title), "Inverse Animation Viewer [%.0f fps]", frameCount / elapsed);
            glfwSetWindowTitle(window, title);
            frameCount = 0;
            lastFrame = now;
        }
    }

    sim.Stop();
    renderer.Shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
