#include "scene_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

SceneData LoadScene(const std::string& jsonPath) {
    std::ifstream f(jsonPath);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scene file: " + jsonPath);

    std::stringstream buf;
    buf << f.rdbuf();
    auto js = nlohmann::json::parse(buf.str());

    SceneData scene;
    const auto& obj = js["objects"][0];

    int nVerts = obj["vertPos"].size();
    scene.vertices.resize(nVerts);
    for (int i = 0; i < nVerts; ++i) {
        auto& p = obj["vertPos"][i];
        scene.vertices[i] = Vector3f((float)p["x"], (float)p["y"], (float)p["z"]);
    }

    auto& tris = obj["triangles"];
    scene.indices.resize(tris.size());
    for (size_t i = 0; i < tris.size(); ++i)
        scene.indices[i] = (uint32_t)(int)tris[i];

    return scene;
}
