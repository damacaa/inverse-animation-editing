#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "Types.h"

struct SceneData {
    std::vector<Vector3f> vertices;
    std::vector<uint32_t> indices;
    uint32_t objectId = 0;
};

SceneData LoadScene(const std::string& jsonPath);
