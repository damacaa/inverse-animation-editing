#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

struct QueueFamilyIndices {
    std::optional<uint32_t> graphics;
    std::optional<uint32_t> present;

    bool Complete() const { return graphics.has_value() && present.has_value(); }
};

struct SwapchainSupport {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

class VulkanRenderer {
public:
    bool Init(GLFWwindow* window);
    void UploadMesh(const Eigen::MatrixXf& positions, const std::vector<uint32_t>& indices);
    void UpdateVertices(const Eigen::MatrixXf& positions);
    void BeginFrame();
    void EndFrame(const Eigen::Matrix4f& viewProj, const Eigen::Vector3f& color);
    void WaitIdle();
    void Shutdown();

    VkExtent2D GetExtent() const { return swapchainExtent_; }
    GLFWwindow* GetWindow() const { return window_; }

private:
    void CreateInstance();
    void CreateSurface();
    void PickPhysicalDevice();
    void CreateLogicalDevice();
    void CreateSwapchain();
    void CreateImageViews();
    void CreateRenderPass();
    void CreatePipeline();
    void CreateFramebuffers();
    void CreateCommandPool();
    void CreateVertexIndexBuffers();
    void CreateSyncObjects();
    void RecreateSwapchain();
    void CleanupSwapchain();

    VkShaderModule CreateShaderModule(const std::vector<char>& code);

    GLFWwindow* window_ = nullptr;

    VkInstance instance_ = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue graphicsQueue_ = VK_NULL_HANDLE;
    VkQueue presentQueue_ = VK_NULL_HANDLE;

    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    std::vector<VkImage> swapchainImages_;
    VkFormat swapchainImageFormat_;
    VkExtent2D swapchainExtent_{};
    std::vector<VkImageView> swapchainImageViews_;

    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline graphicsPipeline_ = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> framebuffers_;

    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;
    uint32_t imageIndex_ = 0;

    VkBuffer vertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory vertexBufferMemory_ = VK_NULL_HANDLE;
    VkBuffer indexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory indexBufferMemory_ = VK_NULL_HANDLE;
    uint32_t indexCount_ = 0;

    VkSemaphore imageAvailableSemaphore_ = VK_NULL_HANDLE;
    VkSemaphore renderFinishedSemaphore_ = VK_NULL_HANDLE;
    VkFence inFlightFence_ = VK_NULL_HANDLE;

    VkDescriptorPool imguiDescPool_ = VK_NULL_HANDLE;
    bool framebufferResized_ = false;

    std::vector<char> vertShaderCode_;
    std::vector<char> fragShaderCode_;
};
