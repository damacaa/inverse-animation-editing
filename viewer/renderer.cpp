#include "renderer.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <vector>

// ---------- helpers ----------

static std::vector<char> LoadBinaryFile(const std::string& path) {
    std::ifstream f(path, std::ios::ate | std::ios::binary);
    if (!f.is_open())
        throw std::runtime_error("Cannot open shader: " + path);
    size_t size = (size_t)f.tellg();
    std::vector<char> buf(size);
    f.seekg(0);
    f.read(buf.data(), size);
    f.close();
    return buf;
}

static VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT,
    VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* data, void*)
{
    std::cerr << "[vulkan] " << data->pMessage << std::endl;
    return VK_FALSE;
}

static void CreateBuffer(VkDevice device, VkPhysicalDevice physDev,
                          VkDeviceSize size, VkBufferUsageFlags usage,
                          VkMemoryPropertyFlags props,
                          VkBuffer& buffer, VkDeviceMemory& memory)
{
    VkBufferCreateInfo ci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    ci.size = size;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &ci, nullptr, &buffer) != VK_SUCCESS)
        throw std::runtime_error("Failed to create buffer");

    VkMemoryRequirements reqs;
    vkGetBufferMemoryRequirements(device, buffer, &reqs);

    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(physDev, &memProps);

    uint32_t typeIndex = 0;
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((reqs.memoryTypeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & props) == props) {
            typeIndex = i;
            break;
        }
    }

    VkMemoryAllocateInfo alloc{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    alloc.allocationSize = reqs.size;
    alloc.memoryTypeIndex = typeIndex;
    if (vkAllocateMemory(device, &alloc, nullptr, &memory) != VK_SUCCESS)
        throw std::runtime_error("Failed to allocate buffer memory");
    vkBindBufferMemory(device, buffer, memory, 0);
}

static VkCommandBuffer BeginSingleTimeCommands(VkDevice device, VkCommandPool pool) {
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandPool = pool;
    ai.commandBufferCount = 1;
    VkCommandBuffer cb;
    vkAllocateCommandBuffers(device, &ai, &cb);

    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cb, &bi);
    return cb;
}

static void EndSingleTimeCommands(VkDevice device, VkCommandPool pool,
                                   VkQueue queue, VkCommandBuffer cb) {
    vkEndCommandBuffer(cb);
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cb;
    vkQueueSubmit(queue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(queue);
    vkFreeCommandBuffers(device, pool, 1, &cb);
}

// ---------- VulkanRenderer ----------

bool VulkanRenderer::Init(GLFWwindow* window) {
    window_ = window;
    try {
        CreateInstance();
        CreateSurface();
        PickPhysicalDevice();
        CreateLogicalDevice();
        CreateSwapchain();
        CreateImageViews();
        CreateRenderPass();
        CreatePipeline();
        CreateFramebuffers();
        CreateCommandPool();
        CreateSyncObjects();
    } catch (const std::exception& e) {
        std::cerr << "Vulkan init failed: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void VulkanRenderer::CreateInstance() {
    VkApplicationInfo ai{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    ai.pApplicationName = "Inverse Animation Viewer";
    ai.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ci.pApplicationInfo = &ai;

    uint32_t glfwExtCount = 0;
    const char** glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCount);
    std::vector<const char*> exts(glfwExts, glfwExts + glfwExtCount);
    exts.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    ci.enabledExtensionCount = (uint32_t)exts.size();
    ci.ppEnabledExtensionNames = exts.data();

    const char* validationLayer = "VK_LAYER_KHRONOS_validation";
    uint32_t layerCount = 0;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
    std::vector<VkLayerProperties> availableLayers(layerCount);
    vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());
    bool hasValidation = false;
    for (auto& l : availableLayers)
        if (strcmp(l.layerName, validationLayer) == 0) hasValidation = true;

    if (hasValidation) {
        ci.enabledLayerCount = 1;
        ci.ppEnabledLayerNames = &validationLayer;
    }

    VkDebugUtilsMessengerCreateInfoEXT dbg{VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
    dbg.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                          VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    dbg.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                      VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                      VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    dbg.pfnUserCallback = DebugCallback;
    ci.pNext = hasValidation ? &dbg : nullptr;

    if (vkCreateInstance(&ci, nullptr, &instance_) != VK_SUCCESS)
        throw std::runtime_error("vkCreateInstance failed");

    if (hasValidation) {
        auto fn = (PFN_vkCreateDebugUtilsMessengerEXT)
            vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT");
        fn(instance_, &dbg, nullptr, &debugMessenger_);
    }
}

void VulkanRenderer::CreateSurface() {
    if (glfwCreateWindowSurface(instance_, window_, nullptr, &surface_) != VK_SUCCESS)
        throw std::runtime_error("glfwCreateWindowSurface failed");
}

void VulkanRenderer::PickPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance_, &count, nullptr);
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance_, &count, devices.data());

    for (auto dev : devices) {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(dev, &props);

        uint32_t qfCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &qfCount, nullptr);
        std::vector<VkQueueFamilyProperties> qfProps(qfCount);
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &qfCount, qfProps.data());

        QueueFamilyIndices idx;
        for (uint32_t i = 0; i < qfCount; ++i) {
            if (qfProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) idx.graphics = i;
            VkBool32 present = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface_, &present);
            if (present) idx.present = i;
            if (idx.Complete()) break;
        }

        if (!idx.Complete()) continue;

        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ||
            props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
            physicalDevice_ = dev;
            break;
        }
    }
    if (physicalDevice_ == VK_NULL_HANDLE && !devices.empty())
        physicalDevice_ = devices[0];
    if (physicalDevice_ == VK_NULL_HANDLE)
        throw std::runtime_error("No suitable GPU");
}

void VulkanRenderer::CreateLogicalDevice() {
    uint32_t qfCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &qfCount, nullptr);
    std::vector<VkQueueFamilyProperties> qfProps(qfCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &qfCount, qfProps.data());

    QueueFamilyIndices idx;
    for (uint32_t i = 0; i < qfCount; ++i) {
        if (qfProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) idx.graphics = i;
        VkBool32 present;
        vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice_, i, surface_, &present);
        if (present) idx.present = i;
        if (idx.Complete()) break;
    }

    std::set<uint32_t> uniqueQFs = {*idx.graphics, *idx.present};
    std::vector<VkDeviceQueueCreateInfo> qcis;
    float priority = 1.0f;
    for (uint32_t qf : uniqueQFs) {
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = qf;
        qci.queueCount = 1;
        qci.pQueuePriorities = &priority;
        qcis.push_back(qci);
    }

    VkPhysicalDeviceFeatures features{};
    features.fillModeNonSolid = VK_TRUE;

    VkDeviceCreateInfo ci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    ci.queueCreateInfoCount = (uint32_t)qcis.size();
    ci.pQueueCreateInfos = qcis.data();
    ci.pEnabledFeatures = &features;

    const char* devExts[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    ci.enabledExtensionCount = 1;
    ci.ppEnabledExtensionNames = devExts;

    if (vkCreateDevice(physicalDevice_, &ci, nullptr, &device_) != VK_SUCCESS)
        throw std::runtime_error("vkCreateDevice failed");

    vkGetDeviceQueue(device_, *idx.graphics, 0, &graphicsQueue_);
    vkGetDeviceQueue(device_, *idx.present, 0, &presentQueue_);
}

void VulkanRenderer::CreateSwapchain() {
    SwapchainSupport support;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice_, surface_, &support.capabilities);

    uint32_t fmtCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice_, surface_, &fmtCount, nullptr);
    support.formats.resize(fmtCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice_, surface_, &fmtCount, support.formats.data());

    uint32_t modeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice_, surface_, &modeCount, nullptr);
    support.presentModes.resize(modeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice_, surface_, &modeCount, support.presentModes.data());

    swapchainImageFormat_ = VK_FORMAT_B8G8R8A8_SRGB;
    VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
    for (auto& f : support.formats) {
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            swapchainImageFormat_ = f.format;
            break;
        }
    }

    VkExtent2D extent = support.capabilities.currentExtent;
    if (extent.width == UINT32_MAX) {
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        extent = { (uint32_t)std::max(1, w), (uint32_t)std::max(1, h) };
    }
    swapchainExtent_ = extent;

    uint32_t imageCount = support.capabilities.minImageCount + 1;
    if (support.capabilities.maxImageCount > 0 && imageCount > support.capabilities.maxImageCount)
        imageCount = support.capabilities.maxImageCount;

    VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    ci.surface = surface_;
    ci.minImageCount = imageCount;
    ci.imageFormat = swapchainImageFormat_;
    ci.imageColorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    ci.imageExtent = extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ci.preTransform = support.capabilities.currentTransform;
    ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    ci.presentMode = presentMode;
    ci.clipped = VK_TRUE;

    uint32_t qfCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &qfCount, nullptr);
    std::vector<VkQueueFamilyProperties> qfProps(qfCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &qfCount, qfProps.data());

    QueueFamilyIndices idx;
    for (uint32_t i = 0; i < qfCount; ++i) {
        if (qfProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) idx.graphics = i;
        VkBool32 present;
        vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice_, i, surface_, &present);
        if (present) idx.present = i;
    }
    if (idx.graphics == idx.present) {
        ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    } else {
        uint32_t families[] = {*idx.graphics, *idx.present};
        ci.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        ci.queueFamilyIndexCount = 2;
        ci.pQueueFamilyIndices = families;
    }

    if (vkCreateSwapchainKHR(device_, &ci, nullptr, &swapchain_) != VK_SUCCESS)
        throw std::runtime_error("vkCreateSwapchainKHR failed");

    vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, nullptr);
    swapchainImages_.resize(imageCount);
    vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, swapchainImages_.data());
}

void VulkanRenderer::CreateImageViews() {
    swapchainImageViews_.resize(swapchainImages_.size());
    for (size_t i = 0; i < swapchainImages_.size(); ++i) {
        VkImageViewCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        ci.image = swapchainImages_[i];
        ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
        ci.format = swapchainImageFormat_;
        ci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        ci.subresourceRange.levelCount = 1;
        ci.subresourceRange.layerCount = 1;
        if (vkCreateImageView(device_, &ci, nullptr, &swapchainImageViews_[i]) != VK_SUCCESS)
            throw std::runtime_error("vkCreateImageView failed");
    }
}

void VulkanRenderer::CreateRenderPass() {
    VkAttachmentDescription color{VK_STRUCTURE_TYPE_ATTACHMENT_DESCRIPTION_2};
    color.format = swapchainImageFormat_;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &ref;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dep.srcAccessMask = 0;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo ci{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    ci.attachmentCount = 1;
    ci.pAttachments = &color;
    ci.subpassCount = 1;
    ci.pSubpasses = &subpass;
    ci.dependencyCount = 1;
    ci.pDependencies = &dep;

    if (vkCreateRenderPass(device_, &ci, nullptr, &renderPass_) != VK_SUCCESS)
        throw std::runtime_error("vkCreateRenderPass failed");
}

VkShaderModule VulkanRenderer::CreateShaderModule(const std::vector<char>& code) {
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    ci.codeSize = code.size();
    ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule mod;
    if (vkCreateShaderModule(device_, &ci, nullptr, &mod) != VK_SUCCESS)
        throw std::runtime_error("vkCreateShaderModule failed");
    return mod;
}

void VulkanRenderer::CreatePipeline() {
    vertShaderCode_ = LoadBinaryFile(SHADER_DIR "/mesh.vert.spv");
    fragShaderCode_ = LoadBinaryFile(SHADER_DIR "/mesh.frag.spv");
    auto vertMod = CreateShaderModule(vertShaderCode_);
    auto fragMod = CreateShaderModule(fragShaderCode_);

    VkPipelineShaderStageCreateInfo vertStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertStage.module = vertMod;
    vertStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragStage = vertStage;
    fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragStage.module = fragMod;

    VkPipelineShaderStageCreateInfo stages[] = {vertStage, fragStage};

    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = 3 * sizeof(float);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription attr{};
    attr.location = 0;
    attr.binding = 0;
    attr.format = VK_FORMAT_R32G32B32_SFLOAT;
    attr.offset = 0;

    VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = 1;
    vertexInput.pVertexAttributeDescriptions = &attr;

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkViewport viewport{};
    viewport.width = (float)swapchainExtent_.width;
    viewport.height = (float)swapchainExtent_.height;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor{{0, 0}, swapchainExtent_};

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.pViewports = &viewport;
    viewportState.scissorCount = 1;
    viewportState.pScissors = &scissor;

    VkPipelineRasterizationStateCreateInfo rasterizer{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_LINE;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState blend{};
    blend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                           VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    VkPipelineColorBlendStateCreateInfo colorBlend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlend.attachmentCount = 1;
    colorBlend.pAttachments = &blend;

    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = 80; // mat4 (64) + vec3 (12) + pad (4)

    VkPipelineLayoutCreateInfo pipeLayout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipeLayout.pushConstantRangeCount = 1;
    pipeLayout.pPushConstantRanges = &pushRange;

    if (vkCreatePipelineLayout(device_, &pipeLayout, nullptr, &pipelineLayout_) != VK_SUCCESS)
        throw std::runtime_error("vkCreatePipelineLayout failed");

    VkGraphicsPipelineCreateInfo pipe{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipe.stageCount = 2;
    pipe.pStages = stages;
    pipe.pVertexInputState = &vertexInput;
    pipe.pInputAssemblyState = &inputAssembly;
    pipe.pViewportState = &viewportState;
    pipe.pRasterizationState = &rasterizer;
    pipe.pMultisampleState = &multisample;
    pipe.pColorBlendState = &colorBlend;
    pipe.layout = pipelineLayout_;
    pipe.renderPass = renderPass_;
    pipe.subpass = 0;

    if (vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipe, nullptr, &graphicsPipeline_) != VK_SUCCESS)
        throw std::runtime_error("vkCreateGraphicsPipelines failed");

    vkDestroyShaderModule(device_, vertMod, nullptr);
    vkDestroyShaderModule(device_, fragMod, nullptr);
}

void VulkanRenderer::CreateFramebuffers() {
    framebuffers_.resize(swapchainImageViews_.size());
    for (size_t i = 0; i < swapchainImageViews_.size(); ++i) {
        VkFramebufferCreateInfo ci{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        ci.renderPass = renderPass_;
        ci.attachmentCount = 1;
        ci.pAttachments = &swapchainImageViews_[i];
        ci.width = swapchainExtent_.width;
        ci.height = swapchainExtent_.height;
        ci.layers = 1;
        if (vkCreateFramebuffer(device_, &ci, nullptr, &framebuffers_[i]) != VK_SUCCESS)
            throw std::runtime_error("vkCreateFramebuffer failed");
    }
}

void VulkanRenderer::CreateCommandPool() {
    uint32_t qfCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &qfCount, nullptr);
    std::vector<VkQueueFamilyProperties> qfProps(qfCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &qfCount, qfProps.data());

    uint32_t gfxFamily = 0;
    for (uint32_t i = 0; i < qfCount; ++i) {
        if (qfProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            gfxFamily = i;
            break;
        }
    }

    VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    ci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    ci.queueFamilyIndex = gfxFamily;
    if (vkCreateCommandPool(device_, &ci, nullptr, &commandPool_) != VK_SUCCESS)
        throw std::runtime_error("vkCreateCommandPool failed");

    VkCommandBufferAllocateInfo cbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cbi.commandPool = commandPool_;
    cbi.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbi.commandBufferCount = 1;
    vkAllocateCommandBuffers(device_, &cbi, &commandBuffer_);
}

void VulkanRenderer::CreateVertexIndexBuffers() {
    // vertex buffer — allocated once in UploadMesh
    // index buffer — same
}

void VulkanRenderer::CreateSyncObjects() {
    VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    vkCreateSemaphore(device_, &si, nullptr, &imageAvailableSemaphore_);
    vkCreateSemaphore(device_, &si, nullptr, &renderFinishedSemaphore_);

    VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    vkCreateFence(device_, &fi, nullptr, &inFlightFence_);
}

void VulkanRenderer::UploadMesh(const Eigen::MatrixXf& positions, const std::vector<uint32_t>& indices) {
    indexCount_ = (uint32_t)indices.size();
    if (indexCount_ == 0) return;

    // Clean up old buffers if re-uploading
    if (vertexBuffer_ != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, vertexBuffer_, nullptr);
        vkFreeMemory(device_, vertexBufferMemory_, nullptr);
    }
    if (indexBuffer_ != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, indexBuffer_, nullptr);
        vkFreeMemory(device_, indexBufferMemory_, nullptr);
    }

    // Vertex buffer — host-visible for per-frame streaming updates
    VkDeviceSize vSize = positions.rows() * 3 * sizeof(float);
    CreateBuffer(device_, physicalDevice_, vSize,
                 VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 vertexBuffer_, vertexBufferMemory_);

    void* data;
    vkMapMemory(device_, vertexBufferMemory_, 0, vSize, 0, &data);
    float* dst = static_cast<float*>(data);
    for (int i = 0; i < positions.rows(); ++i)
        for (int j = 0; j < 3; ++j)
            *dst++ = positions(i, j);
    vkUnmapMemory(device_, vertexBufferMemory_);

    // Index buffer — device-local (indices never change)
    VkDeviceSize iSize = indices.size() * sizeof(uint32_t);
    VkBuffer staging;
    VkDeviceMemory stagingMem;
    CreateBuffer(device_, physicalDevice_, iSize,
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 staging, stagingMem);
    vkMapMemory(device_, stagingMem, 0, iSize, 0, &data);
    memcpy(data, indices.data(), iSize);
    vkUnmapMemory(device_, stagingMem);

    CreateBuffer(device_, physicalDevice_, iSize,
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 indexBuffer_, indexBufferMemory_);

    auto cb = BeginSingleTimeCommands(device_, commandPool_);
    VkBufferCopy copy{0, 0, iSize};
    vkCmdCopyBuffer(cb, staging, indexBuffer_, 1, &copy);
    EndSingleTimeCommands(device_, commandPool_, graphicsQueue_, cb);

    vkDestroyBuffer(device_, staging, nullptr);
    vkFreeMemory(device_, stagingMem, nullptr);
}

void VulkanRenderer::UpdateVertices(const Eigen::MatrixXf& positions) {
    if (vertexBufferMemory_ == VK_NULL_HANDLE) return;
    VkDeviceSize size = positions.rows() * 3 * sizeof(float);
    void* data;
    vkMapMemory(device_, vertexBufferMemory_, 0, size, 0, &data);
    float* dst = static_cast<float*>(data);
    for (int i = 0; i < positions.rows(); ++i)
        for (int j = 0; j < 3; ++j)
            *dst++ = positions(i, j);
    vkUnmapMemory(device_, vertexBufferMemory_);
}

void VulkanRenderer::BeginFrame() {
    vkWaitForFences(device_, 1, &inFlightFence_, VK_TRUE, UINT64_MAX);

    VkResult result = vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX,
                                             imageAvailableSemaphore_, VK_NULL_HANDLE, &imageIndex_);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        RecreateSwapchain();
        return;
    }

    vkResetFences(device_, 1, &inFlightFence_);

    vkResetCommandBuffer(commandBuffer_, 0);
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    vkBeginCommandBuffer(commandBuffer_, &bi);

    VkRenderPassBeginInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rp.renderPass = renderPass_;
    rp.framebuffer = framebuffers_[imageIndex_];
    rp.renderArea = {{0, 0}, swapchainExtent_};
    VkClearValue clear = {{{0.05f, 0.05f, 0.1f, 1.0f}}};
    rp.clearValueCount = 1;
    rp.pClearValues = &clear;
    vkCmdBeginRenderPass(commandBuffer_, &rp, VK_SUBPASS_CONTENTS_INLINE);
}

void VulkanRenderer::EndFrame(const Eigen::Matrix4f& viewProj, const Eigen::Vector3f& color) {
    if (indexCount_ > 0) {
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline_);

        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(commandBuffer_, 0, 1, &vertexBuffer_, offsets);
        vkCmdBindIndexBuffer(commandBuffer_, indexBuffer_, 0, VK_INDEX_TYPE_UINT32);

        struct { float mvp[16]; float color[3]; float _pad; } pc;
        memcpy(pc.mvp, viewProj.data(), 64);
        pc.color[0] = color.x(); pc.color[1] = color.y(); pc.color[2] = color.z();
        pc._pad = 0;
        vkCmdPushConstants(commandBuffer_, pipelineLayout_,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(pc), &pc);

        vkCmdDrawIndexed(commandBuffer_, indexCount_, 1, 0, 0, 0);
    }

    vkCmdEndRenderPass(commandBuffer_);
    vkEndCommandBuffer(commandBuffer_);

    VkPipelineStageFlags waitStages = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.waitSemaphoreCount = 1;
    si.pWaitSemaphores = &imageAvailableSemaphore_;
    si.pWaitDstStageMask = &waitStages;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &commandBuffer_;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores = &renderFinishedSemaphore_;
    vkQueueSubmit(graphicsQueue_, 1, &si, inFlightFence_);

    VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores = &renderFinishedSemaphore_;
    pi.swapchainCount = 1;
    pi.pSwapchains = &swapchain_;
    pi.pImageIndices = &imageIndex_;

    VkResult result = vkQueuePresentKHR(presentQueue_, &pi);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized_) {
        framebufferResized_ = false;
        RecreateSwapchain();
    }
}

void VulkanRenderer::RecreateSwapchain() {
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    while (w == 0 || h == 0) {
        glfwWaitEvents();
        glfwGetFramebufferSize(window_, &w, &h);
    }
    vkDeviceWaitIdle(device_);
    CleanupSwapchain();
    CreateSwapchain();
    CreateImageViews();
    CreateRenderPass();
    CreatePipeline();
    CreateFramebuffers();
}

void VulkanRenderer::CleanupSwapchain() {
    for (auto fb : framebuffers_) vkDestroyFramebuffer(device_, fb, nullptr);
    for (auto iv : swapchainImageViews_) vkDestroyImageView(device_, iv, nullptr);
    vkDestroyPipeline(device_, graphicsPipeline_, nullptr);
    vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
    vkDestroyRenderPass(device_, renderPass_, nullptr);
    vkDestroySwapchainKHR(device_, swapchain_, nullptr);
}

void VulkanRenderer::WaitIdle() {
    vkDeviceWaitIdle(device_);
}

void VulkanRenderer::Shutdown() {
    vkDeviceWaitIdle(device_);

    CleanupSwapchain();

    if (vertexBuffer_ != VK_NULL_HANDLE)
        vkDestroyBuffer(device_, vertexBuffer_, nullptr);
    if (indexBuffer_ != VK_NULL_HANDLE)
        vkDestroyBuffer(device_, indexBuffer_, nullptr);
    if (vertexBufferMemory_ != VK_NULL_HANDLE)
        vkFreeMemory(device_, vertexBufferMemory_, nullptr);
    if (indexBufferMemory_ != VK_NULL_HANDLE)
        vkFreeMemory(device_, indexBufferMemory_, nullptr);

    vkDestroySemaphore(device_, imageAvailableSemaphore_, nullptr);
    vkDestroySemaphore(device_, renderFinishedSemaphore_, nullptr);
    vkDestroyFence(device_, inFlightFence_, nullptr);
    vkDestroyCommandPool(device_, commandPool_, nullptr);
    vkDestroyDevice(device_, nullptr);
    vkDestroySurfaceKHR(instance_, surface_, nullptr);

    if (debugMessenger_ != VK_NULL_HANDLE) {
        auto fn = (PFN_vkDestroyDebugUtilsMessengerEXT)
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT");
        fn(instance_, debugMessenger_, nullptr);
    }
    vkDestroyInstance(instance_, nullptr);
}
