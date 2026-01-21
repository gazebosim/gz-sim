#define Z_FEATURE_SHARED_MEMORY
#define Z_FEATURE_UNSTABLE_API
#include <zenoh.hxx>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

// Graphics headers
#if __APPLE__
  #include <OpenGL/gl.h>
  #include <OpenGL/OpenGL.h>
#elif _WIN32
  #define NOMINMAX
  #include <windows.h>
  #include <GL/glew.h>
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glx.h>
#endif

#include <GLFW/glfw3.h>
#include <gz/rendering.hh>

#include "position_generated.h"

using namespace gz;
using namespace rendering;

// --- Context Helper to hide platform complexity ---
class ContextManager {
public:
    void CaptureGzContext() {
#if __APPLE__
        this->gzCtx = CGLGetCurrentContext();
#elif _WIN32
        this->gzCtx = wglGetCurrentContext();
        this->gzDisplay = wglGetCurrentDC();
#else
        this->gzCtx = glXGetCurrentContext();
        this->gzDisplay = glXGetCurrentDisplay();
        this->gzDrawable = glXGetCurrentDrawable();
#endif
    }

    void CaptureGlfwContext() {
#if __APPLE__
        this->glfwCtx = CGLGetCurrentContext();
#elif _WIN32
        this->glfwCtx = wglGetCurrentContext();
        this->glfwDisplay = wglGetCurrentDC();
#else
        this->glfwCtx = glXGetCurrentContext();
        this->glfwDisplay = glXGetCurrentDisplay();
        this->glfwDrawable = glXGetCurrentDrawable();
#endif
    }

    void SwitchToGz() {
#if __APPLE__
        CGLSetCurrentContext(this->gzCtx);
#elif _WIN32
        wglMakeCurrent(this->gzDisplay, this->gzCtx);
#else
        if (this->gzDisplay) glXMakeCurrent(this->gzDisplay, this->gzDrawable, this->gzCtx);
#endif
    }

    void SwitchToGlfw() {
#if __APPLE__
        CGLSetCurrentContext(this->glfwCtx);
#elif _WIN32
        wglMakeCurrent(this->glfwDisplay, this->glfwCtx);
#else
        glXMakeCurrent(this->glfwDisplay, this->glfwDrawable, this->glfwCtx);
#endif
    }

private:
#if __APPLE__
    CGLContextObj gzCtx;
    CGLContextObj glfwCtx;
#elif _WIN32
    HGLRC gzCtx = 0;
    HDC gzDisplay = 0;
    HGLRC glfwCtx = 0;
    HDC glfwDisplay = 0;
#else
    GLXContext gzCtx;
    Display *gzDisplay;
    GLXDrawable gzDrawable;
    GLXContext glfwCtx;
    Display *glfwDisplay;
    GLXDrawable glfwDrawable;
#endif
};

// --- Globals ---
std::atomic<bool> g_running(true);
struct {
    float x = 0, y = 0, z = 1;
    std::mutex mutex;
} g_pos;

// --- Callbacks ---
void SignalHandler(int) { g_running = false; }

const char *kind_to_str(zenoh::SampleKind kind) {
    switch (kind) {
    case zenoh::SampleKind::Z_SAMPLE_KIND_PUT:
            return "PUT";
    case zenoh::SampleKind::Z_SAMPLE_KIND_DELETE:
            return "DELETE";
        default:
            return "UNKNOWN";
    }
}

void OnPosition(zenoh::Sample& sample) {

  #ifdef DEBUG_SHM
  // if Zenoh is built without SHM support, the only buffer type it can receive is RAW
#if !defined(Z_FEATURE_SHARED_MEMORY)
      const char *payload_type = "RAW";
#endif

  // if Zenoh is built with SHM support but without SHM API (that is unstable), it can
  // receive buffers of any type, but there is no way to detect the buffer type
#if defined(Z_FEATURE_SHARED_MEMORY) && !defined(Z_FEATURE_UNSTABLE_API)
      const char *payload_type = "UNKNOWN";
#endif

  // if Zenoh is built with SHM support and with SHM API, we can detect the exact buffer type
#if defined(Z_FEATURE_SHARED_MEMORY) && defined(Z_FEATURE_UNSTABLE_API)
      const char *payload_type = "RAW";
      {
          // try to convert sample payload into SHM buffer. The conversion will succeed
          // only if payload is carrying underlying SHM buffer
          auto shm = sample.get_payload().as_shm();
          if (shm.has_value()) {
              // try to get mutable access to SHM buffer
              payload_type = zenoh::ZShm::try_mutate(shm.value()).has_value() ? "SHM (MUT)" : "SHM (IMMUT)";
          }
      }
#endif

      std::cout << ">> [Subscriber] Received [" << payload_type << "] " << kind_to_str(sample.get_kind()) << " ('"
                << sample.get_keyexpr().as_string_view() << ")\n";
#endif

    const uint8_t* data_ptr = nullptr;
    std::vector<uint8_t> buf_copy;

    auto shm = sample.get_payload().as_shm();
    if (shm.has_value()) {
        data_ptr = reinterpret_cast<const uint8_t*>(shm.value().get().data());
    } else {
        buf_copy = sample.get_payload().as_vector();
        data_ptr = buf_copy.data();
    }

    if (data_ptr) {
        auto pos = ExternalRenderer::GetPosition(data_ptr);
        std::lock_guard<std::mutex> lock(g_pos.mutex);
        g_pos.x = pos->x();
        g_pos.y = pos->y();
        g_pos.z = pos->z();
    }
}

void KeyCallback(GLFWwindow* window, int key, int, int action, int) {
    if (action == GLFW_PRESS && (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        g_running = false;
    }
}

int main(int, char**) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const unsigned int width = 800;
    const unsigned int height = 600;

    // 1. Init Rendering Engine
    std::map<std::string, std::string> params;
    params["engine"] = "ogre2";
#if __APPLE__
    params["metal"] = "1";
#endif

    auto* renderEngine = gz::rendering::engine("ogre2", params);
    if (!renderEngine) {
        std::cerr << "Failed to load Ogre2 engine" << std::endl;
        return 1;
    }

    // 2. Setup Scene
    auto scene = renderEngine->CreateScene("external_scene");
    scene->SetBackgroundColor(0.1, 0.1, 0.1);
    scene->SetAmbientLight(0.4, 0.4, 0.4);

    auto root = scene->RootVisual();
    
    // Red Box
    auto box = scene->CreateVisual("red_box");
    box->AddGeometry(scene->CreateBox());
    box->SetLocalPosition(0, 0, 1);
    auto mat = scene->CreateMaterial();
    mat->SetAmbient(1.0, 0.0, 0.0);
    mat->SetDiffuse(1.0, 0.0, 0.0);
    box->SetMaterial(mat);
    root->AddChild(box);

    // Camera
    auto camera = scene->CreateCamera("main_camera");
    camera->SetLocalPosition(-5, 0, 2);
    camera->SetLocalRotation(0, 0.2, 0);
    camera->SetImageWidth(width);
    camera->SetImageHeight(height);
    camera->SetAspectRatio(1.333);
    root->AddChild(camera);

    auto image = std::make_shared<Image>(camera->CreateImage());

    // 3. Setup Zenoh
    auto zConfig = zenoh::Config::create_default();
    auto zSession = zenoh::Session::open(std::move(zConfig));
    auto zSub = zSession.declare_subscriber(
        zenoh::KeyExpr("external_renderer/position"),
        [](zenoh::Sample& s) { OnPosition(s); },
        zenoh::closures::none);

    // 4. Setup Window & Contexts
    ContextManager ctxMgr;
    ctxMgr.CaptureGzContext(); // Capture the hidden Ogre context

    // Initial capture forces engine initialization
    camera->Capture(*image);

    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW" << std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(width, height, "Renderer App", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create window" << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, KeyCallback);
    
    ctxMgr.CaptureGlfwContext(); // Capture our new visible context

    std::cout << "Renderer started. Waiting for position updates..." << std::endl;

    // 5. Main Loop
    while (!glfwWindowShouldClose(window) && g_running) {
        // Sync Data
        {
            std::lock_guard<std::mutex> lock(g_pos.mutex);
            if (box) box->SetLocalPosition(g_pos.x, g_pos.y, g_pos.z);
        }

        // Render Offscreen (Gz Context)
        ctxMgr.SwitchToGz();
        camera->Capture(*image);

        // Render to Window (GLFW Context)
        ctxMgr.SwitchToGlfw();
        
        glClearColor(0.5, 0.5, 0.5, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Draw the image buffer
        glPixelZoom(1, -1);
        glRasterPos2f(-1, 1);
        glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, image->template Data<unsigned char>());

        glfwSwapBuffers(window);
        glfwPollEvents();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); 
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
