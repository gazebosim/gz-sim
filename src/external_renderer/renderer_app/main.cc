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
        gz_ctx = CGLGetCurrentContext();
#elif _WIN32
        gz_ctx = wglGetCurrentContext();
        gz_display = wglGetCurrentDC();
#else
        gz_ctx = glXGetCurrentContext();
        gz_display = glXGetCurrentDisplay();
        gz_drawable = glXGetCurrentDrawable();
#endif
    }

    void CaptureGlfwContext() {
#if __APPLE__
        glfw_ctx = CGLGetCurrentContext();
#elif _WIN32
        glfw_ctx = wglGetCurrentContext();
        glfw_display = wglGetCurrentDC();
#else
        glfw_ctx = glXGetCurrentContext();
        glfw_display = glXGetCurrentDisplay();
        glfw_drawable = glXGetCurrentDrawable();
#endif
    }

    void SwitchToGz() {
#if __APPLE__
        CGLSetCurrentContext(gz_ctx);
#elif _WIN32
        wglMakeCurrent(gz_display, gz_ctx);
#else
        if (gz_display) glXMakeCurrent(gz_display, gz_drawable, gz_ctx);
#endif
    }

    void SwitchToGlfw() {
#if __APPLE__
        CGLSetCurrentContext(glfw_ctx);
#elif _WIN32
        wglMakeCurrent(glfw_display, glfw_ctx);
#else
        glXMakeCurrent(glfw_display, glfw_drawable, glfw_ctx);
#endif
    }

private:
#if __APPLE__
    CGLContextObj gz_ctx;
    CGLContextObj glfw_ctx;
#elif _WIN32
    HGLRC gz_ctx = 0;
    HDC gz_display = 0;
    HGLRC glfw_ctx = 0;
    HDC glfw_display = 0;
#else
    GLXContext gz_ctx;
    Display *gz_display;
    GLXDrawable gz_drawable;
    GLXContext glfw_ctx;
    Display *glfw_display;
    GLXDrawable glfw_drawable;
#endif
};

// --- Globals ---
std::atomic<bool> g_running(true);
struct {
    float x = 0, y = 0, z = 1;
    std::mutex mutex;
} g_pos;

// --- Callbacks ---
void signal_handler(int) { g_running = false; }

void on_position(const zenoh::Sample& sample) {
    const auto buf = sample.get_payload().as_vector();
    auto pos = ExternalRenderer::GetPosition(buf.data());
    std::lock_guard<std::mutex> lock(g_pos.mutex);
    g_pos.x = pos->x();
    g_pos.y = pos->y();
    g_pos.z = pos->z();
}

void key_callback(GLFWwindow* window, int key, int, int action, int) {
    if (action == GLFW_PRESS && (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        g_running = false;
    }
}

int main(int, char**) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const unsigned int width = 800;
    const unsigned int height = 600;

    // 1. Init Rendering Engine
    std::map<std::string, std::string> params;
    params["engine"] = "ogre2";
#if __APPLE__
    params["metal"] = "1";
#endif

    auto* render_engine = gz::rendering::engine("ogre2", params);
    if (!render_engine) {
        std::cerr << "Failed to load Ogre2 engine" << std::endl;
        return 1;
    }

    // 2. Setup Scene
    auto scene = render_engine->CreateScene("external_scene");
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
    auto z_config = zenoh::Config::create_default();
    auto z_session = zenoh::Session::open(std::move(z_config));
    auto z_sub = z_session.declare_subscriber(
        zenoh::KeyExpr("external_renderer/position"),
        [](const zenoh::Sample& s) { on_position(s); },
        zenoh::closures::none);

    // 4. Setup Window & Contexts
    ContextManager ctx_mgr;
    ctx_mgr.CaptureGzContext(); // Capture the hidden Ogre context

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
    glfwSetKeyCallback(window, key_callback);
    
    ctx_mgr.CaptureGlfwContext(); // Capture our new visible context

    std::cout << "Renderer started. Waiting for position updates..." << std::endl;

    // 5. Main Loop
    while (!glfwWindowShouldClose(window) && g_running) {
        // Sync Data
        {
            std::lock_guard<std::mutex> lock(g_pos.mutex);
            if (box) box->SetLocalPosition(g_pos.x, g_pos.y, g_pos.z);
        }

        // Render Offscreen (Gz Context)
        ctx_mgr.SwitchToGz();
        camera->Capture(*image);

        // Render to Window (GLFW Context)
        ctx_mgr.SwitchToGlfw();
        
        glClearColor(0.5, 0.5, 0.5, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Draw the image buffer
        glPixelZoom(1, -1);
        glRasterPos2f(-1, 1);
        glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, image->template Data<unsigned char>());

        glfwSwapBuffers(window);
        glfwPollEvents();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}