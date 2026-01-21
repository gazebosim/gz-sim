#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <map>
#include <atomic>
#include <csignal>
#include <vector>
#include <zenoh.hxx>

#if __APPLE__
  #include <OpenGL/gl.h>
  #include <OpenGL/OpenGL.h>
#elif _WIN32
  #define NOMINMAX
  #include <windows.h>
  #include <GL/glew.h>
  #include <GL/glu.h>
  #include "Wingdi.h"
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
#endif

#if !defined(__APPLE__) && !defined(_WIN32)
  #include <GL/glx.h>
#endif

#include <GLFW/glfw3.h>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Image.hh>

#include "position_generated.h"

// Global variables for windowing
unsigned int imgw = 0;
unsigned int imgh = 0;
gz::rendering::CameraPtr g_camera;
gz::rendering::ImagePtr g_image;

#if __APPLE__
  CGLContextObj g_context;
  CGLContextObj g_glfwContext;
#elif _WIN32
  HGLRC g_context = 0;
  HDC g_display = 0;
  HGLRC g_glfwContext = 0;
  HDC g_glfwDisplay = 0;
#else
  GLXContext g_context;
  Display *g_display;
  GLXDrawable g_drawable;
  GLXContext g_glfwContext;
  Display *g_glfwDisplay;
  GLXDrawable g_glfwDrawable;
#endif

std::atomic<bool> g_running(true);

struct TargetPos {
    float x = 0, y = 0, z = 1;
    std::mutex mutex;
};
TargetPos g_pos;

void on_position(const zenoh::Sample& sample) {
    const auto buf = sample.get_payload().as_vector();
    auto pos = ExternalRenderer::GetPosition(buf.data());
    std::lock_guard<std::mutex> lock(g_pos.mutex);
    g_pos.x = pos->x();
    g_pos.y = pos->y();
    g_pos.z = pos->z();
}

void signal_handler(int) {
    g_running = false;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS)
  {
    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)
    {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      g_running = false;
    }
  }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 1. Initialize rendering engine
    std::map<std::string, std::string> params;
    params["engine"] = "ogre2";
#if __APPLE__
    params["metal"] = "1";
    std::cout << "Using Metal backend" << std::endl;
#endif
    // params["headless"] = "0";

    gz::rendering::RenderEngine* engine = gz::rendering::engine("ogre2", params);
    if (!engine) {
        std::cerr << "Failed to load Ogre2 engine" << std::endl;
        return 1;
    }

    // 2. Create scene
    gz::rendering::ScenePtr scene = engine->CreateScene("external_scene");
    if (!scene) {
        std::cerr << "Failed to create scene" << std::endl;
        return 1;
    }
    scene->SetBackgroundColor(0.1, 0.1, 0.1);
    scene->SetAmbientLight(0.4, 0.4, 0.4);

    // 3. Create entities
    gz::rendering::VisualPtr root = scene->RootVisual();
    gz::rendering::VisualPtr box = scene->CreateVisual("red_box");
    box->AddGeometry(scene->CreateBox());
    box->SetLocalPosition(0, 0, 1);
    
    gz::rendering::MaterialPtr red = scene->CreateMaterial();
    red->SetAmbient(1.0, 0.0, 0.0);
    red->SetDiffuse(1.0, 0.0, 0.0);
    box->SetMaterial(red);
    root->AddChild(box);

    // 4. Create camera
    g_camera = scene->CreateCamera("main_camera");
    g_camera->SetLocalPosition(-5, 0, 2);
    g_camera->SetLocalRotation(0, 0.2, 0);
    imgw = 800;
    imgh = 600;
    g_camera->SetImageWidth(imgw);
    g_camera->SetImageHeight(imgh);
    g_camera->SetAspectRatio(1.333);
    root->AddChild(g_camera);

    // Create Image to capture into
    g_image = std::make_shared<gz::rendering::Image>(g_camera->CreateImage());

    // 5. Setup Zenoh
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));
    auto sub = session.declare_subscriber(
        zenoh::KeyExpr("external_renderer/position"),
        [](const zenoh::Sample &sample) { on_position(sample); },
        zenoh::closures::none);

    // 6. Setup GLFW and run loop
    
    // Capture current context (from gz-rendering)
#if __APPLE__
  g_context = CGLGetCurrentContext();
#elif _WIN32
  g_context = wglGetCurrentContext();
  g_display = wglGetCurrentDC();
#else
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif
  
    // Initial capture to setup things
    g_camera->Capture(*g_image);

    if (!glfwInit()) {
        std::cerr << "Error initializing GLFW" << std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(imgw, imgh, "Renderer App", NULL, NULL);
    if (!window) {
        std::cerr << "Error creating GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);

#if __APPLE__
  g_glfwContext = CGLGetCurrentContext();
#elif _WIN32
  g_glfwContext = wglGetCurrentContext();
  g_glfwDisplay = wglGetCurrentDC();
#else
  g_glfwDisplay = glXGetCurrentDisplay();
  g_glfwDrawable = glXGetCurrentDrawable();
  g_glfwContext = glXGetCurrentContext();
#endif

    std::cout << "Renderer started. Waiting for position updates..." << std::endl;

    while (!glfwWindowShouldClose(window) && g_running) {
        // Update position
        {
            std::lock_guard<std::mutex> lock(g_pos.mutex);
            gz::rendering::VisualPtr b = scene->VisualByName("red_box");
            if (b) {
                b->SetLocalPosition(g_pos.x, g_pos.y, g_pos.z);
            }
        }

        // Switch to gz-rendering context
#if __APPLE__
        CGLSetCurrentContext(g_context);
#elif _WIN32
        wglMakeCurrent(g_display, g_context);
#else
        if (g_display) glXMakeCurrent(g_display, g_drawable, g_context);
#endif

        g_camera->Capture(*g_image);

        // Switch to GLFW context
#if __APPLE__
        CGLSetCurrentContext(g_glfwContext);
#elif _WIN32
        wglMakeCurrent(g_glfwDisplay, g_glfwContext);
#else
        glXMakeCurrent(g_glfwDisplay, g_glfwDrawable, g_glfwContext);
#endif

        unsigned char *data = g_image->Data<unsigned char>();

        glClearColor(0.5, 0.5, 0.5, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPixelZoom(1, -1);
        glRasterPos2f(-1, 1);
        glDrawPixels(imgw, imgh, GL_RGB, GL_UNSIGNED_BYTE, data);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
