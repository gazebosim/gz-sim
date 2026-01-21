#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <map>
#include <atomic>
#include <csignal>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Geometry.hh>

#include <zenoh.hxx>
#include "position_generated.h"

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

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 1. Initialize rendering engine with Metal support
    std::map<std::string, std::string> params;
    params["engine"] = "ogre2";
#if __APPLE__
    params["metal"] = "1";
    std::cout << "Using Metal backend" << std::endl;
#endif
    params["headless"] = "0";

    gz::rendering::RenderEngine* engine = gz::rendering::engine("ogre2", params);
    if (!engine) {
        std::cerr << "Failed to load Ogre2 engine with Metal." << std::endl;
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
    // On macOS/Metal, Ogre2 might require a window ID if it doesn't auto-create one.
    // However, if we don't provide one, it often attempts to create a default window.
    gz::rendering::CameraPtr camera = scene->CreateCamera("main_camera");
    camera->SetLocalPosition(-5, 0, 2);
    camera->SetLocalRotation(0, 0.2, 0);
    camera->SetImageWidth(800);
    camera->SetImageHeight(600);
    camera->SetAspectRatio(1.333);
    root->AddChild(camera);

    // 5. Setup Zenoh
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));
    auto sub = session.declare_subscriber(
        zenoh::KeyExpr("external_renderer/position"),
        [](const zenoh::Sample &sample) { on_position(sample); },
        zenoh::closures::none);

    std::cout << "Renderer started. Waiting for position updates..." << std::endl;

    // 5. Run GLUT loop
    while (g_running) {
        {
            std::lock_guard<std::mutex> lock(g_pos.mutex);
            box->SetLocalPosition(g_pos.x, g_pos.y, g_pos.z);
        }
        
        // This triggers the actual render. 
        // If a window was created by the engine, it should update.
        camera->Update();
        
        // Ogre2 usually needs this to pump window events if it created the window
        // In gz-rendering, this might be handled internally or we might need 
        // to call an engine-specific update if available.
        
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    return 0;
}
