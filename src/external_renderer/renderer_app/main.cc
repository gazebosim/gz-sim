#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <map>

#if __APPLE__
  #include <GLUT/glut.h>
#else
  #include <GL/glut.h>
#endif

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Image.hh>

#include <zenoh.hxx>
#include "position_generated.h"

/////////////////////////////////////////////////
// Globals
gz::rendering::ScenePtr g_scene;
gz::rendering::CameraPtr g_camera;
gz::rendering::VisualPtr g_box;
gz::rendering::ImagePtr g_image;

struct TargetPos {
    float x = 0, y = 0, z = 1;
    std::mutex mutex;
};
TargetPos g_pos;

unsigned int g_width = 800;
unsigned int g_height = 600;

/////////////////////////////////////////////////
void on_position(const zenoh::Sample& sample) {
    const auto buf = sample.get_payload().as_vector();
    auto pos = ExternalRenderer::GetPosition(buf.data());
    std::lock_guard<std::mutex> lock(g_pos.mutex);
    g_pos.x = pos->x();
    g_pos.y = pos->y();
    g_pos.z = pos->z();
}

/////////////////////////////////////////////////
void display_cb() {
    if (g_box) {
        std::lock_guard<std::mutex> lock(g_pos.mutex);
        g_box->SetLocalPosition(g_pos.x, g_pos.y, g_pos.z);
    }
    
    if (g_camera && g_image) {
        // Capture scene from Gazebo camera
        g_camera->Capture(*g_image);
        unsigned char *data = g_image->Data<unsigned char>();

        // Draw the captured image to the GLUT window
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPixelZoom(1, -1);
        glRasterPos2f(-1, 1);
        glDrawPixels(g_width, g_height, GL_RGB, GL_UNSIGNED_BYTE, data);
        glutSwapBuffers();
    }
}

/////////////////////////////////////////////////
void idle_cb() {
    glutPostRedisplay();
}

/////////////////////////////////////////////////
void build_scene() {
    g_scene->SetBackgroundColor(0.1, 0.1, 0.1);
    g_scene->SetAmbientLight(0.4, 0.4, 0.4);

    gz::rendering::VisualPtr root = g_scene->RootVisual();

    // Create a red box
    g_box = g_scene->CreateVisual("red_box");
    g_box->AddGeometry(g_scene->CreateBox());
    g_box->SetLocalPosition(0, 0, 1);
    
    gz::rendering::MaterialPtr red = g_scene->CreateMaterial();
    red->SetAmbient(1.0, 0.0, 0.0);
    red->SetDiffuse(1.0, 0.0, 0.0);
    g_box->SetMaterial(red);
    root->AddChild(g_box);

    // Create a camera
    g_camera = g_scene->CreateCamera("main_camera");
    g_camera->SetLocalPosition(-5, 0, 2);
    g_camera->SetLocalRotation(0, 0.2, 0);
    g_camera->SetImageWidth(g_width);
    g_camera->SetImageHeight(g_height);
    g_camera->SetAspectRatio(static_cast<double>(g_width) / g_height);
    root->AddChild(g_camera);

    // Prepare image buffer for capture
    g_image = std::make_shared<gz::rendering::Image>(g_camera->CreateImage());
}

/////////////////////////////////////////////////
int main(int argc, char** argv) {
    // 1. Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_width, g_height);
    glutCreateWindow("Gazebo External Renderer (Ogre2 + Zenoh)");

    // 2. Initialize rendering engine
    std::map<std::string, std::string> params;
    params["useCurrentGLContext"] = "1";
    std::string engineName = "ogre2";
    
    gz::rendering::RenderEngine* engine = gz::rendering::engine(engineName, params);
    if (!engine) {
        std::cerr << "Failed to load engine: " << engineName << std::endl;
        return 1;
    }

    // 3. Create scene and entities
    g_scene = engine->CreateScene("external_scene");
    if (!g_scene) {
        std::cerr << "Failed to create scene" << std::endl;
        return 1;
    }
    build_scene();

    // 4. Setup Zenoh
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));
    auto sub = session.declare_subscriber(zenoh::KeyExpr("external_renderer/position"), on_position);

    std::cout << "Renderer started. Waiting for position updates..." << std::endl;

    // 5. Run GLUT loop
    glutDisplayFunc(display_cb);
    glutIdleFunc(idle_cb);
    glutMainLoop();

    return 0;
}
