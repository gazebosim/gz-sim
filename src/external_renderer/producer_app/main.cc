#include <cstdint>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <zenoh.hxx>
#include "position_generated.h"

int main(int argc, char** argv) {
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));

    auto key_expr = zenoh::KeyExpr("external_renderer/position");
    auto pub = session.declare_publisher(key_expr);

    float t = 0.0f;
    while (true) {
        flatbuffers::FlatBufferBuilder builder(1024);
        auto pos = ExternalRenderer::CreatePosition(builder, 
            2.0f * std::cos(t), 
            2.0f * std::sin(t), 
            1.0f);
        builder.Finish(pos);

        auto bufPtr = builder.GetBufferPointer();
        pub.put(zenoh::Bytes(std::vector<uint8_t>(bufPtr, bufPtr + builder.GetSize())));

        std::cout << "Published position: " << 2.0f * std::cos(t) << ", " << 2.0f * std::sin(t) << ", 1.0" << std::endl;

        t += 0.05f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
