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

    // Initialize SHM Provider
    zenoh::PosixShmProvider shm_provider(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    std::cout << "SHM Provider initialized." << std::endl;

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

        // Allocate directly in SHM
        auto size = builder.GetSize();
        auto res = shm_provider.alloc_gc_defrag_blocking(size, zenoh::AllocAlignment({0}));
        if (std::holds_alternative<zenoh::ZShmMut>(res)) {
             auto& shm_buf = std::get<zenoh::ZShmMut>(res);
             // Copy data to SHM
             std::memcpy(shm_buf.data(), builder.GetBufferPointer(), size);
             
             // Publish the SHM buffer
             pub.put(std::move(shm_buf));
        } else {
             std::cerr << "SHM alloc failed" << std::endl;
        }

        std::cout << "Published position (SHM): " << 2.0f * std::cos(t) << ", " << 2.0f * std::sin(t) << ", 1.0" << std::endl;

        t += 0.05f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
