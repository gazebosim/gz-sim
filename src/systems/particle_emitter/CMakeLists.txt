gz_add_system(particle-emitter
  SOURCES
    ParticleEmitter.cc
  PUBLIC_LINK_LIBS
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)
