if (pybind11_FOUND)
  gz_add_system(python-system-loader
    SOURCES
      PythonSystemLoader.cc
    PRIVATE_LINK_LIBS
      pybind11::embed
  )
endif()
