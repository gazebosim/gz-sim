set (files
  dirt_diffusespecular.png
  water.jpg
  flat_normal.png
  fungus_diffusespecular.png
  grass_diffusespecular.png
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/materials/textures/)
