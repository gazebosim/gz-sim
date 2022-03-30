#include "ModelPhotoShootTest.hh"

// Test the Model Photo Shoot plugin on the example world.
TEST_F(ModelPhotoShootTest, ModelPhotoShootDefaultJoints)
{
  this->ModelPhotoShootTestCmd(
      "examples/worlds/model_photo_shoot.sdf");
}
