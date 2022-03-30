#include "ModelPhotoShootTest.hh"

// Test the Model Photo Shoot plugin on the example world.
TEST_F(ModelPhotoShootTest, ModelPhotoShootRandomJoints)
{
  this->ModelPhotoShootTestCmd(
      "test/worlds/model_photo_shoot_random_joints.sdf");
}
