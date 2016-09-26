require 'ignition/math'

v1 = Ignition::Math::Vector3d.new(0, 0, 0)
printf("V1: %f %f %f\n", v1.X(), v1.Y(), v1.Z())

v2 = Ignition::Math::Vector3d.new(1, 0, 0)
printf("V2: %f %f %f\n", v2.X(), v2.Y(), v2.Z())

printf("Distance v1->v2: %f\n", v1.Distance(v2))
