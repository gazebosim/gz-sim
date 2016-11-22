# Modify the RUBYLIB environment variable to include the ignition math 
# library install path. For example, if you install to /user:
#
# $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
#
require 'ignition/math'

printf("PI in degrees = %f\n", Ignition::Math::Angle.Pi.Degree)

a1 = Ignition::Math::Angle.new(1.5707)
a2 = Ignition::Math::Angle.new(0.7854)
printf("a1 = %f radians, %f degrees\n", a1.Radian, a1.Degree)
printf("a2 = %f radians, %f degrees\n", a2.Radian, a2.Degree)
printf("a1 * a2 = %f radians, %f degrees\n", (a1 * a2).Radian, (a1 * a2).Degree)
printf("a1 + a2 = %f radians, %f degrees\n", (a1 + a2).Radian, (a1 + a2).Degree)
printf("a1 - a2 = %f radians, %f degrees\n", (a1 - a2).Radian, (a1 - a2).Degree)

a3 = Ignition::Math::Angle.new(15.707)
printf("a3 = %f radians, %f degrees\n", a3.Radian, a3.Degree)
a3.Normalize
printf("a3.Normalize = %f radians, %f degrees\n", a3.Radian, a3.Degree)
