# This example will only work if the Ruby interface library was compiled and
# installed.
#
# Modify the RUBYLIB environment variable to include the ignition math 
# library install path. For example, if you install to /user:
#
# $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
#
require 'ignition/math'

va = Ignition::Math::Vector2d.new(1, 2)
vb = Ignition::Math::Vector2d.new(3, 4)
vc = Ignition::Math::Vector2d.new(vb)

printf("va = %f %f\n", va.X(), va.Y())
printf("vb = %f %f\n", vb.X(), vb.Y())
printf("vc = %f %f\n", vc.X(), vc.Y())

vb += va
printf("vb += va: %f %f\n", vb.X(), vb.Y())

vb.Normalize
printf("vb.Normalize = %f %f\n", vb.X(), vb.Y())

printf("vb.Distance(va) = %f\n", vb.Distance(va))
