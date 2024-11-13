# Migrating Added Mass from the Hydrodynamics to the Fluid Added Mass tag

Historically, we used the specify added mass in the parameters of the hydrodynamics plugin.
Starting with Gazebo Ionic, we have deprecated that in favour of using the SDF `<fluid_added_mass>` tag directly. This guarantees better numerical stability. That being said, in order to actually simulate added mass your underlying physics engine must be able to support the added mass tag. So far that only includes the DART physics engine.

# Example migration

Given an example plugin config:
```
<plugin
    filename="gz-sim-hydrodynamics-system"
    name="gz::sim::systems::Hydrodynamics">
    <link_name>base_link</link_name>
    <xDotU>-4.876161</xDotU>
    <yDotV>-126.324739</yDotV>
    <zDotW>-126.324739</zDotW>
    <kDotP>0</kDotP>
    <mDotQ>-33.46</mDotQ>
    <nDotR>-33.46</nDotR>
    <xUabsU>-6.2282</xUabsU>
    <xU>0</xU>
    <yVabsV>-601.27</yVabsV>
    <yV>0</yV>
    <zWabsW>-601.27</zWabsW>
    <zW>0</zW>
    <kPabsP>-0.1916</kPabsP>
    <kP>0</kP>
    <mQabsQ>-632.698957</mQabsQ>
    <mQ>0</mQ>
    <nRabsR>-632.698957</nRabsR>
    <nR>0</nR>
</plugin>
```

In your model file under the link's inertial tag add the parameters like so:
```
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="tethys">
    <!-- Body -->
    <link name="base_link">
      <inertial>
        <mass>147.8671</mass>
        <inertia>
          <ixx>3.000000</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>41.980233</iyy>
          <iyz>0</iyz>
          <izz>41.980233</izz>
        </inertia>
        <fluid_added_mass>
	    <xx>4.876161</xx>
	    <xy>0.0</xy>
	    <xz>0.0</xz>
	    <xp>0.0</xp>
	    <xq>0.0</xq>
	    <xr>0.0</xr>
	    <yy>126.324739</yy>
	    <yz>0.0</yz>
	    <yp>0.0</yp>
	    <yq>0.0</yq>
	    <yr>0.0</yr>
	    <zz>126.324739</zz>
	    <zp>0.0</zp>
	    <zq>0.0</zq>
	    <zr>0.0</zr>
	    <pp>0.1916</pp>
	    <pq>33.46</pq>
	    <pr>0.0</pr>
	    <qq>0.0</qq>
	    <qr>33.46</qr>
	    <rr>0.0</rr>
  	</fluid_added_mass>
      </inertial>
    ...
    </link>
```

Finally get rid of all the parameters with `Dot` in the plugin defintition.


# Parameter mappings

The hydrodynamics plugin traditionally uses `<X|Y|X|K|M|Ndot|U|V|W|P|Q|R>` format. Whereas the added mass is specified in the SDFormat file as `<x|y|z|p|q|rx|y|z|p|q|r>`. So for instance `<XdotU>` becomes `<xx>`.
