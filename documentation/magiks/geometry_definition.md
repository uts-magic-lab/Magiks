# Geometry Definition:

This tutorial shows how to define a manipulator geometry to be used by MAGIKS.


The geometry of a manipulator is defined by specifying the DH parameters of it.
DH paramers are stored in class [```Manipulator_Geometry_Settings```](http://uts-magic-lab.github.io/Magiks/ classmagiks_1_1geometry_1_1manipulator__geometry_1_1_manipulator___geometry___settings.html)

### Geometry of PA10:
For example to define the DH parameters of PA10, first create an instance of class ```Manipulator_Geometry_Settings```:

```
import magiks.geometry.manipulator_geometry as geolib 
import math_tools.general_math as genmath # A library of mathemathical global variables

PA10_geometry_settings         = geolib.Manipulator_Geometry_Settings(nlink = 7, manip_name = 'PA10')
PA10_geometry_settings.theta   = genmath.deg_to_rad*numpy.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
PA10_geometry_settings.alpha   = genmath.deg_to_rad*numpy.array([-90.00, 90.00, -90.00, 90.00, -90.00, 90.00, 0.00])
PA10_geometry_settings.a       = numpy.array([0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000])
PA10_geometry_settings.d       = numpy.array([0.3150, 0.0000, 0.4500, 0.0000, 0.5000, 0.0000, 0.0800])
```


