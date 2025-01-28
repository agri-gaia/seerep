# Geodetic projects and querying within another spatial reference system

This tutorial features creating a project in a user-specified geographic
spatial reference system and
spatially querying SEEREP from another spatial reference system.

The transforms are implemented with the help of [PROJ](https://proj.org/en/stable/)s
C API.

## Creating geodetic projects

Source:
[examples/python/gRPC/meta/gRPC_fb_createGeodeticCoordProject.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/meta/gRPC_fb_createGeodeticCoordProject.py)

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/meta/gRPC_fb_createGeodeticCoordProject.py:15:28"
```

This function call creates a project in the reference system `EPSG:4326`
with its topocentric origin at `latitude=52.358199`, `longitude=8.279679`
and `altitude=4`.
With `latitude` and `longitude` in decimal degree and `altitude` in meters.
Altitude specifies the ellipsoidal height of the origin.
The `"2"` string here defines the name of the projects map frame.

**NOTE**: Currently only SRIDs(Spatial Reference System Identifiers)
with geographic coordinates are supported.
The coordinates defined by the SRID also have to be in latitude first
and longitude second order.

## Query with polygons defined in other spatial reference systems

Source: [examples/python/gRPC/images/gRPC_fb_queryImage.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/images/gRPC_fb_queryImages.py)

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_queryImages.py:146:169"
```

**NOTE**: This is not the full function definition.

This queries SEEREP from another SRID, in this case `"EPSG:3857"`, in which the
spatial polygon has to be defined in.
In this case every SRID supported by `PROJ` can be used, `x` is the first coordinate
and `y` the second.

According to the used SRID this can mean a different type of
coordinate. When using `"EPSG:4314"` for example `x` would be the `latitude` and
`y` the `longitude` in decimal degree (see [EPSG:4314](https://epsg.io/4314))
or when using `"EPSG:32632"` `x` would be `easting` and `y` would be `northing`
in meters (see [EPSG:32632](https://epsg.io/32632)).

The polygon will be transformed to the projects CRS (coordinate reference system)
and then projected to topocentric using `PROJ` in the backend.

**NOTE**: These transformations can have strong deviations or even use ballpark
transformations with even larger errors in the resulting positions depending on
the different SRIDs used.
For testing [https://epsg.io/](https://epsg.io/) can be used,
it uses `PROJ` in the backend as well.

## Transforms backend

For the first stage transform
(transforming coordinates to the projects coordinate reference system) the
[cs2cs](https://proj.org/en/stable/apps/cs2cs.html) [C API equivalent](https://proj.org/en/stable/development/reference/functions.html#c.proj_create_crs_to_crs)
is used. The following example essentially simulates that step.

```bash
cs2cs EPSG:4326 EPSG:3857 <<<"52.3588799817  8.2796787735   -100.0000000000"
```

In the second stage projection to topocentric a `PROJ` string, like in the
following [cct](https://proj.org/en/stable/apps/cct.html) pipeline, is used.

```bash
echo "8.2796787735   52.3588799817  -100.0000000000" | cct -d 10 +proj=pipeline
+step +proj=cart +step +proj=topocentric +lat_0=52.358199 +lon_0=8.279679 +h_0=0.000000
```

The source code for the transforms can be found [here](https://github.com/agri-gaia/seerep/blob/main/seerep_srv/seerep_core/src/core_project.cpp)
in the `transformToMapFrame` method.
