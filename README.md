

# Laser detection and tracking.

##  Considerations:


## Second attemp:
We try to properly calibrate the camera and to be able to project points over the plane on the chessboard. That means that a constraint of this is that we will only be able to work on a plane (wall, flat surface, floor); And so, we can't handle furniture in the scene. But this condition is easily met.

The camera is independently calibrate with the chessboar plane. This is a usual calibration, easy to do. 
The projection of pixel coords to the chessboard plane is a bit challenging but managable. I've done it in another project.

The laser calibration is also handled independently. but we need to use the same world coordinates. For that we manually locate the beam over the chess boards intersection points (since we know their locations) and record the angles of the laser for that point. Collecting 4 points we can compute R and T from laser to world coordinates.

Lastly, laser and camera are linked to the world coordinates. Then it's easy to select a pixed on the image and deduce the corresponding angles for the laser, such that the laser beam points over the point selected in the pixel coords.
 

## First attempt:
This attempt failed cause the calibration of the laser and the camera is too slow. We are changing of approach.

