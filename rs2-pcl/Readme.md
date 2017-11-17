### rs2-pcl, on-going project

1. read data `depth(i,j)` from rs2 
2. compute normalized image map `nim(i,j) = (x,y,1)`
3. compute vertex map `vtx(i,j) = (X,Y,Z)`
4. compute normal map `normal(i,j) = (Nx, Ny, Nz) = unit[(-dz/dx, -dz/dy, 1)]` 
   - such that `<normal(i,j), (0,0,1)>` is negative; this means that the direction of normal is reverse to the viewing direction.
   - or use SVD using neighbors of the vtx.
5. file save in pcl data format `.pcd`
   - `depth`, `nim`, `vtx`, `normal`
6. file save the intrinsic of depth sensor, and the intrinsic of rgb sensor.
