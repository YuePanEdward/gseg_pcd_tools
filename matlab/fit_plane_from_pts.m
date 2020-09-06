function ref_pln = fit_plane_from_pts (pts)
  % breif: fit a 3d plane from a set of 3D points
  % input: pts [K*3], K is the number of points
  % output: struct ref_pln
  %        with the following properties
          % -nvec (3x1 normal vector of the plane)
          % -offset (1x1 offset such that for all points X of the plane nvec * X - offset = 0 holds)
  
  % method SVD       
  pts_mean=mean(pts,1); % [1*3]
  center_plane=bsxfun(@minus,pts,pts_mean);
  [U,S,V]=svd(center_plane); 
  
  ref_pln.nvec= V(:,3);
  ref_pln.offset= -pts_mean * ref_pln.nvec;

end