function ref_pln = fit_plane_from_pts (pts, dist_thre)
  % fit_plane_from_pts: function for fit a 3d plane from a set of 3D points
  % input: pts [K*3], K is the number of points
  %        dist_thre, scalar, maximum distance threshold for plane fitting RANSAC (m)
  % output: struct ref_pln
  %        with the following properties
  % -nvec (3x1 normal vector of the plane)
  % -offset (1x1 offset such that for all points X of the plane nvec * X - offset = 0 holds)
  % Matlab Computer Vision Toolbox required
  
  ptCloud = pointCloud(pts);
  warning off;
  %pcfitplane introduced in Computer Vision Toolbox
  plane_model = pcfitplane(ptCloud,dist_thre,'MaxNumTrials',50); 
  ref_pln.nvec= plane_model.Normal';
  ref_pln.offset= plane_model.Parameters(4);

end

% method SVD (deprecated due to the problem of memory)       
%   pts_mean=mean(pts,1); % [1*3]
%   center_plane=bsxfun(@minus,pts,pts_mean);
%   [U,S,V]=svd(center_plane);  
%   ref_pln.nvec= V(:,3);
%   ref_pln.offset= -pts_mean * ref_pln.nvec;