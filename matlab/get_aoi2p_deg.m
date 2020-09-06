function aoi2p_deg = get_aoi2p_deg (scn_pos, cell_center, ref_pln_nvec) 
  % scn_pos [3*n] 
  % cell_center [3*m] 
  % aoi2p_deg [m*n]
  
  n=size(scn_pos,2);
  m=size(cell_center,2);
  aoi2p_deg = zeros(m,n);
  
  for i=1:m
      for j=1:n
          direction_vec= cell_center(:,i) - scn_pos(:,j);
          cos_angle= (ref_pln_nvec' * direction_vec)/norm(ref_pln_nvec,2)/norm(direction_vec,2); 
          aoi2p_deg(i,j) = acosd(cos_angle);
          if aoi2p_deg(i,j)> 90.0
              aoi2p_deg(i,j) = 180.0 -aoi2p_deg(i,j);
          end
      end
  end

end