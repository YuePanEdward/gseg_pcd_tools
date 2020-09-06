function mdc2s_m = get_mdc2s_m (scn_pos, cell_center)
  % scn_pos [3*n] 
  % cell_center [3*m] 
  % mdc2s_m [m*n]
  
  n=size(scn_pos,2);
  m=size(cell_center,2);
  mdc2s_m = zeros(m,n);
  
  for i=1:m
      for j=1:n
          mdc2s_m(i,j) = norm(cell_center(:,i) - scn_pos(:,j),2);
      end
  end
 

end