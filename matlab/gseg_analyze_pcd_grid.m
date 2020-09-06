
function D = gseg_analyze_pcd_grid(sourcelst,refplane,celldef)

% data_src	{1xN} cell array of source directory names, i.e. the names extracted from the input argument sourcelst
% scn_pos	[3xN] array of scanner coordinates (columns correspond-ing to input directories, rows corresponding to X, Y, Z re-spectively; all values in meters, as extracted from the input file specified as sourcelst)
% ref_pln	structure containing the reference plane used for the anal-ysis; the fields are: 
   % -source (file name of reference plane, i.e. the above input parameter refplane)
   % -nvec (3x1 normal vector of the plane)
   % -offset (1x1 offset such that for all points X of the plane nvec * X - offset = 0 holds)
% cell_def	[Mx4x3] array of corner coordinates (3d, same coordinate system as point clouds) of the M cells extracted from the input file celldef; the pages of this array contain the X, Y and Z coordinates, respectively; the columns refer to the lower left, upper left, upper right and lower right corner of the cells
% mdc2s_m	[MxN] array of mean distance of the scanners from the centers of the grid cells, in meters; the (j,k)-element con-tains the distance from the scanner as of the k-th input di-rectory to the center of the j-th cell (as calculated from scn_pos and cell_def).
% aoi2p_deg	[MxN] array of angles of incidence for (hypothetical) ray from scanner to cell center at the reference plane in deg, calculated from scn_pos and cell_def
% md2p_mm	[MxN] array of mean distance from reference plane per grid cell in mm; the (j,k)-element contains the arithmetic mean of the orthogonal distances of all points of the j-th cell and k-th source directory from the reference plane; if the k-th directory does not have data for the j-th cell, i.e. if there is no file j.pcd in the directory D.data_src{k}, the (j,k)-element of md2p_mm is NaN. The dimension M i.e. number of analyzed cells is determined by celldef (ideally, all directories contain exactly one pcd-file per cell; it is admissible that some directories do not contain pcd-files for all cells)
% stdd2p_mm	corresponding [MxN] array of standard deviations of the orthogonal distances from the reference plane
% ppc_dat	[MxNx6] array of best fit plane and related additional data per cell and point cloud; the data (j,k,:) refer to the best-fit plane through all points from point cloud j within the k-th directory, i.e. to a plane calculated only from the points in j.pcd within the directory D.data_src{k}; the pages contain the components of the normal vector (X, Y, Z), the offset, the number of points used for the estimation of the respective best-fit plane (i.e. the number of points within the cell from the corresponding scan), and the angle of incidence from the scanner to the center of the corresponding best fit plane in deg
% stdloc_mm	[MxN] array of standard deviations of the orthogonal distances from the best-fit plane per cell (i.e. not using the overall reference plane)

[folders_path,stations_x,stations_y,stations_z]=textread(sourcelst,'%s%f%f%f','delimiter',',','headerlines',0);
[ref_pln_a,ref_pln_b,ref_pln_c,ref_pln_d]=textread(refplane,'%f%f%f%f',1,'headerlines',0);
[cells_id,cells_x_bl,cells_y_bl,cells_z_bl,cells_x_tl,cells_y_tl,cells_z_tl,cells_x_tr,cells_y_tr,cells_z_tr,cells_x_br,cells_y_br,cells_z_br]=textread(celldef,'%n%f%f%f%f%f%f%f%f%f%f%f%f','headerlines',1);

m=size(cells_id,1);
n=size(stations_x,1);

D.data_src = folders_path';
D.scn_pos=[stations_x stations_y stations_z]';
D.ref_pln.source=refplane;
D.ref_pln.nvec=[ref_pln_a; ref_pln_b; ref_pln_c];
D.ref_pln.offset=ref_pln_d; 

cell_center = []; 
proj_cell_center = [];
D.cell_def = zeros(m, 4, 3);

% according to the reference plane 
for i=1:m % for each grid (cell)
    cell_i = [cells_x_bl(i) cells_y_bl(i) cells_z_bl(i); cells_x_tl(i) cells_y_tl(i) cells_z_tl(i); cells_x_tr(i) cells_y_tr(i) cells_z_tr(i);cells_x_br(i) cells_y_br(i) cells_z_br(i)];
    D.cell_def(i,:,:)= cell_i; 
    cell_i_center = mean(cell_i);
    [~,cell_i_center_proj] = projpoint2plane(D.ref_pln, cell_i_center); %get cell center
    cell_center = [cell_center; cell_i_center];
    proj_cell_center = [proj_cell_center; cell_i_center_proj];
end
cell_center = cell_center'; % 3*m
proj_cell_center = proj_cell_center'; % 3*m
D.mdc2s_m= get_mdc2s_m(D.scn_pos, proj_cell_center);  % calculate distance from scanner to grid centers
D.aoi2p_deg= get_aoi2p_deg(D.scn_pos, proj_cell_center,D.ref_pln.nvec); % calculate the incidence angle

for i=1:m %for each grid
    for j=1:n %for each scanner
        cur_pc_name = [D.data_src{j} filesep int2str(i) '.pcd'];
        if exist(cur_pc_name,'file')~=0 % the file exists 
            cur_grid_pc = pcread(cur_pc_name); % import the grid point cloud one by one
            cur_proj_dist = zeros(cur_grid_pc.Count,1);
            for k=1:cur_grid_pc.Count
               [cur_proj_dist(k),~] = projpoint2plane(D.ref_pln, cur_grid_pc.Location(k,:));
            end
     
            D.md2p_mm(i,j)=mean(cur_proj_dist);
            D.stdd2p_mm(i,j)=std(cur_proj_dist);
            
            if(cur_grid_pc.Count >=3)
                % fit the plane for each grid  (each frame's best fit plane)             
                cur_grid_ref_pln = fit_plane_from_pts (cur_grid_pc.Location);
                D.ppc_dat(i,j,1:3)=cur_grid_ref_pln.nvec';
                D.ppc_dat(i,j,4) = cur_grid_ref_pln.offset;
                D.ppc_dat(i,j,5)=cur_grid_pc.Count;
                [~,cur_fit_cell_center] = projpoint2plane(cur_grid_ref_pln, cell_center(:,i)');
                D.ppc_dat(i,j,6)= get_aoi2p_deg(D.scn_pos(:,j),cur_fit_cell_center',cur_grid_ref_pln.nvec);
                cur_proj_dist_best_fit = zeros(cur_grid_pc.Count,1);
                for k=1:cur_grid_pc.Count
                   [cur_proj_dist_best_fit(k),~] = projpoint2plane(cur_grid_ref_pln, cur_grid_pc.Location(k,:));
                end
                D.stdloc_mm(i,j)=std(cur_proj_dist_best_fit); 
             
            else  % point number is not enough for fitting the plane
                D.ppc_dat(i,j,:) =NaN;
                D.stdloc_mm(i,j) =NaN;
            end
            
        else % the file dose not exists 
            D.md2p_mm(i,j)=NaN;
            D.stdd2p_mm(i,j)=NaN;
            D.ppc_dat(i,j,:) =NaN;
            D.stdloc_mm(i,j) =NaN;
        end
    end
end

end