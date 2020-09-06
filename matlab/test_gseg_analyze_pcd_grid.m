% Codes for testing gseg_analyze_pcd_grid
% run this script to test gseg_analyze_pcd_grid

clear;

basefolder=['.' filesep 'testdata'];
sourcelst=[basefolder filesep 'sourcelst_example_linux.txt'];
refplane=[basefolder filesep 'test_plane_coefficients.txt'];
celldef=[basefolder filesep 'test_grids_def.txt'];

% run the gseg_analyze_pcd_grid function
D = gseg_analyze_pcd_grid(sourcelst,refplane,celldef)

% plot
figure;
subplot(2,1,1);
plot(reshape(D.ppc_dat(:,2,5),1,[]),'o');
ylabel('Number of points');
title(sprintf('Source: %s',D.data_src{2}));
subplot(2,1,2);
plot(D.stdd2p_mm(:,2),'ks','displayname','w.r.t. reference plane');
hold on;
plot(D.stdloc_mm(:,2),'r+','displayname','w.r.t. local best-fit plane');
legend('w.r.t. reference plane', 'w.r.t. local best-fit plane'); % added
ylabel('STD (orth.) /mm');
xlabel('Cell ID');
