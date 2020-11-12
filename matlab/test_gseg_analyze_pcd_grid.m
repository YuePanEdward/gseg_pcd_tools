% Codes for testing gseg_analyze_pcd_grid
% run this script to test gseg_analyze_pcd_grid
clear;clc;close all;

%% data path
basefolder=['.' filesep 'testdata'];
sourcelst=[basefolder filesep 'sourcelst_example_win.txt'];
refplane=[basefolder filesep 'test_plane_coefficients.txt'];
celldef=[basefolder filesep 'test_grids_def.txt'];

%% run the gseg_analyze_pcd_grid function
D = gseg_analyze_pcd_grid(sourcelst,refplane,celldef) 
%output the attributes of the structure

%% test plot 
for i=1:size(D.data_src,2)
figure(i);
subplot(2,1,1);
plot(reshape(D.ppc_dat(:,i,5),1,[]),'o');
ylabel('Number of points');
title(sprintf('Source: %s',D.data_src{i}));
subplot(2,1,2);
plot(D.stdd2p_mm(:,i),'ks','displayname','w.r.t. reference plane');
hold on;
plot(D.stdloc_mm(:,i),'r+','displayname','w.r.t. local best-fit plane');
legend('w.r.t. reference plane', 'w.r.t. local best-fit plane'); % added
ylabel('STD (orth.) /mm');
xlabel('Cell ID');
end

