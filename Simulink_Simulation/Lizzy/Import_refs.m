% Import reference thetas to workspace
function [refTh1, refTh2] = import_refs(shape)
    if string(shape) == "square"
        refTh1_mat = load('refs\sq_ref_th1.mat');
        refTh2_mat = load('refs\sq_ref_th2.mat');
        refTh1 = refTh1_mat.th1(2,:);
        refTh2 = refTh2_mat.th2(2,:);
    else if string(shape) == "triangle"
        refTh1_mat = load('refs\tri_ref_th1.mat');
        refTh2_mat = load('refs\tri_ref_th2.mat');
        refTh1 = refTh1_mat.refTh1_mat(2,:);
        refTh2 = refTh2_mat.refTh2_mat(2,:);
    else %circle
        refTh1_mat = load('refs\circ_ref_th1.mat');
        refTh2_mat = load('refs\circ_ref_th2.mat');
        refTh1 = refTh1_mat.refTh1_mat(2,:);
        refTh2 = refTh2_mat.refTh2_mat(2,:);
    end
end

% IGNORE - messing with .mat files
% ref = load('refs\ref_tri_0.mat');
% firstSecondStruct = ref.reference(:, [1, 2])'; % First and second columns
% firstThirdStruct = ref.reference(:, [1, 3])'; % First and third columns
% refTh1_mat = firstSecondStruct;
% refTh2_mat = firstThirdStruct;
% save('refs\tri_ref_th1.mat', 'refTh1_mat');
% save('refs\tri_ref_th2.mat', 'refTh2_mat');