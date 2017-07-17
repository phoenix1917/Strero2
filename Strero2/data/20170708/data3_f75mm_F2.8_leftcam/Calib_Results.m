% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 7964.826040484365000 ; 8585.994139390959400 ];

%-- Principal point:
cc = [ -58.360661561927159 ; 262.069433371418940 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.104832453796137 ; 21.264964738689777 ; -0.007102970821841 ; -0.031969777829461 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 89.309642080673981 ; 101.665404000905720 ];

%-- Principal point uncertainty:
cc_error = [ 0.000000000000000 ; 0.000000000000000 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.446250545735497 ; 17.350604207614815 ; 0.003147817872244 ; 0.016204069672938 ; 0.000000000000000 ];

%-- Image size:
nx = 720;
ny = 576;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 30;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 0;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ NaN ; NaN ; NaN ];
Tc_1  = [ NaN ; NaN ; NaN ];
omc_error_1 = [ NaN ; NaN ; NaN ];
Tc_error_1  = [ NaN ; NaN ; NaN ];

%-- Image #2:
omc_2 = [ 1.807892e+00 ; 1.694975e+00 ; -5.738073e-01 ];
Tc_2  = [ 1.588877e+02 ; -6.109851e+01 ; 5.673845e+03 ];
omc_error_2 = [ 3.083509e-03 ; 2.899119e-03 ; 5.487969e-03 ];
Tc_error_2  = [ 2.496680e-01 ; 2.979378e-01 ; 6.032248e+01 ];

%-- Image #3:
omc_3 = [ -1.955313e+00 ; -1.968229e+00 ; -7.594353e-01 ];
Tc_3  = [ 1.110464e+02 ; -3.209312e+01 ; 5.521722e+03 ];
omc_error_3 = [ 5.275949e-03 ; 5.340710e-03 ; 3.115089e-03 ];
Tc_error_3  = [ 1.724711e-01 ; 1.599192e-01 ; 6.064753e+01 ];

%-- Image #4:
omc_4 = [ 1.926127e+00 ; 1.845348e+00 ; 6.364555e-01 ];
Tc_4  = [ 9.527203e+01 ; -1.090945e+02 ; 5.497026e+03 ];
omc_error_4 = [ 4.333251e-03 ; 4.226565e-03 ; 5.136545e-03 ];
Tc_error_4  = [ 4.918136e-01 ; 6.505428e-02 ; 6.565106e+01 ];

%-- Image #5:
omc_5 = [ -1.899891e+00 ; -1.898838e+00 ; 4.472510e-01 ];
Tc_5  = [ 2.233539e+02 ; -7.145800e+01 ; 5.625048e+03 ];
omc_error_5 = [ 5.453925e-03 ; 5.447893e-03 ; 1.005115e-02 ];
Tc_error_5  = [ 9.724963e-01 ; 7.153511e-02 ; 6.389101e+01 ];

%-- Image #6:
omc_6 = [ 1.680274e+00 ; 1.715300e+00 ; -3.542871e-01 ];
Tc_6  = [ 1.502256e+02 ; -9.160309e+01 ; 5.593369e+03 ];
omc_error_6 = [ 1.444130e-03 ; 1.894734e-03 ; 6.555522e-03 ];
Tc_error_6  = [ 2.880132e-01 ; 3.581810e-01 ; 5.976089e+01 ];

%-- Image #7:
omc_7 = [ -2.016631e+00 ; -1.315391e+00 ; -1.883279e-01 ];
Tc_7  = [ 1.241229e+02 ; -3.382166e+01 ; 5.610250e+03 ];
omc_error_7 = [ 2.875653e-03 ; 2.477279e-03 ; 6.121856e-03 ];
Tc_error_7  = [ 2.300805e-01 ; 1.689041e-01 ; 5.991344e+01 ];

%-- Image #8:
omc_8 = [ -1.840581e+00 ; -1.992055e+00 ; -1.093474e+00 ];
Tc_8  = [ 1.727899e+02 ; -9.730283e+01 ; 5.502703e+03 ];
omc_error_8 = [ 3.926575e-03 ; 3.743735e-03 ; 1.484423e-03 ];
Tc_error_8  = [ 2.426988e-01 ; 3.797322e-01 ; 6.105755e+01 ];

%-- Image #9:
omc_9 = [ 1.711651e+00 ; 2.106177e+00 ; -1.121702e+00 ];
Tc_9  = [ 2.177417e+02 ; -1.319082e+02 ; 5.703404e+03 ];
omc_error_9 = [ 3.976513e-03 ; 3.982598e-03 ; 2.343819e-03 ];
Tc_error_9  = [ 2.524946e-01 ; 5.701730e-01 ; 6.044377e+01 ];

%-- Image #10:
omc_10 = [ -1.334818e+00 ; -2.191204e+00 ; -6.293261e-01 ];
Tc_10  = [ 1.909362e+02 ; -1.166429e+02 ; 5.489285e+03 ];
omc_error_10 = [ 1.580940e-03 ; 3.248267e-03 ; 7.715423e-03 ];
Tc_error_10  = [ 2.285785e-01 ; 5.504036e-01 ; 5.989910e+01 ];

%-- Image #11:
omc_11 = [ NaN ; NaN ; NaN ];
Tc_11  = [ NaN ; NaN ; NaN ];
omc_error_11 = [ NaN ; NaN ; NaN ];
Tc_error_11  = [ NaN ; NaN ; NaN ];

%-- Image #12:
omc_12 = [ -1.987111e+00 ; -1.610012e+00 ; -8.160420e-01 ];
Tc_12  = [ 1.138239e+02 ; -5.164427e+01 ; 5.563720e+03 ];
omc_error_12 = [ 4.290057e-03 ; 3.395737e-03 ; 1.977776e-03 ];
Tc_error_12  = [ 1.940682e-01 ; 2.520055e-01 ; 6.093768e+01 ];

%-- Image #13:
omc_13 = [ -1.410733e+00 ; -2.131032e+00 ; -6.906850e-01 ];
Tc_13  = [ 2.144610e+02 ; -1.018319e+02 ; 5.567988e+03 ];
omc_error_13 = [ 1.993231e-03 ; 3.580221e-03 ; 6.661999e-03 ];
Tc_error_13  = [ 2.499787e-01 ; 4.964628e-01 ; 6.085927e+01 ];

%-- Image #14:
omc_14 = [ 2.179120e+00 ; 1.591348e+00 ; 4.866238e-02 ];
Tc_14  = [ 1.147129e+02 ; -5.641428e+01 ; 5.558049e+03 ];
omc_error_14 = [ 1.710053e-03 ; 1.365087e-03 ; 1.355725e-02 ];
Tc_error_14  = [ 3.826949e-01 ; 1.486731e-01 ; 6.127869e+01 ];

%-- Image #15:
omc_15 = [ -1.973994e+00 ; -1.913334e+00 ; -7.420282e-01 ];
Tc_15  = [ 2.112178e+02 ; -1.037361e+02 ; 5.585277e+03 ];
omc_error_15 = [ 5.250700e-03 ; 5.221672e-03 ; 3.226015e-03 ];
Tc_error_15  = [ 2.378476e-01 ; 4.564054e-01 ; 6.147446e+01 ];

%-- Image #16:
omc_16 = [ -1.881705e+00 ; -1.901004e+00 ; -6.444936e-01 ];
Tc_16  = [ 9.449514e+01 ; -1.270269e+02 ; 5.604687e+03 ];
omc_error_16 = [ 4.685897e-03 ; 4.944107e-03 ; 5.138837e-03 ];
Tc_error_16  = [ 1.785011e-01 ; 6.267217e-01 ; 6.138157e+01 ];

%-- Image #17:
omc_17 = [ -1.998406e+00 ; -2.064298e+00 ; -9.126051e-01 ];
Tc_17  = [ 1.161086e+02 ; -1.551912e+01 ; 5.574817e+03 ];
omc_error_17 = [ 5.037961e-03 ; 4.979021e-03 ; 1.795710e-03 ];
Tc_error_17  = [ 2.122417e-01 ; 9.783063e-02 ; 6.188206e+01 ];

%-- Image #18:
omc_18 = [ 1.710425e+00 ; 2.183613e+00 ; -1.252523e+00 ];
Tc_18  = [ 1.978790e+02 ; -7.455436e+01 ; 5.726698e+03 ];
omc_error_18 = [ 3.929281e-03 ; 3.791658e-03 ; 1.684895e-03 ];
Tc_error_18  = [ 2.828451e-01 ; 2.890162e-01 ; 6.064519e+01 ];

%-- Image #19:
omc_19 = [ 2.361877e+00 ; 1.811643e+00 ; 7.832780e-01 ];
Tc_19  = [ 1.375841e+02 ; -4.012273e+01 ; 5.496615e+03 ];
omc_error_19 = [ 6.706896e-03 ; 5.242490e-03 ; 3.376784e-03 ];
Tc_error_19  = [ 5.452572e-01 ; 9.367956e-02 ; 6.356181e+01 ];

%-- Image #20:
omc_20 = [ NaN ; NaN ; NaN ];
Tc_20  = [ NaN ; NaN ; NaN ];
omc_error_20 = [ NaN ; NaN ; NaN ];
Tc_error_20  = [ NaN ; NaN ; NaN ];

%-- Image #21:
omc_21 = [ NaN ; NaN ; NaN ];
Tc_21  = [ NaN ; NaN ; NaN ];
omc_error_21 = [ NaN ; NaN ; NaN ];
Tc_error_21  = [ NaN ; NaN ; NaN ];

%-- Image #22:
omc_22 = [ NaN ; NaN ; NaN ];
Tc_22  = [ NaN ; NaN ; NaN ];
omc_error_22 = [ NaN ; NaN ; NaN ];
Tc_error_22  = [ NaN ; NaN ; NaN ];

%-- Image #23:
omc_23 = [ NaN ; NaN ; NaN ];
Tc_23  = [ NaN ; NaN ; NaN ];
omc_error_23 = [ NaN ; NaN ; NaN ];
Tc_error_23  = [ NaN ; NaN ; NaN ];

%-- Image #24:
omc_24 = [ NaN ; NaN ; NaN ];
Tc_24  = [ NaN ; NaN ; NaN ];
omc_error_24 = [ NaN ; NaN ; NaN ];
Tc_error_24  = [ NaN ; NaN ; NaN ];

%-- Image #25:
omc_25 = [ NaN ; NaN ; NaN ];
Tc_25  = [ NaN ; NaN ; NaN ];
omc_error_25 = [ NaN ; NaN ; NaN ];
Tc_error_25  = [ NaN ; NaN ; NaN ];

%-- Image #26:
omc_26 = [ NaN ; NaN ; NaN ];
Tc_26  = [ NaN ; NaN ; NaN ];
omc_error_26 = [ NaN ; NaN ; NaN ];
Tc_error_26  = [ NaN ; NaN ; NaN ];

%-- Image #27:
omc_27 = [ NaN ; NaN ; NaN ];
Tc_27  = [ NaN ; NaN ; NaN ];
omc_error_27 = [ NaN ; NaN ; NaN ];
Tc_error_27  = [ NaN ; NaN ; NaN ];

%-- Image #28:
omc_28 = [ NaN ; NaN ; NaN ];
Tc_28  = [ NaN ; NaN ; NaN ];
omc_error_28 = [ NaN ; NaN ; NaN ];
Tc_error_28  = [ NaN ; NaN ; NaN ];

%-- Image #29:
omc_29 = [ NaN ; NaN ; NaN ];
Tc_29  = [ NaN ; NaN ; NaN ];
omc_error_29 = [ NaN ; NaN ; NaN ];
Tc_error_29  = [ NaN ; NaN ; NaN ];

%-- Image #30:
omc_30 = [ NaN ; NaN ; NaN ];
Tc_30  = [ NaN ; NaN ; NaN ];
omc_error_30 = [ NaN ; NaN ; NaN ];
Tc_error_30  = [ NaN ; NaN ; NaN ];

