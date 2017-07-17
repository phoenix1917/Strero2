% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 7863.112230095615800 ; 8559.964338364985500 ];

%-- Principal point:
cc = [ 298.984854861006910 ; 530.085547914549690 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.123091604622401 ; 14.458759007205499 ; 0.008669990624750 ; 0.003323978575104 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 76.458731932240312 ; 83.083459383581967 ];

%-- Principal point uncertainty:
cc_error = [ 112.876595971101080 ; 104.076135589452040 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.303511246634226 ; 49.232777927212240 ; 0.008350969338739 ; 0.007875529334196 ; 0.000000000000000 ];

%-- Image size:
nx = 720;
ny = 576;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 25;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.237595e+00 ; 2.165182e+00 ; 1.172792e-01 ];
Tc_1  = [ -1.120856e+02 ; -2.736739e+02 ; 5.534277e+03 ];
omc_error_1 = [ 1.461945e-02 ; 1.385152e-02 ; 1.974245e-02 ];
Tc_error_1  = [ 7.958393e+01 ; 6.730196e+01 ; 5.329298e+01 ];

%-- Image #2:
omc_2 = [ -2.091156e+00 ; -1.665499e+00 ; -8.215364e-01 ];
Tc_2  = [ -9.165769e+01 ; -2.202703e+02 ; 5.490048e+03 ];
omc_error_2 = [ 8.078514e-03 ; 1.203343e-02 ; 1.710226e-02 ];
Tc_error_2  = [ 7.893784e+01 ; 6.676833e+01 ; 5.405838e+01 ];

%-- Image #3:
omc_3 = [ 2.020753e+00 ; 1.736052e+00 ; -4.252144e-01 ];
Tc_3  = [ -1.220607e+02 ; -1.979446e+02 ; 5.591812e+03 ];
omc_error_3 = [ 8.895173e-03 ; 1.161943e-02 ; 1.868624e-02 ];
Tc_error_3  = [ 8.034949e+01 ; 6.799456e+01 ; 5.413127e+01 ];

%-- Image #4:
omc_4 = [ 2.012650e+00 ; 1.927113e+00 ; 4.645156e-01 ];
Tc_4  = [ -1.529931e+02 ; -2.913168e+02 ; 5.505827e+03 ];
omc_error_4 = [ 1.202233e-02 ; 9.484718e-03 ; 1.650452e-02 ];
Tc_error_4  = [ 7.920376e+01 ; 6.696356e+01 ; 5.371083e+01 ];

%-- Image #5:
omc_5 = [ -1.830717e+00 ; -1.856124e+00 ; 5.206822e-01 ];
Tc_5  = [ -4.559129e+00 ; -2.360318e+02 ; 5.626785e+03 ];
omc_error_5 = [ 1.169135e-02 ; 8.959834e-03 ; 1.737398e-02 ];
Tc_error_5  = [ 8.087825e+01 ; 6.840667e+01 ; 5.335062e+01 ];

%-- Image #6:
omc_6 = [ 1.810498e+00 ; 1.775685e+00 ; -4.179272e-01 ];
Tc_6  = [ -1.232854e+02 ; -2.431360e+02 ; 5.591847e+03 ];
omc_error_6 = [ 8.605043e-03 ; 1.239073e-02 ; 1.719625e-02 ];
Tc_error_6  = [ 8.036417e+01 ; 6.799869e+01 ; 5.410409e+01 ];

%-- Image #7:
omc_7 = [ -2.143468e+00 ; -1.322823e+00 ; -1.520849e-01 ];
Tc_7  = [ -1.256834e+02 ; -1.973568e+02 ; 5.581707e+03 ];
omc_error_7 = [ 1.060481e-02 ; 8.937983e-03 ; 1.633793e-02 ];
Tc_error_7  = [ 8.018897e+01 ; 6.787157e+01 ; 5.412869e+01 ];

%-- Image #8:
omc_8 = [ 2.044133e+00 ; 1.967064e+00 ; -9.784627e-01 ];
Tc_8  = [ -1.441584e+02 ; -1.938347e+02 ; 5.690016e+03 ];
omc_error_8 = [ 6.548119e-03 ; 1.343077e-02 ; 2.040547e-02 ];
Tc_error_8  = [ 8.181316e+01 ; 6.921197e+01 ; 5.401404e+01 ];

%-- Image #9:
omc_9 = [ -2.061124e+00 ; -1.762680e+00 ; -8.357496e-01 ];
Tc_9  = [ -1.515314e+02 ; -2.368376e+02 ; 5.513373e+03 ];
omc_error_9 = [ 7.717261e-03 ; 1.222482e-02 ; 1.720302e-02 ];
Tc_error_9  = [ 7.930226e+01 ; 6.707591e+01 ; 5.429591e+01 ];

%-- Image #10:
omc_10 = [ -2.181872e+00 ; -1.842220e+00 ; 5.135570e-01 ];
Tc_10  = [ -1.476196e+02 ; -2.421908e+02 ; 5.646732e+03 ];
omc_error_10 = [ 1.269474e-02 ; 7.242183e-03 ; 2.027968e-02 ];
Tc_error_10  = [ 8.120893e+01 ; 6.867508e+01 ; 5.363989e+01 ];

%-- Image #11:
omc_11 = [ -2.049895e+00 ; -2.016960e+00 ; -3.384084e-01 ];
Tc_11  = [ -6.093318e+01 ; -2.942052e+02 ; 5.577562e+03 ];
omc_error_11 = [ 8.070132e-03 ; 1.065127e-02 ; 1.712044e-02 ];
Tc_error_11  = [ 8.021598e+01 ; 6.783131e+01 ; 5.478841e+01 ];

%-- Image #12:
omc_12 = [ 1.488972e+00 ; 2.293033e+00 ; -1.212009e+00 ];
Tc_12  = [ -2.588678e+01 ; -2.476876e+02 ; 5.660633e+03 ];
omc_error_12 = [ 5.571211e-03 ; 1.527190e-02 ; 1.879632e-02 ];
Tc_error_12  = [ 8.137872e+01 ; 6.884009e+01 ; 5.350562e+01 ];

%-- Image #13:
omc_13 = [ -1.328036e+00 ; -2.394136e+00 ; -7.186020e-01 ];
Tc_13  = [ -2.330646e+01 ; -2.925393e+02 ; 5.413371e+03 ];
omc_error_13 = [ 4.861014e-03 ; 1.374425e-02 ; 1.685261e-02 ];
Tc_error_13  = [ 7.785214e+01 ; 6.583520e+01 ; 5.358248e+01 ];

%-- Image #14:
omc_14 = [ -2.159757e+00 ; -2.102378e+00 ; 4.287477e-01 ];
Tc_14  = [ -1.570739e+02 ; -2.330265e+02 ; 5.604719e+03 ];
omc_error_14 = [ 1.260806e-02 ; 8.490008e-03 ; 2.204680e-02 ];
Tc_error_14  = [ 8.061827e+01 ; 6.817574e+01 ; 5.344300e+01 ];

%-- Image #15:
omc_15 = [ NaN ; NaN ; NaN ];
Tc_15  = [ NaN ; NaN ; NaN ];
omc_error_15 = [ NaN ; NaN ; NaN ];
Tc_error_15  = [ NaN ; NaN ; NaN ];

%-- Image #16:
omc_16 = [ NaN ; NaN ; NaN ];
Tc_16  = [ NaN ; NaN ; NaN ];
omc_error_16 = [ NaN ; NaN ; NaN ];
Tc_error_16  = [ NaN ; NaN ; NaN ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ NaN ; NaN ; NaN ];
Tc_19  = [ NaN ; NaN ; NaN ];
omc_error_19 = [ NaN ; NaN ; NaN ];
Tc_error_19  = [ NaN ; NaN ; NaN ];

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

