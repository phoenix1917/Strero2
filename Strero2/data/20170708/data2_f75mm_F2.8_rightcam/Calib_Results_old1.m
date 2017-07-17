% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 7855.250534598558300 ; 8548.671403077356400 ];

%-- Principal point:
cc = [ 289.994788576201760 ; 559.135498939269040 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.048888695372704 ; 10.218783256309868 ; 0.006164506122194 ; 0.002622112079595 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 94.614328578535762 ; 102.855833369114590 ];

%-- Principal point uncertainty:
cc_error = [ 155.100853175191730 ; 132.885202259347070 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.382467274878335 ; 51.488905722323807 ; 0.010393570693423 ; 0.009244109346191 ; 0.000000000000000 ];

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
omc_1 = [ 2.239711e+00 ; 2.166757e+00 ; 1.218439e-01 ];
Tc_1  = [ -1.057978e+02 ; -2.924849e+02 ; 5.527991e+03 ];
omc_error_1 = [ 1.942670e-02 ; 1.843087e-02 ; 2.702768e-02 ];
Tc_error_1  = [ 1.093290e+02 ; 8.594096e+01 ; 6.589483e+01 ];

%-- Image #2:
omc_2 = [ -2.089319e+00 ; -1.662479e+00 ; -8.222125e-01 ];
Tc_2  = [ -8.541516e+01 ; -2.389219e+02 ; 5.484687e+03 ];
omc_error_2 = [ 1.035411e-02 ; 1.596823e-02 ; 2.336547e-02 ];
Tc_error_2  = [ 1.084557e+02 ; 8.526964e+01 ; 6.680670e+01 ];

%-- Image #3:
omc_3 = [ 2.023486e+00 ; 1.738723e+00 ; -4.230480e-01 ];
Tc_3  = [ -1.157070e+02 ; -2.169672e+02 ; 5.586026e+03 ];
omc_error_3 = [ 1.147230e-02 ; 1.561885e-02 ; 2.486453e-02 ];
Tc_error_3  = [ 1.104003e+02 ; 8.683587e+01 ; 6.694901e+01 ];

%-- Image #4:
omc_4 = [ 2.015232e+00 ; 1.928038e+00 ; 4.672905e-01 ];
Tc_4  = [ -1.467269e+02 ; -3.100272e+02 ; 5.499394e+03 ];
omc_error_4 = [ 1.614698e-02 ; 1.294798e-02 ; 2.200336e-02 ];
Tc_error_4  = [ 1.088019e+02 ; 8.550728e+01 ; 6.641365e+01 ];

%-- Image #5:
omc_5 = [ -1.827570e+00 ; -1.854806e+00 ; 5.178886e-01 ];
Tc_5  = [ 1.800131e+00 ; -2.551642e+02 ; 5.619558e+03 ];
omc_error_5 = [ 1.568506e-02 ; 1.240941e-02 ; 2.265311e-02 ];
Tc_error_5  = [ 1.110946e+02 ; 8.734645e+01 ; 6.597711e+01 ];

%-- Image #6:
omc_6 = [ 1.812742e+00 ; 1.777950e+00 ; -4.164241e-01 ];
Tc_6  = [ -1.169456e+02 ; -2.621371e+02 ; 5.586078e+03 ];
omc_error_6 = [ 1.107745e-02 ; 1.674271e-02 ; 2.284146e-02 ];
Tc_error_6  = [ 1.104223e+02 ; 8.684381e+01 ; 6.692277e+01 ];

%-- Image #7:
omc_7 = [ -2.140367e+00 ; -1.320836e+00 ; -1.523850e-01 ];
Tc_7  = [ -1.192585e+02 ; -2.162864e+02 ; 5.575891e+03 ];
omc_error_7 = [ 1.368982e-02 ; 1.222004e-02 ; 2.223322e-02 ];
Tc_error_7  = [ 1.101839e+02 ; 8.668085e+01 ; 6.692278e+01 ];

%-- Image #8:
omc_8 = [ 2.045885e+00 ; 1.970642e+00 ; -9.770995e-01 ];
Tc_8  = [ -1.377068e+02 ; -2.131848e+02 ; 5.684746e+03 ];
omc_error_8 = [ 8.354284e-03 ; 1.771336e-02 ; 2.723004e-02 ];
Tc_error_8  = [ 1.124072e+02 ; 8.839528e+01 ; 6.678160e+01 ];

%-- Image #9:
omc_9 = [ -2.059270e+00 ; -1.759514e+00 ; -8.368138e-01 ];
Tc_9  = [ -1.452565e+02 ; -2.556009e+02 ; 5.508251e+03 ];
omc_error_9 = [ 9.895357e-03 ; 1.621582e-02 ; 2.346821e-02 ];
Tc_error_9  = [ 1.089573e+02 ; 8.566506e+01 ; 6.712516e+01 ];

%-- Image #10:
omc_10 = [ -2.178768e+00 ; -1.840836e+00 ; 5.112877e-01 ];
Tc_10  = [ -1.412007e+02 ; -2.613423e+02 ; 5.640622e+03 ];
omc_error_10 = [ 1.708337e-02 ; 9.989548e-03 ; 2.651983e-02 ];
Tc_error_10  = [ 1.115625e+02 ; 8.769932e+01 ; 6.629382e+01 ];

%-- Image #11:
omc_11 = [ -2.047894e+00 ; -2.014471e+00 ; -3.397267e-01 ];
Tc_11  = [ -5.461384e+01 ; -3.132044e+02 ; 5.571683e+03 ];
omc_error_11 = [ 1.058455e-02 ; 1.438829e-02 ; 2.321493e-02 ];
Tc_error_11  = [ 1.102071e+02 ; 8.662526e+01 ; 6.777999e+01 ];

%-- Image #12:
omc_12 = [ 1.489966e+00 ; 2.297351e+00 ; -1.209762e+00 ];
Tc_12  = [ -1.934920e+01 ; -2.670282e+02 ; 5.654499e+03 ];
omc_error_12 = [ 7.354100e-03 ; 2.026368e-02 ; 2.484417e-02 ];
Tc_error_12  = [ 1.118007e+02 ; 8.790937e+01 ; 6.615923e+01 ];

%-- Image #13:
omc_13 = [ -1.326775e+00 ; -2.390890e+00 ; -7.209172e-01 ];
Tc_13  = [ -1.708393e+01 ; -3.110467e+02 ; 5.407928e+03 ];
omc_error_13 = [ 6.190715e-03 ; 1.864422e-02 ; 2.223201e-02 ];
Tc_error_13  = [ 1.069644e+02 ; 8.408143e+01 ; 6.632027e+01 ];

%-- Image #14:
omc_14 = [ -2.157209e+00 ; -2.100909e+00 ; 4.264612e-01 ];
Tc_14  = [ -1.507217e+02 ; -2.520716e+02 ; 5.599066e+03 ];
omc_error_14 = [ 1.702139e-02 ; 1.155609e-02 ; 2.851337e-02 ];
Tc_error_14  = [ 1.107532e+02 ; 8.706463e+01 ; 6.606436e+01 ];

%-- Image #15:
omc_15 = [ 1.625148e+00 ; 2.272324e+00 ; -9.965976e-01 ];
Tc_15  = [ -5.331273e+01 ; -2.662281e+02 ; 5.617174e+03 ];
omc_error_15 = [ 6.674719e-03 ; 1.948862e-02 ; 2.538197e-02 ];
Tc_error_15  = [ 1.110703e+02 ; 8.733065e+01 ; 6.600447e+01 ];

%-- Image #16:
omc_16 = [ 2.313326e+00 ; 1.805351e+00 ; 9.502440e-01 ];
Tc_16  = [ -1.376011e+02 ; -2.202556e+02 ; 5.384843e+03 ];
omc_error_16 = [ 2.021744e-02 ; 7.736178e-03 ; 2.360207e-02 ];
Tc_error_16  = [ 1.065093e+02 ; 8.373469e+01 ; 6.531488e+01 ];

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

