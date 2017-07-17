% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 7933.733650339129800 ; 8594.367151440046700 ];

%-- Principal point:
cc = [ -39.480760891855290 ; 265.677351133864530 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.082244393458675 ; 24.137895571133953 ; -0.004773160535985 ; -0.032085742451675 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 57.513408412214538 ; 63.028035164956343 ];

%-- Principal point uncertainty:
cc_error = [ 0.000000000000000 ; 0.000000000000000 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.317058079906550 ; 13.192115010067164 ; 0.001947821891971 ; 0.011082702130550 ; 0.000000000000000 ];

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
omc_1 = [ 2.183516e+00 ; 2.187559e+00 ; 1.357440e-01 ];
Tc_1  = [ 1.293490e+02 ; -8.814083e+01 ; 5.622443e+03 ];
omc_error_1 = [ 8.890308e-03 ; 8.701355e-03 ; 1.238024e-02 ];
Tc_error_1  = [ 3.186313e-01 ; 6.396105e-02 ; 4.118739e+01 ];

%-- Image #2:
omc_2 = [ 1.805668e+00 ; 1.691350e+00 ; -5.766447e-01 ];
Tc_2  = [ 1.455592e+02 ; -6.323027e+01 ; 5.656706e+03 ];
omc_error_2 = [ 1.413552e-03 ; 1.379000e-03 ; 2.434260e-03 ];
Tc_error_2  = [ 1.740266e-01 ; 1.454605e-01 ; 3.883636e+01 ];

%-- Image #3:
omc_3 = [ -1.950111e+00 ; -1.964161e+00 ; -7.655387e-01 ];
Tc_3  = [ 9.800819e+01 ; -3.427129e+01 ; 5.504670e+03 ];
omc_error_3 = [ 2.286118e-03 ; 2.311408e-03 ; 1.777890e-03 ];
Tc_error_3  = [ 1.234413e-01 ; 9.497956e-02 ; 3.888086e+01 ];

%-- Image #4:
omc_4 = [ 1.929040e+00 ; 1.848019e+00 ; 6.347142e-01 ];
Tc_4  = [ 8.267336e+01 ; -1.114242e+02 ; 5.503396e+03 ];
omc_error_4 = [ 1.921591e-03 ; 1.893225e-03 ; 2.484919e-03 ];
Tc_error_4  = [ 2.032898e-01 ; 5.750688e-02 ; 4.069485e+01 ];

%-- Image #5:
omc_5 = [ -1.906484e+00 ; -1.905814e+00 ; 4.366777e-01 ];
Tc_5  = [ 2.110072e+02 ; -7.384970e+01 ; 5.631118e+03 ];
omc_error_5 = [ 2.443107e-03 ; 2.496412e-03 ; 4.753917e-03 ];
Tc_error_5  = [ 4.015809e-01 ; 5.794158e-02 ; 4.004960e+01 ];

%-- Image #6:
omc_6 = [ 1.679482e+00 ; 1.712655e+00 ; -3.584117e-01 ];
Tc_6  = [ 1.372292e+02 ; -9.363779e+01 ; 5.581773e+03 ];
omc_error_6 = [ 7.902509e-04 ; 1.027489e-03 ; 2.849687e-03 ];
Tc_error_6  = [ 1.765775e-01 ; 1.683688e-01 ; 3.858039e+01 ];

%-- Image #7:
omc_7 = [ -2.014340e+00 ; -1.314673e+00 ; -1.970243e-01 ];
Tc_7  = [ 1.108816e+02 ; -3.605951e+01 ; 5.593938e+03 ];
omc_error_7 = [ 1.355403e-03 ; 1.201418e-03 ; 2.703796e-03 ];
Tc_error_7  = [ 1.623048e-01 ; 9.667184e-02 ; 3.861430e+01 ];

%-- Image #8:
omc_8 = [ -1.836390e+00 ; -1.989410e+00 ; -1.097267e+00 ];
Tc_8  = [ 1.598994e+02 ; -9.925992e+01 ; 5.487791e+03 ];
omc_error_8 = [ 1.743269e-03 ; 1.658776e-03 ; 1.137325e-03 ];
Tc_error_8  = [ 1.507888e-01 ; 1.792120e-01 ; 3.905146e+01 ];

%-- Image #9:
omc_9 = [ 1.708763e+00 ; 2.101691e+00 ; -1.121096e+00 ];
Tc_9  = [ 2.043826e+02 ; -1.337781e+02 ; 5.686677e+03 ];
omc_error_9 = [ 1.765247e-03 ; 1.784795e-03 ; 1.357309e-03 ];
Tc_error_9  = [ 1.692256e-01 ; 2.581784e-01 ; 3.868073e+01 ];

%-- Image #10:
omc_10 = [ -1.333470e+00 ; -2.190329e+00 ; -6.390282e-01 ];
Tc_10  = [ 1.779641e+02 ; -1.184533e+02 ; 5.471009e+03 ];
omc_error_10 = [ 8.426380e-04 ; 1.577490e-03 ; 3.345526e-03 ];
Tc_error_10  = [ 1.611439e-01 ; 2.493438e-01 ; 3.852561e+01 ];

%-- Image #11:
omc_11 = [ -1.438782e-01 ; 2.890820e+00 ; -1.097486e+00 ];
Tc_11  = [ 3.677412e+02 ; -9.967189e+01 ; 5.629916e+03 ];
omc_error_11 = [ 4.774899e-04 ; 1.197776e-03 ; 3.630154e-03 ];
Tc_error_11  = [ 1.819567e-01 ; 2.222592e-01 ; 3.839843e+01 ];

%-- Image #12:
omc_12 = [ -1.982661e+00 ; -1.607811e+00 ; -8.206816e-01 ];
Tc_12  = [ 1.006670e+02 ; -5.375859e+01 ; 5.545277e+03 ];
omc_error_12 = [ 1.877821e-03 ; 1.519949e-03 ; 1.269726e-03 ];
Tc_error_12  = [ 1.386003e-01 ; 1.289706e-01 ; 3.901305e+01 ];

%-- Image #13:
omc_13 = [ -1.408967e+00 ; -2.129696e+00 ; -6.994498e-01 ];
Tc_13  = [ 2.012774e+02 ; -1.037226e+02 ; 5.548875e+03 ];
omc_error_13 = [ 9.941556e-04 ; 1.682034e-03 ; 2.925461e-03 ];
Tc_error_13  = [ 1.747226e-01 ; 2.272960e-01 ; 3.909430e+01 ];

%-- Image #14:
omc_14 = [ 2.178432e+00 ; 1.590138e+00 ; 3.858285e-02 ];
Tc_14  = [ 1.018763e+02 ; -5.866098e+01 ; 5.554387e+03 ];
omc_error_14 = [ 1.195694e-03 ; 1.004558e-03 ; 5.876028e-03 ];
Tc_error_14  = [ 1.872015e-01 ; 8.660383e-02 ; 3.922128e+01 ];

%-- Image #15:
omc_15 = [ -1.968941e+00 ; -1.909488e+00 ; -7.478911e-01 ];
Tc_15  = [ 1.980924e+02 ; -1.056785e+02 ; 5.568227e+03 ];
omc_error_15 = [ 2.297423e-03 ; 2.287677e-03 ; 1.882815e-03 ];
Tc_error_15  = [ 1.672012e-01 ; 2.115385e-01 ; 3.931638e+01 ];

%-- Image #16:
omc_16 = [ -1.877379e+00 ; -1.897643e+00 ; -6.525005e-01 ];
Tc_16  = [ 8.122264e+01 ; -1.287765e+02 ; 5.584398e+03 ];
omc_error_16 = [ 2.058996e-03 ; 2.179466e-03 ; 2.418787e-03 ];
Tc_error_16  = [ 1.244774e-01 ; 2.794924e-01 ; 3.928916e+01 ];

%-- Image #17:
omc_17 = [ -1.993177e+00 ; -2.060318e+00 ; -9.167093e-01 ];
Tc_17  = [ 1.030420e+02 ; -1.780303e+01 ; 5.562047e+03 ];
omc_error_17 = [ 2.199272e-03 ; 2.161511e-03 ; 1.423138e-03 ];
Tc_error_17  = [ 1.249967e-01 ; 7.621227e-02 ; 3.963772e+01 ];

%-- Image #18:
omc_18 = [ 1.707451e+00 ; 2.179244e+00 ; -1.250806e+00 ];
Tc_18  = [ 1.845169e+02 ; -7.668910e+01 ; 5.712417e+03 ];
omc_error_18 = [ 1.742338e-03 ; 1.685094e-03 ; 1.158855e-03 ];
Tc_error_18  = [ 1.683207e-01 ; 1.427847e-01 ; 3.882907e+01 ];

%-- Image #19:
omc_19 = [ 2.366450e+00 ; 1.815701e+00 ; 7.849625e-01 ];
Tc_19  = [ 1.250410e+02 ; -4.241271e+01 ; 5.496697e+03 ];
omc_error_19 = [ 2.953961e-03 ; 2.301812e-03 ; 2.090230e-03 ];
Tc_error_19  = [ 2.314237e-01 ; 7.409870e-02 ; 3.989839e+01 ];

%-- Image #20:
omc_20 = [ 2.350248e+00 ; 8.308858e-01 ; -3.399129e-02 ];
Tc_20  = [ 1.374625e+02 ; 3.936351e+01 ; 5.495593e+03 ];
omc_error_20 = [ 2.095028e-03 ; 8.675342e-04 ; 2.745982e-03 ];
Tc_error_20  = [ 1.733836e-01 ; 1.065627e-01 ; 3.851016e+01 ];

%-- Image #21:
omc_21 = [ -1.947166e+00 ; -2.148925e+00 ; -6.307411e-01 ];
Tc_21  = [ 1.703605e+02 ; -8.440818e+01 ; 5.481681e+03 ];
omc_error_21 = [ 2.822695e-03 ; 3.241175e-03 ; 2.566848e-03 ];
Tc_error_21  = [ 1.498736e-01 ; 1.645353e-01 ; 3.862496e+01 ];

%-- Image #22:
omc_22 = [ 1.908525e+00 ; 1.760747e+00 ; 3.688890e-01 ];
Tc_22  = [ 1.030556e+02 ; -8.619675e+01 ; 5.498337e+03 ];
omc_error_22 = [ 1.398143e-03 ; 1.299758e-03 ; 3.674603e-03 ];
Tc_error_22  = [ 2.361543e-01 ; 5.623083e-02 ; 4.013110e+01 ];

%-- Image #23:
omc_23 = [ -1.628133e+00 ; -1.580394e+00 ; 3.822846e-01 ];
Tc_23  = [ 1.885329e+02 ; -1.088500e+02 ; 5.653224e+03 ];
omc_error_23 = [ 9.176333e-04 ; 8.323265e-04 ; 2.515723e-03 ];
Tc_error_23  = [ 3.242091e-01 ; 7.447881e-02 ; 3.969568e+01 ];

%-- Image #24:
omc_24 = [ -2.253052e+00 ; -1.871143e+00 ; 8.783663e-01 ];
Tc_24  = [ 1.754690e+02 ; -4.081078e+01 ; 5.721176e+03 ];
omc_error_24 = [ 2.918665e-03 ; 2.667170e-03 ; 1.987803e-03 ];
Tc_error_24  = [ 2.532738e-01 ; 7.462133e-02 ; 3.978920e+01 ];

%-- Image #25:
omc_25 = [ 1.406296e+00 ; 1.774541e+00 ; -2.626177e-01 ];
Tc_25  = [ 1.133502e+02 ; -1.234068e+02 ; 5.681892e+03 ];
omc_error_25 = [ 4.553367e-04 ; 7.273972e-04 ; 2.478684e-03 ];
Tc_error_25  = [ 1.734163e-01 ; 1.659927e-01 ; 3.955668e+01 ];

%-- Image #26:
omc_26 = [ -1.910887e+00 ; -2.165675e+00 ; -1.131362e+00 ];
Tc_26  = [ 1.442248e+02 ; -1.028985e+02 ; 5.508265e+03 ];
omc_error_26 = [ 1.833006e-03 ; 1.767934e-03 ; 1.251597e-03 ];
Tc_error_26  = [ 1.666642e-01 ; 1.475034e-01 ; 3.946472e+01 ];

%-- Image #27:
omc_27 = [ -1.595418e+00 ; -2.076963e+00 ; -4.593740e-01 ];
Tc_27  = [ 1.805151e+02 ; -1.189452e+02 ; 5.555202e+03 ];
omc_error_27 = [ 1.199364e-03 ; 1.847598e-03 ; 4.104343e-03 ];
Tc_error_27  = [ 1.752307e-01 ; 2.492310e-01 ; 3.888362e+01 ];

%-- Image #28:
omc_28 = [ 2.311817e+00 ; 1.517596e+00 ; -5.212216e-01 ];
Tc_28  = [ 9.602407e+01 ; -2.491778e+01 ; 5.596899e+03 ];
omc_error_28 = [ 3.548116e-03 ; 2.184971e-03 ; 2.508550e-03 ];
Tc_error_28  = [ 1.513337e-01 ; 8.560620e-02 ; 3.853695e+01 ];

%-- Image #29:
omc_29 = [ 1.750137e+00 ; 1.961714e+00 ; -1.095854e+00 ];
Tc_29  = [ 1.699137e+02 ; -7.013782e+01 ; 5.644948e+03 ];
omc_error_29 = [ 1.723316e-03 ; 1.596094e-03 ; 1.273223e-03 ];
Tc_error_29  = [ 1.623579e-01 ; 1.509025e-01 ; 3.834988e+01 ];

%-- Image #30:
omc_30 = [ -1.990807e+00 ; -1.680964e+00 ; -9.249391e-01 ];
Tc_30  = [ 8.754795e+01 ; -6.457285e+01 ; 5.536695e+03 ];
omc_error_30 = [ 1.895819e-03 ; 1.515811e-03 ; 1.143198e-03 ];
Tc_error_30  = [ 1.277681e-01 ; 1.394791e-01 ; 3.913543e+01 ];
