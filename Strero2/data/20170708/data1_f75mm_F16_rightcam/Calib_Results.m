% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 7992.443552479582600 ; 8661.311027181896900 ];

%-- Principal point:
cc = [ -28.056495087352140 ; 307.490235160155120 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.244357398025924 ; 29.153905680029737 ; 0.004731746648726 ; -0.028836018288317 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 51.910816490618849 ; 53.792405745374680 ];

%-- Principal point uncertainty:
cc_error = [ 0.000000000000000 ; 0.000000000000000 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.234644890542553 ; 10.530456517529444 ; 0.001307841675586 ; 0.007928405874812 ; 0.000000000000000 ];

%-- Image size:
nx = 720;
ny = 576;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 29;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 0;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.126660e+00 ; -2.195582e+00 ; 3.971182e-02 ];
Tc_1  = [ 9.598683e+01 ; -1.256765e+02 ; 5.650409e+03 ];
omc_error_1 = [ 4.956210e-03 ; 5.484116e-03 ; 1.681397e-02 ];
Tc_error_1  = [ 1.133686e-01 ; 6.241547e-02 ; 3.495673e+01 ];

%-- Image #2:
omc_2 = [ -1.792355e+00 ; -1.934112e+00 ; -7.294760e-01 ];
Tc_2  = [ 1.375701e+02 ; -1.261041e+02 ; 5.534873e+03 ];
omc_error_2 = [ 6.579328e-04 ; 7.990207e-04 ; 1.076659e-03 ];
Tc_error_2  = [ 1.013630e-01 ; 9.703870e-02 ; 3.460378e+01 ];

%-- Image #3:
omc_3 = [ 1.562482e+00 ; 1.832629e+00 ; -7.886666e-01 ];
Tc_3  = [ 9.866553e+01 ; -4.376287e+01 ; 5.681123e+03 ];
omc_error_3 = [ 4.583468e-04 ; 5.414900e-04 ; 8.752771e-04 ];
Tc_error_3  = [ 1.020573e-01 ; 5.729209e-02 ; 3.456525e+01 ];

%-- Image #4:
omc_4 = [ 1.814261e+00 ; 1.793885e+00 ; 8.571012e-01 ];
Tc_4  = [ 5.693719e+01 ; -1.333905e+02 ; 5.460548e+03 ];
omc_error_4 = [ 5.883084e-04 ; 6.122474e-04 ; 7.667936e-04 ];
Tc_error_4  = [ 7.322938e-02 ; 4.417675e-02 ; 3.424283e+01 ];

%-- Image #5:
omc_5 = [ -1.807101e+00 ; -1.935579e+00 ; 5.380702e-01 ];
Tc_5  = [ 2.188824e+02 ; -1.206700e+02 ; 5.669839e+03 ];
omc_error_5 = [ 8.267099e-04 ; 9.633599e-04 ; 1.398796e-03 ];
Tc_error_5  = [ 1.721794e-01 ; 3.966456e-02 ; 3.428136e+01 ];

%-- Image #6:
omc_6 = [ -1.771303e+00 ; -1.918721e+00 ; 5.490829e-01 ];
Tc_6  = [ 2.141237e+02 ; -1.213198e+02 ; 5.664432e+03 ];
omc_error_6 = [ 7.637958e-04 ; 9.035841e-04 ; 1.292334e-03 ];
Tc_error_6  = [ 1.704044e-01 ; 3.917665e-02 ; 3.423277e+01 ];

%-- Image #7:
omc_7 = [ -2.119345e+00 ; -1.740570e+00 ; -1.090381e+00 ];
Tc_7  = [ 1.454838e+02 ; -6.019347e+01 ; 5.497095e+03 ];
omc_error_7 = [ 6.871088e-04 ; 5.788552e-04 ; 9.059757e-04 ];
Tc_error_7  = [ 1.036378e-01 ; 5.894779e-02 ; 3.438347e+01 ];

%-- Image #8:
omc_8 = [ 2.057731e+00 ; 2.044871e+00 ; -1.014335e+00 ];
Tc_8  = [ 1.469967e+02 ; -9.033411e+01 ; 5.669644e+03 ];
omc_error_8 = [ 8.524844e-04 ; 7.322624e-04 ; 1.056586e-03 ];
Tc_error_8  = [ 1.152831e-01 ; 6.578743e-02 ; 3.411364e+01 ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ NaN ; NaN ; NaN ];
Tc_10  = [ NaN ; NaN ; NaN ];
omc_error_10 = [ NaN ; NaN ; NaN ];
Tc_error_10  = [ NaN ; NaN ; NaN ];

%-- Image #11:
omc_11 = [ 1.501541e+00 ; 1.852409e+00 ; -2.680461e-01 ];
Tc_11  = [ 1.725813e+02 ; -1.266337e+02 ; 5.628201e+03 ];
omc_error_11 = [ 3.680998e-04 ; 5.564109e-04 ; 1.096226e-03 ];
Tc_error_11  = [ 1.323714e-01 ; 6.686223e-02 ; 3.451796e+01 ];

%-- Image #12:
omc_12 = [ 1.705007e+00 ; 2.153352e+00 ; 4.703527e-01 ];
Tc_12  = [ 1.646580e+02 ; -1.542054e+02 ; 5.539258e+03 ];
omc_error_12 = [ 7.872933e-04 ; 1.028957e-03 ; 1.345307e-03 ];
Tc_error_12  = [ 1.367775e-01 ; 3.828146e-02 ; 3.430093e+01 ];

%-- Image #13:
omc_13 = [ 2.207719e+00 ; 2.050145e+00 ; 8.908457e-01 ];
Tc_13  = [ 1.025238e+02 ; -8.129014e+01 ; 5.467876e+03 ];
omc_error_13 = [ 9.192872e-04 ; 8.085910e-04 ; 1.139549e-03 ];
Tc_error_13  = [ 8.936547e-02 ; 5.734559e-02 ; 3.412528e+01 ];

%-- Image #14:
omc_14 = [ 1.459788e+00 ; 2.296069e+00 ; -1.714512e-01 ];
Tc_14  = [ 1.882977e+02 ; -1.729277e+02 ; 5.597519e+03 ];
omc_error_14 = [ 5.502613e-04 ; 9.173877e-04 ; 2.035602e-03 ];
Tc_error_14  = [ 1.357453e-01 ; 7.094996e-02 ; 3.431123e+01 ];

%-- Image #15:
omc_15 = [ 1.661847e+00 ; 2.193462e+00 ; 8.162849e-01 ];
Tc_15  = [ 1.769800e+02 ; -1.346224e+02 ; 5.485130e+03 ];
omc_error_15 = [ 6.668826e-04 ; 8.620838e-04 ; 9.535181e-04 ];
Tc_error_15  = [ 1.237727e-01 ; 4.966394e-02 ; 3.417429e+01 ];

%-- Image #16:
omc_16 = [ 1.875982e+00 ; 1.821552e+00 ; -9.708574e-01 ];
Tc_16  = [ 1.806587e+02 ; -1.245414e+02 ; 5.700408e+03 ];
omc_error_16 = [ 6.726243e-04 ; 6.353187e-04 ; 8.974050e-04 ];
Tc_error_16  = [ 1.278943e-01 ; 9.194887e-02 ; 3.449761e+01 ];

%-- Image #17:
omc_17 = [ -2.451994e+00 ; -1.903756e+00 ; -4.396113e-01 ];
Tc_17  = [ 1.164219e+02 ; -8.099069e+01 ; 5.552426e+03 ];
omc_error_17 = [ 1.628412e-03 ; 1.334679e-03 ; 2.515730e-03 ];
Tc_error_17  = [ 1.042455e-01 ; 5.707327e-02 ; 3.426446e+01 ];

%-- Image #18:
omc_18 = [ 1.766439e+00 ; 1.843852e+00 ; -2.321851e-01 ];
Tc_18  = [ 6.805820e+01 ; -1.179055e+02 ; 5.590756e+03 ];
omc_error_18 = [ 4.776390e-04 ; 6.095561e-04 ; 1.445934e-03 ];
Tc_error_18  = [ 8.958514e-02 ; 6.725377e-02 ; 3.445295e+01 ];

%-- Image #19:
omc_19 = [ 1.634978e+00 ; 2.437799e+00 ; 1.092804e+00 ];
Tc_19  = [ 1.916324e+02 ; -1.479049e+02 ; 5.453140e+03 ];
omc_error_19 = [ 6.030342e-04 ; 7.239203e-04 ; 1.034083e-03 ];
Tc_error_19  = [ 1.015011e-01 ; 8.401208e-02 ; 3.419395e+01 ];

%-- Image #20:
omc_20 = [ 1.123103e+00 ; 2.255540e+00 ; -5.017619e-01 ];
Tc_20  = [ 1.884734e+02 ; -1.641793e+02 ; 5.694205e+03 ];
omc_error_20 = [ 3.653816e-04 ; 6.248991e-04 ; 1.246488e-03 ];
Tc_error_20  = [ 1.237908e-01 ; 8.939235e-02 ; 3.463509e+01 ];

%-- Image #21:
omc_21 = [ 2.080239e+00 ; 2.066072e+00 ; 2.720179e-01 ];
Tc_21  = [ 7.403764e+01 ; -8.880387e+01 ; 5.556466e+03 ];
omc_error_21 = [ 1.431885e-03 ; 1.455622e-03 ; 2.719560e-03 ];
Tc_error_21  = [ 9.355117e-02 ; 3.919232e-02 ; 3.434994e+01 ];

%-- Image #22:
omc_22 = [ -2.224123e+00 ; -1.884091e+00 ; 7.446216e-01 ];
Tc_22  = [ 1.748877e+02 ; -6.722945e+01 ; 5.670528e+03 ];
omc_error_22 = [ 1.163172e-03 ; 1.035625e-03 ; 1.537512e-03 ];
Tc_error_22  = [ 1.345878e-01 ; 5.034541e-02 ; 3.416175e+01 ];

%-- Image #23:
omc_23 = [ -2.083665e+00 ; -2.054357e+00 ; 3.774012e-01 ];
Tc_23  = [ 1.797015e+02 ; -8.600069e+01 ; 5.660295e+03 ];
omc_error_23 = [ 1.798351e-03 ; 1.889561e-03 ; 2.991194e-03 ];
Tc_error_23  = [ 1.413593e-01 ; 4.234622e-02 ; 3.438240e+01 ];

%-- Image #24:
omc_24 = [ NaN ; NaN ; NaN ];
Tc_24  = [ NaN ; NaN ; NaN ];
omc_error_24 = [ NaN ; NaN ; NaN ];
Tc_error_24  = [ NaN ; NaN ; NaN ];

%-- Image #25:
omc_25 = [ -2.246181e+00 ; -2.144236e+00 ; -6.500580e-02 ];
Tc_25  = [ 8.035980e+01 ; -1.387186e+02 ; 5.625751e+03 ];
omc_error_25 = [ 6.596703e-03 ; 6.564350e-03 ; 9.682071e-03 ];
Tc_error_25  = [ 9.060730e-02 ; 8.289768e-02 ; 3.459301e+01 ];

%-- Image #26:
omc_26 = [ 1.813668e+00 ; 2.105912e+00 ; 5.493942e-02 ];
Tc_26  = [ 1.536691e+02 ; -1.344205e+02 ; 5.576198e+03 ];
omc_error_26 = [ 7.959611e-04 ; 1.018056e-03 ; 2.349655e-03 ];
Tc_error_26  = [ 1.348717e-01 ; 4.399234e-02 ; 3.435462e+01 ];

%-- Image #27:
omc_27 = [ 2.148259e+00 ; 1.787369e+00 ; 7.435483e-01 ];
Tc_27  = [ 1.168555e+02 ; -7.654396e+01 ; 5.478666e+03 ];
omc_error_27 = [ 8.818094e-04 ; 7.490835e-04 ; 1.171330e-03 ];
Tc_error_27  = [ 1.098758e-01 ; 4.610494e-02 ; 3.413827e+01 ];

%-- Image #28:
omc_28 = [ NaN ; NaN ; NaN ];
Tc_28  = [ NaN ; NaN ; NaN ];
omc_error_28 = [ NaN ; NaN ; NaN ];
Tc_error_28  = [ NaN ; NaN ; NaN ];

%-- Image #29:
omc_29 = [ 2.035510e+00 ; 1.671215e+00 ; -3.987640e-01 ];
Tc_29  = [ 1.117397e+02 ; -8.996650e+01 ; 5.596823e+03 ];
omc_error_29 = [ 7.616954e-04 ; 6.999377e-04 ; 1.440395e-03 ];
Tc_error_29  = [ 1.114378e-01 ; 7.129107e-02 ; 3.436541e+01 ];

