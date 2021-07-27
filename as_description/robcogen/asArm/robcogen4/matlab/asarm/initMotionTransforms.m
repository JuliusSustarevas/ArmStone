function tr = initMotionTransforms()

tr.fr_xarmlink1_XM_fr_base_link = zeros(6, 6);
tr.fr_xarmlink1_XM_fr_base_link(3,3) = 1.0;
tr.fr_xarmlink1_XM_fr_base_link(6,2) = - 0.175;
tr.fr_xarmlink1_XM_fr_base_link(6,6) = 1.0;

tr.fr_base_link_XM_fr_xarmlink1 = zeros(6, 6);
tr.fr_base_link_XM_fr_xarmlink1(3,3) = 1.0;
tr.fr_base_link_XM_fr_xarmlink1(5,3) = - 0.175;
tr.fr_base_link_XM_fr_xarmlink1(6,6) = 1.0;

tr.fr_xarmlink2_XM_fr_xarmlink1 = zeros(6, 6);
tr.fr_xarmlink2_XM_fr_xarmlink1(3,2) = 0.999999;
tr.fr_xarmlink2_XM_fr_xarmlink1(3,3) = - 3.6732E-6;
tr.fr_xarmlink2_XM_fr_xarmlink1(6,5) = 0.999999;
tr.fr_xarmlink2_XM_fr_xarmlink1(6,6) = - 3.6732E-6;

tr.fr_xarmlink1_XM_fr_xarmlink2 = zeros(6, 6);
tr.fr_xarmlink1_XM_fr_xarmlink2(2,3) = 0.999999;
tr.fr_xarmlink1_XM_fr_xarmlink2(3,3) = - 3.6732E-6;
tr.fr_xarmlink1_XM_fr_xarmlink2(5,6) = 0.999999;
tr.fr_xarmlink1_XM_fr_xarmlink2(6,6) = - 3.6732E-6;

tr.fr_xarmlink3_XM_fr_xarmlink2 = zeros(6, 6);
tr.fr_xarmlink3_XM_fr_xarmlink2(3,3) = 1.0;
tr.fr_xarmlink3_XM_fr_xarmlink2(6,1) = - 0.2845;
tr.fr_xarmlink3_XM_fr_xarmlink2(6,2) = - 0.0535;
tr.fr_xarmlink3_XM_fr_xarmlink2(6,6) = 1.0;

tr.fr_xarmlink2_XM_fr_xarmlink3 = zeros(6, 6);
tr.fr_xarmlink2_XM_fr_xarmlink3(3,3) = 1.0;
tr.fr_xarmlink2_XM_fr_xarmlink3(4,3) = - 0.2845;
tr.fr_xarmlink2_XM_fr_xarmlink3(5,3) = - 0.0535;
tr.fr_xarmlink2_XM_fr_xarmlink3(6,6) = 1.0;

tr.fr_xarmlink4_XM_fr_xarmlink3 = zeros(6, 6);
tr.fr_xarmlink4_XM_fr_xarmlink3(3,2) = 0.999999;
tr.fr_xarmlink4_XM_fr_xarmlink3(3,3) = - 3.6732E-6;
tr.fr_xarmlink4_XM_fr_xarmlink3(6,1) = - 1.25807E-6;
tr.fr_xarmlink4_XM_fr_xarmlink3(6,2) = 2.84673E-7;
tr.fr_xarmlink4_XM_fr_xarmlink3(6,3) = 0.0774999;
tr.fr_xarmlink4_XM_fr_xarmlink3(6,5) = 0.999999;
tr.fr_xarmlink4_XM_fr_xarmlink3(6,6) = - 3.6732E-6;

tr.fr_xarmlink3_XM_fr_xarmlink4 = zeros(6, 6);
tr.fr_xarmlink3_XM_fr_xarmlink4(2,3) = 0.999999;
tr.fr_xarmlink3_XM_fr_xarmlink4(3,3) = - 3.6732E-6;
tr.fr_xarmlink3_XM_fr_xarmlink4(4,3) = - 1.25807E-6;
tr.fr_xarmlink3_XM_fr_xarmlink4(5,3) = 2.84673E-7;
tr.fr_xarmlink3_XM_fr_xarmlink4(5,6) = 0.999999;
tr.fr_xarmlink3_XM_fr_xarmlink4(6,3) = 0.0774999;
tr.fr_xarmlink3_XM_fr_xarmlink4(6,6) = - 3.6732E-6;

tr.fr_xarmlink5_XM_fr_xarmlink4 = zeros(6, 6);
tr.fr_xarmlink5_XM_fr_xarmlink4(3,2) = - 0.999999;
tr.fr_xarmlink5_XM_fr_xarmlink4(3,3) = - 3.6732E-6;
tr.fr_xarmlink5_XM_fr_xarmlink4(6,5) = - 0.999999;
tr.fr_xarmlink5_XM_fr_xarmlink4(6,6) = - 3.6732E-6;

tr.fr_xarmlink4_XM_fr_xarmlink5 = zeros(6, 6);
tr.fr_xarmlink4_XM_fr_xarmlink5(2,3) = - 0.999999;
tr.fr_xarmlink4_XM_fr_xarmlink5(3,3) = - 3.6732E-6;
tr.fr_xarmlink4_XM_fr_xarmlink5(5,6) = - 0.999999;
tr.fr_xarmlink4_XM_fr_xarmlink5(6,6) = - 3.6732E-6;

tr.fr_xarmlink6_XM_fr_xarmlink5 = zeros(6, 6);
tr.fr_xarmlink6_XM_fr_xarmlink5(3,2) = 0.999999;
tr.fr_xarmlink6_XM_fr_xarmlink5(3,3) = - 3.6732E-6;
tr.fr_xarmlink6_XM_fr_xarmlink5(6,1) = - 3.563E-7;
tr.fr_xarmlink6_XM_fr_xarmlink5(6,2) = 2.79163E-7;
tr.fr_xarmlink6_XM_fr_xarmlink5(6,3) = 0.0759999;
tr.fr_xarmlink6_XM_fr_xarmlink5(6,5) = 0.999999;
tr.fr_xarmlink6_XM_fr_xarmlink5(6,6) = - 3.6732E-6;

tr.fr_xarmlink5_XM_fr_xarmlink6 = zeros(6, 6);
tr.fr_xarmlink5_XM_fr_xarmlink6(2,3) = 0.999999;
tr.fr_xarmlink5_XM_fr_xarmlink6(3,3) = - 3.6732E-6;
tr.fr_xarmlink5_XM_fr_xarmlink6(4,3) = - 3.563E-7;
tr.fr_xarmlink5_XM_fr_xarmlink6(5,3) = 2.79163E-7;
tr.fr_xarmlink5_XM_fr_xarmlink6(5,6) = 0.999999;
tr.fr_xarmlink5_XM_fr_xarmlink6(6,3) = 0.0759999;
tr.fr_xarmlink5_XM_fr_xarmlink6(6,6) = - 3.6732E-6;

