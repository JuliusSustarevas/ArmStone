function tr = initForceTransforms()

tr.fr_xarmlink1_XF_fr_base_link = zeros(6, 6);
tr.fr_xarmlink1_XF_fr_base_link(3,3) = 1.0;
tr.fr_xarmlink1_XF_fr_base_link(3,5) = - 0.175;
tr.fr_xarmlink1_XF_fr_base_link(6,6) = 1.0;

tr.fr_base_link_XF_fr_xarmlink1 = zeros(6, 6);
tr.fr_base_link_XF_fr_xarmlink1(2,6) = - 0.175;
tr.fr_base_link_XF_fr_xarmlink1(3,3) = 1.0;
tr.fr_base_link_XF_fr_xarmlink1(6,6) = 1.0;

tr.fr_xarmlink2_XF_fr_xarmlink1 = zeros(6, 6);
tr.fr_xarmlink2_XF_fr_xarmlink1(3,2) = 0.999999;
tr.fr_xarmlink2_XF_fr_xarmlink1(3,3) = - 3.6732E-6;
tr.fr_xarmlink2_XF_fr_xarmlink1(6,5) = 0.999999;
tr.fr_xarmlink2_XF_fr_xarmlink1(6,6) = - 3.6732E-6;

tr.fr_xarmlink1_XF_fr_xarmlink2 = zeros(6, 6);
tr.fr_xarmlink1_XF_fr_xarmlink2(2,3) = 0.999999;
tr.fr_xarmlink1_XF_fr_xarmlink2(3,3) = - 3.6732E-6;
tr.fr_xarmlink1_XF_fr_xarmlink2(5,6) = 0.999999;
tr.fr_xarmlink1_XF_fr_xarmlink2(6,6) = - 3.6732E-6;

tr.fr_xarmlink3_XF_fr_xarmlink2 = zeros(6, 6);
tr.fr_xarmlink3_XF_fr_xarmlink2(3,3) = 1.0;
tr.fr_xarmlink3_XF_fr_xarmlink2(3,4) = - 0.2845;
tr.fr_xarmlink3_XF_fr_xarmlink2(3,5) = - 0.0535;
tr.fr_xarmlink3_XF_fr_xarmlink2(6,6) = 1.0;

tr.fr_xarmlink2_XF_fr_xarmlink3 = zeros(6, 6);
tr.fr_xarmlink2_XF_fr_xarmlink3(1,6) = - 0.2845;
tr.fr_xarmlink2_XF_fr_xarmlink3(2,6) = - 0.0535;
tr.fr_xarmlink2_XF_fr_xarmlink3(3,3) = 1.0;
tr.fr_xarmlink2_XF_fr_xarmlink3(6,6) = 1.0;

tr.fr_xarmlink4_XF_fr_xarmlink3 = zeros(6, 6);
tr.fr_xarmlink4_XF_fr_xarmlink3(3,2) = 0.999999;
tr.fr_xarmlink4_XF_fr_xarmlink3(3,3) = - 3.6732E-6;
tr.fr_xarmlink4_XF_fr_xarmlink3(3,4) = - 1.25807E-6;
tr.fr_xarmlink4_XF_fr_xarmlink3(3,5) = 2.84673E-7;
tr.fr_xarmlink4_XF_fr_xarmlink3(3,6) = 0.0774999;
tr.fr_xarmlink4_XF_fr_xarmlink3(6,5) = 0.999999;
tr.fr_xarmlink4_XF_fr_xarmlink3(6,6) = - 3.6732E-6;

tr.fr_xarmlink3_XF_fr_xarmlink4 = zeros(6, 6);
tr.fr_xarmlink3_XF_fr_xarmlink4(1,6) = - 1.25807E-6;
tr.fr_xarmlink3_XF_fr_xarmlink4(2,3) = 0.999999;
tr.fr_xarmlink3_XF_fr_xarmlink4(2,6) = 2.84673E-7;
tr.fr_xarmlink3_XF_fr_xarmlink4(3,3) = - 3.6732E-6;
tr.fr_xarmlink3_XF_fr_xarmlink4(3,6) = 0.0774999;
tr.fr_xarmlink3_XF_fr_xarmlink4(5,6) = 0.999999;
tr.fr_xarmlink3_XF_fr_xarmlink4(6,6) = - 3.6732E-6;

tr.fr_xarmlink5_XF_fr_xarmlink4 = zeros(6, 6);
tr.fr_xarmlink5_XF_fr_xarmlink4(3,2) = - 0.999999;
tr.fr_xarmlink5_XF_fr_xarmlink4(3,3) = - 3.6732E-6;
tr.fr_xarmlink5_XF_fr_xarmlink4(6,5) = - 0.999999;
tr.fr_xarmlink5_XF_fr_xarmlink4(6,6) = - 3.6732E-6;

tr.fr_xarmlink4_XF_fr_xarmlink5 = zeros(6, 6);
tr.fr_xarmlink4_XF_fr_xarmlink5(2,3) = - 0.999999;
tr.fr_xarmlink4_XF_fr_xarmlink5(3,3) = - 3.6732E-6;
tr.fr_xarmlink4_XF_fr_xarmlink5(5,6) = - 0.999999;
tr.fr_xarmlink4_XF_fr_xarmlink5(6,6) = - 3.6732E-6;

tr.fr_xarmlink6_XF_fr_xarmlink5 = zeros(6, 6);
tr.fr_xarmlink6_XF_fr_xarmlink5(3,2) = 0.999999;
tr.fr_xarmlink6_XF_fr_xarmlink5(3,3) = - 3.6732E-6;
tr.fr_xarmlink6_XF_fr_xarmlink5(3,4) = - 3.563E-7;
tr.fr_xarmlink6_XF_fr_xarmlink5(3,5) = 2.79163E-7;
tr.fr_xarmlink6_XF_fr_xarmlink5(3,6) = 0.0759999;
tr.fr_xarmlink6_XF_fr_xarmlink5(6,5) = 0.999999;
tr.fr_xarmlink6_XF_fr_xarmlink5(6,6) = - 3.6732E-6;

tr.fr_xarmlink5_XF_fr_xarmlink6 = zeros(6, 6);
tr.fr_xarmlink5_XF_fr_xarmlink6(1,6) = - 3.563E-7;
tr.fr_xarmlink5_XF_fr_xarmlink6(2,3) = 0.999999;
tr.fr_xarmlink5_XF_fr_xarmlink6(2,6) = 2.79163E-7;
tr.fr_xarmlink5_XF_fr_xarmlink6(3,3) = - 3.6732E-6;
tr.fr_xarmlink5_XF_fr_xarmlink6(3,6) = 0.0759999;
tr.fr_xarmlink5_XF_fr_xarmlink6(5,6) = 0.999999;
tr.fr_xarmlink5_XF_fr_xarmlink6(6,6) = - 3.6732E-6;

