function tr = initHomogeneousTransforms()

tr.fr_xarmlink1_Xh_fr_base_link = zeros(4, 4);
tr.fr_xarmlink1_Xh_fr_base_link(3,3) = 1.0;
tr.fr_xarmlink1_Xh_fr_base_link(3,4) = - 0.214;
tr.fr_xarmlink1_Xh_fr_base_link(4,4) = 1.0;

tr.fr_base_link_Xh_fr_xarmlink1 = zeros(4, 4);
tr.fr_base_link_Xh_fr_xarmlink1(1,4) = 0.175;
tr.fr_base_link_Xh_fr_xarmlink1(3,3) = 1.0;
tr.fr_base_link_Xh_fr_xarmlink1(3,4) = 0.214;
tr.fr_base_link_Xh_fr_xarmlink1(4,4) = 1.0;

tr.fr_xarmlink2_Xh_fr_xarmlink1 = zeros(4, 4);
tr.fr_xarmlink2_Xh_fr_xarmlink1(3,2) = 0.999999;
tr.fr_xarmlink2_Xh_fr_xarmlink1(3,3) = - 3.6732E-6;
tr.fr_xarmlink2_Xh_fr_xarmlink1(4,4) = 1;

tr.fr_xarmlink1_Xh_fr_xarmlink2 = zeros(4, 4);
tr.fr_xarmlink1_Xh_fr_xarmlink2(2,3) = 0.999999;
tr.fr_xarmlink1_Xh_fr_xarmlink2(3,3) = - 3.6732E-6;
tr.fr_xarmlink1_Xh_fr_xarmlink2(4,4) = 1;

tr.fr_xarmlink3_Xh_fr_xarmlink2 = zeros(4, 4);
tr.fr_xarmlink3_Xh_fr_xarmlink2(3,3) = 1.0;
tr.fr_xarmlink3_Xh_fr_xarmlink2(4,4) = 1.0;

tr.fr_xarmlink2_Xh_fr_xarmlink3 = zeros(4, 4);
tr.fr_xarmlink2_Xh_fr_xarmlink3(1,4) = 0.0535;
tr.fr_xarmlink2_Xh_fr_xarmlink3(2,4) = - 0.2845;
tr.fr_xarmlink2_Xh_fr_xarmlink3(3,3) = 1.0;
tr.fr_xarmlink2_Xh_fr_xarmlink3(4,4) = 1.0;

tr.fr_xarmlink4_Xh_fr_xarmlink3 = zeros(4, 4);
tr.fr_xarmlink4_Xh_fr_xarmlink3(3,2) = 0.999999;
tr.fr_xarmlink4_Xh_fr_xarmlink3(3,3) = - 3.6732E-6;
tr.fr_xarmlink4_Xh_fr_xarmlink3(3,4) = - 0.342499;
tr.fr_xarmlink4_Xh_fr_xarmlink3(4,4) = 1.0;

tr.fr_xarmlink3_Xh_fr_xarmlink4 = zeros(4, 4);
tr.fr_xarmlink3_Xh_fr_xarmlink4(1,4) = 0.0775;
tr.fr_xarmlink3_Xh_fr_xarmlink4(2,3) = 0.999999;
tr.fr_xarmlink3_Xh_fr_xarmlink4(2,4) = 0.3425;
tr.fr_xarmlink3_Xh_fr_xarmlink4(3,3) = - 3.6732E-6;
tr.fr_xarmlink3_Xh_fr_xarmlink4(4,4) = 1.0;

tr.fr_xarmlink5_Xh_fr_xarmlink4 = zeros(4, 4);
tr.fr_xarmlink5_Xh_fr_xarmlink4(3,2) = - 0.999999;
tr.fr_xarmlink5_Xh_fr_xarmlink4(3,3) = - 3.6732E-6;
tr.fr_xarmlink5_Xh_fr_xarmlink4(4,4) = 1;

tr.fr_xarmlink4_Xh_fr_xarmlink5 = zeros(4, 4);
tr.fr_xarmlink4_Xh_fr_xarmlink5(2,3) = - 0.999999;
tr.fr_xarmlink4_Xh_fr_xarmlink5(3,3) = - 3.6732E-6;
tr.fr_xarmlink4_Xh_fr_xarmlink5(4,4) = 1;

tr.fr_xarmlink6_Xh_fr_xarmlink5 = zeros(4, 4);
tr.fr_xarmlink6_Xh_fr_xarmlink5(3,2) = 0.999999;
tr.fr_xarmlink6_Xh_fr_xarmlink5(3,3) = - 3.6732E-6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(3,4) = - 0.0969999;
tr.fr_xarmlink6_Xh_fr_xarmlink5(4,4) = 1.0;

tr.fr_xarmlink5_Xh_fr_xarmlink6 = zeros(4, 4);
tr.fr_xarmlink5_Xh_fr_xarmlink6(1,4) = 0.076;
tr.fr_xarmlink5_Xh_fr_xarmlink6(2,3) = 0.999999;
tr.fr_xarmlink5_Xh_fr_xarmlink6(2,4) = 0.097;
tr.fr_xarmlink5_Xh_fr_xarmlink6(3,3) = - 3.6732E-6;
tr.fr_xarmlink5_Xh_fr_xarmlink6(4,4) = 1.0;

