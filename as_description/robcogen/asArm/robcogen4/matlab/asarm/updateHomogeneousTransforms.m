function out = updateHomogeneousTransforms(tr, q, params)

s__q_xarmjoint1 = sin( q(1));
s__q_xarmjoint2 = sin( q(2));
s__q_xarmjoint3 = sin( q(3));
s__q_xarmjoint4 = sin( q(4));
s__q_xarmjoint5 = sin( q(5));
s__q_xarmjoint6 = sin( q(6));
c__q_xarmjoint1 = cos( q(1));
c__q_xarmjoint2 = cos( q(2));
c__q_xarmjoint3 = cos( q(3));
c__q_xarmjoint4 = cos( q(4));
c__q_xarmjoint5 = cos( q(5));
c__q_xarmjoint6 = cos( q(6));


tr.fr_xarmlink1_Xh_fr_base_link(1,1) =  c__q_xarmjoint1;
tr.fr_xarmlink1_Xh_fr_base_link(1,2) =  s__q_xarmjoint1;
tr.fr_xarmlink1_Xh_fr_base_link(1,4) = - 0.175 *  c__q_xarmjoint1;
tr.fr_xarmlink1_Xh_fr_base_link(2,1) = - s__q_xarmjoint1;
tr.fr_xarmlink1_Xh_fr_base_link(2,2) =  c__q_xarmjoint1;
tr.fr_xarmlink1_Xh_fr_base_link(2,4) =  0.175 *  s__q_xarmjoint1;



tr.fr_base_link_Xh_fr_xarmlink1(1,1) =  c__q_xarmjoint1;
tr.fr_base_link_Xh_fr_xarmlink1(1,2) = - s__q_xarmjoint1;
tr.fr_base_link_Xh_fr_xarmlink1(2,1) =  s__q_xarmjoint1;
tr.fr_base_link_Xh_fr_xarmlink1(2,2) =  c__q_xarmjoint1;



tr.fr_xarmlink2_Xh_fr_xarmlink1(1,1) =  c__q_xarmjoint2;
tr.fr_xarmlink2_Xh_fr_xarmlink1(1,2) = - 3.6732E-6 *  s__q_xarmjoint2;
tr.fr_xarmlink2_Xh_fr_xarmlink1(1,3) = - 0.999999 *  s__q_xarmjoint2;
tr.fr_xarmlink2_Xh_fr_xarmlink1(2,1) = - s__q_xarmjoint2;
tr.fr_xarmlink2_Xh_fr_xarmlink1(2,2) = - 3.6732E-6 *  c__q_xarmjoint2;
tr.fr_xarmlink2_Xh_fr_xarmlink1(2,3) = - 0.999999 *  c__q_xarmjoint2;



tr.fr_xarmlink1_Xh_fr_xarmlink2(1,1) =  c__q_xarmjoint2;
tr.fr_xarmlink1_Xh_fr_xarmlink2(1,2) = - s__q_xarmjoint2;
tr.fr_xarmlink1_Xh_fr_xarmlink2(2,1) = - 3.6732E-6 *  s__q_xarmjoint2;
tr.fr_xarmlink1_Xh_fr_xarmlink2(2,2) = - 3.6732E-6 *  c__q_xarmjoint2;
tr.fr_xarmlink1_Xh_fr_xarmlink2(3,1) = - 0.999999 *  s__q_xarmjoint2;
tr.fr_xarmlink1_Xh_fr_xarmlink2(3,2) = - 0.999999 *  c__q_xarmjoint2;



tr.fr_xarmlink3_Xh_fr_xarmlink2(1,1) =  c__q_xarmjoint3;
tr.fr_xarmlink3_Xh_fr_xarmlink2(1,2) =  s__q_xarmjoint3;
tr.fr_xarmlink3_Xh_fr_xarmlink2(1,4) = ( 0.2845 *  s__q_xarmjoint3) - ( 0.0535 *  c__q_xarmjoint3);
tr.fr_xarmlink3_Xh_fr_xarmlink2(2,1) = - s__q_xarmjoint3;
tr.fr_xarmlink3_Xh_fr_xarmlink2(2,2) =  c__q_xarmjoint3;
tr.fr_xarmlink3_Xh_fr_xarmlink2(2,4) = ( 0.0535 *  s__q_xarmjoint3) + ( 0.2845 *  c__q_xarmjoint3);



tr.fr_xarmlink2_Xh_fr_xarmlink3(1,1) =  c__q_xarmjoint3;
tr.fr_xarmlink2_Xh_fr_xarmlink3(1,2) = - s__q_xarmjoint3;
tr.fr_xarmlink2_Xh_fr_xarmlink3(2,1) =  s__q_xarmjoint3;
tr.fr_xarmlink2_Xh_fr_xarmlink3(2,2) =  c__q_xarmjoint3;



tr.fr_xarmlink4_Xh_fr_xarmlink3(1,1) =  c__q_xarmjoint4;
tr.fr_xarmlink4_Xh_fr_xarmlink3(1,2) = - 3.6732E-6 *  s__q_xarmjoint4;
tr.fr_xarmlink4_Xh_fr_xarmlink3(1,3) = - 0.999999 *  s__q_xarmjoint4;
tr.fr_xarmlink4_Xh_fr_xarmlink3(1,4) = ( 1.25807E-6 *  s__q_xarmjoint4) - ( 0.0775 *  c__q_xarmjoint4);
tr.fr_xarmlink4_Xh_fr_xarmlink3(2,1) = - s__q_xarmjoint4;
tr.fr_xarmlink4_Xh_fr_xarmlink3(2,2) = - 3.6732E-6 *  c__q_xarmjoint4;
tr.fr_xarmlink4_Xh_fr_xarmlink3(2,3) = - 0.999999 *  c__q_xarmjoint4;
tr.fr_xarmlink4_Xh_fr_xarmlink3(2,4) = ( 0.0775 *  s__q_xarmjoint4) + ( 1.25807E-6 *  c__q_xarmjoint4);



tr.fr_xarmlink3_Xh_fr_xarmlink4(1,1) =  c__q_xarmjoint4;
tr.fr_xarmlink3_Xh_fr_xarmlink4(1,2) = - s__q_xarmjoint4;
tr.fr_xarmlink3_Xh_fr_xarmlink4(2,1) = - 3.6732E-6 *  s__q_xarmjoint4;
tr.fr_xarmlink3_Xh_fr_xarmlink4(2,2) = - 3.6732E-6 *  c__q_xarmjoint4;
tr.fr_xarmlink3_Xh_fr_xarmlink4(3,1) = - 0.999999 *  s__q_xarmjoint4;
tr.fr_xarmlink3_Xh_fr_xarmlink4(3,2) = - 0.999999 *  c__q_xarmjoint4;



tr.fr_xarmlink5_Xh_fr_xarmlink4(1,1) =  c__q_xarmjoint5;
tr.fr_xarmlink5_Xh_fr_xarmlink4(1,2) = - 3.6732E-6 *  s__q_xarmjoint5;
tr.fr_xarmlink5_Xh_fr_xarmlink4(1,3) =  0.999999 *  s__q_xarmjoint5;
tr.fr_xarmlink5_Xh_fr_xarmlink4(2,1) = - s__q_xarmjoint5;
tr.fr_xarmlink5_Xh_fr_xarmlink4(2,2) = - 3.6732E-6 *  c__q_xarmjoint5;
tr.fr_xarmlink5_Xh_fr_xarmlink4(2,3) =  0.999999 *  c__q_xarmjoint5;



tr.fr_xarmlink4_Xh_fr_xarmlink5(1,1) =  c__q_xarmjoint5;
tr.fr_xarmlink4_Xh_fr_xarmlink5(1,2) = - s__q_xarmjoint5;
tr.fr_xarmlink4_Xh_fr_xarmlink5(2,1) = - 3.6732E-6 *  s__q_xarmjoint5;
tr.fr_xarmlink4_Xh_fr_xarmlink5(2,2) = - 3.6732E-6 *  c__q_xarmjoint5;
tr.fr_xarmlink4_Xh_fr_xarmlink5(3,1) =  0.999999 *  s__q_xarmjoint5;
tr.fr_xarmlink4_Xh_fr_xarmlink5(3,2) =  0.999999 *  c__q_xarmjoint5;



tr.fr_xarmlink6_Xh_fr_xarmlink5(1,1) =  c__q_xarmjoint6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(1,2) = - 3.6732E-6 *  s__q_xarmjoint6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(1,3) = - 0.999999 *  s__q_xarmjoint6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(1,4) = ( 3.563E-7 *  s__q_xarmjoint6) - ( 0.076 *  c__q_xarmjoint6);
tr.fr_xarmlink6_Xh_fr_xarmlink5(2,1) = - s__q_xarmjoint6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(2,2) = - 3.6732E-6 *  c__q_xarmjoint6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(2,3) = - 0.999999 *  c__q_xarmjoint6;
tr.fr_xarmlink6_Xh_fr_xarmlink5(2,4) = ( 0.076 *  s__q_xarmjoint6) + ( 3.563E-7 *  c__q_xarmjoint6);



tr.fr_xarmlink5_Xh_fr_xarmlink6(1,1) =  c__q_xarmjoint6;
tr.fr_xarmlink5_Xh_fr_xarmlink6(1,2) = - s__q_xarmjoint6;
tr.fr_xarmlink5_Xh_fr_xarmlink6(2,1) = - 3.6732E-6 *  s__q_xarmjoint6;
tr.fr_xarmlink5_Xh_fr_xarmlink6(2,2) = - 3.6732E-6 *  c__q_xarmjoint6;
tr.fr_xarmlink5_Xh_fr_xarmlink6(3,1) = - 0.999999 *  s__q_xarmjoint6;
tr.fr_xarmlink5_Xh_fr_xarmlink6(3,2) = - 0.999999 *  c__q_xarmjoint6;





out = tr;