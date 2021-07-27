function out = updateForceTransforms(tr, q, params)

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


tr.fr_xarmlink1_XF_fr_base_link(1,1) =  c__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(1,2) =  s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(1,4) = - 0.214 *  s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(1,5) =  0.214 *  c__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(1,6) =  0.175 *  s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(2,1) = - s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(2,2) =  c__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(2,4) = - 0.214 *  c__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(2,5) = - 0.214 *  s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(2,6) =  0.175 *  c__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(4,4) =  c__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(4,5) =  s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(5,4) = - s__q_xarmjoint1;
tr.fr_xarmlink1_XF_fr_base_link(5,5) =  c__q_xarmjoint1;



tr.fr_base_link_XF_fr_xarmlink1(1,1) =  c__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(1,2) = - s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(1,4) = - 0.214 *  s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(1,5) = - 0.214 *  c__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(2,1) =  s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(2,2) =  c__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(2,4) =  0.214 *  c__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(2,5) = - 0.214 *  s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(3,4) =  0.175 *  s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(3,5) =  0.175 *  c__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(4,4) =  c__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(4,5) = - s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(5,4) =  s__q_xarmjoint1;
tr.fr_base_link_XF_fr_xarmlink1(5,5) =  c__q_xarmjoint1;



tr.fr_xarmlink2_XF_fr_xarmlink1(1,1) =  c__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(1,2) = - 3.6732E-6 *  s__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(1,3) = - 0.999999 *  s__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(2,1) = - s__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(2,2) = - 3.6732E-6 *  c__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(2,3) = - 0.999999 *  c__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(4,4) =  c__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(4,5) = - 3.6732E-6 *  s__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(4,6) = - 0.999999 *  s__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(5,4) = - s__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(5,5) = - 3.6732E-6 *  c__q_xarmjoint2;
tr.fr_xarmlink2_XF_fr_xarmlink1(5,6) = - 0.999999 *  c__q_xarmjoint2;



tr.fr_xarmlink1_XF_fr_xarmlink2(1,1) =  c__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(1,2) = - s__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(2,1) = - 3.6732E-6 *  s__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(2,2) = - 3.6732E-6 *  c__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(3,1) = - 0.999999 *  s__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(3,2) = - 0.999999 *  c__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(4,4) =  c__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(4,5) = - s__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(5,4) = - 3.6732E-6 *  s__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(5,5) = - 3.6732E-6 *  c__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(6,4) = - 0.999999 *  s__q_xarmjoint2;
tr.fr_xarmlink1_XF_fr_xarmlink2(6,5) = - 0.999999 *  c__q_xarmjoint2;



tr.fr_xarmlink3_XF_fr_xarmlink2(1,1) =  c__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(1,2) =  s__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(1,6) = ( 0.0535 *  s__q_xarmjoint3) + ( 0.2845 *  c__q_xarmjoint3);
tr.fr_xarmlink3_XF_fr_xarmlink2(2,1) = - s__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(2,2) =  c__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(2,6) = ( 0.0535 *  c__q_xarmjoint3) - ( 0.2845 *  s__q_xarmjoint3);
tr.fr_xarmlink3_XF_fr_xarmlink2(4,4) =  c__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(4,5) =  s__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(5,4) = - s__q_xarmjoint3;
tr.fr_xarmlink3_XF_fr_xarmlink2(5,5) =  c__q_xarmjoint3;



tr.fr_xarmlink2_XF_fr_xarmlink3(1,1) =  c__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(1,2) = - s__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(2,1) =  s__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(2,2) =  c__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(3,4) = ( 0.0535 *  s__q_xarmjoint3) + ( 0.2845 *  c__q_xarmjoint3);
tr.fr_xarmlink2_XF_fr_xarmlink3(3,5) = ( 0.0535 *  c__q_xarmjoint3) - ( 0.2845 *  s__q_xarmjoint3);
tr.fr_xarmlink2_XF_fr_xarmlink3(4,4) =  c__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(4,5) = - s__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(5,4) =  s__q_xarmjoint3;
tr.fr_xarmlink2_XF_fr_xarmlink3(5,5) =  c__q_xarmjoint3;



tr.fr_xarmlink4_XF_fr_xarmlink3(1,1) =  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(1,2) = - 3.6732E-6 *  s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(1,3) = - 0.999999 *  s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(1,4) = - 0.342499 *  s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(1,5) =  0.0774999 *  s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(1,6) = (- 2.84673E-7 *  s__q_xarmjoint4) - ( 0.3425 *  c__q_xarmjoint4);
tr.fr_xarmlink4_XF_fr_xarmlink3(2,1) = - s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(2,2) = - 3.6732E-6 *  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(2,3) = - 0.999999 *  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(2,4) = - 0.342499 *  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(2,5) =  0.0774999 *  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(2,6) = ( 0.3425 *  s__q_xarmjoint4) - ( 2.84673E-7 *  c__q_xarmjoint4);
tr.fr_xarmlink4_XF_fr_xarmlink3(4,4) =  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(4,5) = - 3.6732E-6 *  s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(4,6) = - 0.999999 *  s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(5,4) = - s__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(5,5) = - 3.6732E-6 *  c__q_xarmjoint4;
tr.fr_xarmlink4_XF_fr_xarmlink3(5,6) = - 0.999999 *  c__q_xarmjoint4;



tr.fr_xarmlink3_XF_fr_xarmlink4(1,1) =  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(1,2) = - s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(1,4) = - 0.342499 *  s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(1,5) = - 0.342499 *  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(2,1) = - 3.6732E-6 *  s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(2,2) = - 3.6732E-6 *  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(2,4) =  0.0774999 *  s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(2,5) =  0.0774999 *  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(3,1) = - 0.999999 *  s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(3,2) = - 0.999999 *  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(3,4) = (- 2.84673E-7 *  s__q_xarmjoint4) - ( 0.3425 *  c__q_xarmjoint4);
tr.fr_xarmlink3_XF_fr_xarmlink4(3,5) = ( 0.3425 *  s__q_xarmjoint4) - ( 2.84673E-7 *  c__q_xarmjoint4);
tr.fr_xarmlink3_XF_fr_xarmlink4(4,4) =  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(4,5) = - s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(5,4) = - 3.6732E-6 *  s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(5,5) = - 3.6732E-6 *  c__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(6,4) = - 0.999999 *  s__q_xarmjoint4;
tr.fr_xarmlink3_XF_fr_xarmlink4(6,5) = - 0.999999 *  c__q_xarmjoint4;



tr.fr_xarmlink5_XF_fr_xarmlink4(1,1) =  c__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(1,2) = - 3.6732E-6 *  s__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(1,3) =  0.999999 *  s__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(2,1) = - s__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(2,2) = - 3.6732E-6 *  c__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(2,3) =  0.999999 *  c__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(4,4) =  c__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(4,5) = - 3.6732E-6 *  s__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(4,6) =  0.999999 *  s__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(5,4) = - s__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(5,5) = - 3.6732E-6 *  c__q_xarmjoint5;
tr.fr_xarmlink5_XF_fr_xarmlink4(5,6) =  0.999999 *  c__q_xarmjoint5;



tr.fr_xarmlink4_XF_fr_xarmlink5(1,1) =  c__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(1,2) = - s__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(2,1) = - 3.6732E-6 *  s__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(2,2) = - 3.6732E-6 *  c__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(3,1) =  0.999999 *  s__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(3,2) =  0.999999 *  c__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(4,4) =  c__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(4,5) = - s__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(5,4) = - 3.6732E-6 *  s__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(5,5) = - 3.6732E-6 *  c__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(6,4) =  0.999999 *  s__q_xarmjoint5;
tr.fr_xarmlink4_XF_fr_xarmlink5(6,5) =  0.999999 *  c__q_xarmjoint5;



tr.fr_xarmlink6_XF_fr_xarmlink5(1,1) =  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(1,2) = - 3.6732E-6 *  s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(1,3) = - 0.999999 *  s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(1,4) = - 0.0969999 *  s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(1,5) =  0.0759999 *  s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(1,6) = (- 2.79163E-7 *  s__q_xarmjoint6) - ( 0.097 *  c__q_xarmjoint6);
tr.fr_xarmlink6_XF_fr_xarmlink5(2,1) = - s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(2,2) = - 3.6732E-6 *  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(2,3) = - 0.999999 *  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(2,4) = - 0.0969999 *  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(2,5) =  0.0759999 *  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(2,6) = ( 0.097 *  s__q_xarmjoint6) - ( 2.79163E-7 *  c__q_xarmjoint6);
tr.fr_xarmlink6_XF_fr_xarmlink5(4,4) =  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(4,5) = - 3.6732E-6 *  s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(4,6) = - 0.999999 *  s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(5,4) = - s__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(5,5) = - 3.6732E-6 *  c__q_xarmjoint6;
tr.fr_xarmlink6_XF_fr_xarmlink5(5,6) = - 0.999999 *  c__q_xarmjoint6;



tr.fr_xarmlink5_XF_fr_xarmlink6(1,1) =  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(1,2) = - s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(1,4) = - 0.0969999 *  s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(1,5) = - 0.0969999 *  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(2,1) = - 3.6732E-6 *  s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(2,2) = - 3.6732E-6 *  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(2,4) =  0.0759999 *  s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(2,5) =  0.0759999 *  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(3,1) = - 0.999999 *  s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(3,2) = - 0.999999 *  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(3,4) = (- 2.79163E-7 *  s__q_xarmjoint6) - ( 0.097 *  c__q_xarmjoint6);
tr.fr_xarmlink5_XF_fr_xarmlink6(3,5) = ( 0.097 *  s__q_xarmjoint6) - ( 2.79163E-7 *  c__q_xarmjoint6);
tr.fr_xarmlink5_XF_fr_xarmlink6(4,4) =  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(4,5) = - s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(5,4) = - 3.6732E-6 *  s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(5,5) = - 3.6732E-6 *  c__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(6,4) = - 0.999999 *  s__q_xarmjoint6;
tr.fr_xarmlink5_XF_fr_xarmlink6(6,5) = - 0.999999 *  c__q_xarmjoint6;





out = tr;