function [H Ic_base_link F] = updateJSIM(inertia_props, force_transforms)

% Initialization of the composite-inertia matrices
Ic_xarmlink1 = inertia_props.lf_xarmlink1.tensor6D;
Ic_xarmlink2 = inertia_props.lf_xarmlink2.tensor6D;
Ic_xarmlink3 = inertia_props.lf_xarmlink3.tensor6D;
Ic_xarmlink4 = inertia_props.lf_xarmlink4.tensor6D;
Ic_xarmlink5 = inertia_props.lf_xarmlink5.tensor6D;
Ic_xarmlink6 = inertia_props.lf_xarmlink6.tensor6D;
Ic_base_link = inertia_props.lf_base_link.tensor6D;

% "Bottom-up" loop to update the inertia-composite property of each link,
%  for the current configuration

% Link xarmlink6
Ic_xarmlink5 = Ic_xarmlink5 + force_transforms.fr_xarmlink5_XF_fr_xarmlink6 * Ic_xarmlink6 * force_transforms.fr_xarmlink5_XF_fr_xarmlink6';

F(:,6) = Ic_xarmlink6(:,3);
H(6, 6) = F(3,6);

F(:,6) = force_transforms.fr_xarmlink5_XF_fr_xarmlink6 * F(:,6);
H(5, 6) = H(6, 5) = F(3,6);

F(:,6) = force_transforms.fr_xarmlink4_XF_fr_xarmlink5 * F(:,6);
H(4, 6) = H(6, 4) = F(3,6);

F(:,6) = force_transforms.fr_xarmlink3_XF_fr_xarmlink4 * F(:,6);
H(3, 6) = H(6, 3) = F(3,6);

F(:,6) = force_transforms.fr_xarmlink2_XF_fr_xarmlink3 * F(:,6);
H(2, 6) = H(6, 2) = F(3,6);

F(:,6) = force_transforms.fr_xarmlink1_XF_fr_xarmlink2 * F(:,6);
H(1, 6) = H(6, 1) = F(3,6);

F(:,6) = force_transforms.fr_base_link_XF_fr_xarmlink1 * F(:,6);

% Link xarmlink5
Ic_xarmlink4 = Ic_xarmlink4 + force_transforms.fr_xarmlink4_XF_fr_xarmlink5 * Ic_xarmlink5 * force_transforms.fr_xarmlink4_XF_fr_xarmlink5';

F(:,5) = Ic_xarmlink5(:,3);
H(5, 5) = F(3,5);

F(:,5) = force_transforms.fr_xarmlink4_XF_fr_xarmlink5 * F(:,5);
H(4, 5) = H(5, 4) = F(3,5);

F(:,5) = force_transforms.fr_xarmlink3_XF_fr_xarmlink4 * F(:,5);
H(3, 5) = H(5, 3) = F(3,5);

F(:,5) = force_transforms.fr_xarmlink2_XF_fr_xarmlink3 * F(:,5);
H(2, 5) = H(5, 2) = F(3,5);

F(:,5) = force_transforms.fr_xarmlink1_XF_fr_xarmlink2 * F(:,5);
H(1, 5) = H(5, 1) = F(3,5);

F(:,5) = force_transforms.fr_base_link_XF_fr_xarmlink1 * F(:,5);

% Link xarmlink4
Ic_xarmlink3 = Ic_xarmlink3 + force_transforms.fr_xarmlink3_XF_fr_xarmlink4 * Ic_xarmlink4 * force_transforms.fr_xarmlink3_XF_fr_xarmlink4';

F(:,4) = Ic_xarmlink4(:,3);
H(4, 4) = F(3,4);

F(:,4) = force_transforms.fr_xarmlink3_XF_fr_xarmlink4 * F(:,4);
H(3, 4) = H(4, 3) = F(3,4);

F(:,4) = force_transforms.fr_xarmlink2_XF_fr_xarmlink3 * F(:,4);
H(2, 4) = H(4, 2) = F(3,4);

F(:,4) = force_transforms.fr_xarmlink1_XF_fr_xarmlink2 * F(:,4);
H(1, 4) = H(4, 1) = F(3,4);

F(:,4) = force_transforms.fr_base_link_XF_fr_xarmlink1 * F(:,4);

% Link xarmlink3
Ic_xarmlink2 = Ic_xarmlink2 + force_transforms.fr_xarmlink2_XF_fr_xarmlink3 * Ic_xarmlink3 * force_transforms.fr_xarmlink2_XF_fr_xarmlink3';

F(:,3) = Ic_xarmlink3(:,3);
H(3, 3) = F(3,3);

F(:,3) = force_transforms.fr_xarmlink2_XF_fr_xarmlink3 * F(:,3);
H(2, 3) = H(3, 2) = F(3,3);

F(:,3) = force_transforms.fr_xarmlink1_XF_fr_xarmlink2 * F(:,3);
H(1, 3) = H(3, 1) = F(3,3);

F(:,3) = force_transforms.fr_base_link_XF_fr_xarmlink1 * F(:,3);

% Link xarmlink2
Ic_xarmlink1 = Ic_xarmlink1 + force_transforms.fr_xarmlink1_XF_fr_xarmlink2 * Ic_xarmlink2 * force_transforms.fr_xarmlink1_XF_fr_xarmlink2';

F(:,2) = Ic_xarmlink2(:,3);
H(2, 2) = F(3,2);

F(:,2) = force_transforms.fr_xarmlink1_XF_fr_xarmlink2 * F(:,2);
H(1, 2) = H(2, 1) = F(3,2);

F(:,2) = force_transforms.fr_base_link_XF_fr_xarmlink1 * F(:,2);

% Link xarmlink1
Ic_base_link = Ic_base_link + force_transforms.fr_base_link_XF_fr_xarmlink1 * Ic_xarmlink1 * force_transforms.fr_base_link_XF_fr_xarmlink1';

F(:,1) = Ic_xarmlink1(:,3);
H(1, 1) = F(3,1);

F(:,1) = force_transforms.fr_base_link_XF_fr_xarmlink1 * F(:,1);
