function ci = compositeInertia(ip, xf, transformsType)

% Computes the spatial composite inertia of each link of the robot.
% Arguments:
% - ip : the structure with the inertia properties
% - xf : the structure with the spatial coordinate transformation matrices
% - transformsType : a string specifying which is the type of the given
%      coordinate transforms, either velocity ('motion') or force ('force').
%      Optional argument, default is 'force'.

if nargin < 3
    transformsType = 'force';
end

%
% Initialization of the composite-inertia matrices
%
ci.xarmlink1_Ic = ip.lf_xarmlink1.tensor6D;
ci.xarmlink2_Ic = ip.lf_xarmlink2.tensor6D;
ci.xarmlink3_Ic = ip.lf_xarmlink3.tensor6D;
ci.xarmlink4_Ic = ip.lf_xarmlink4.tensor6D;
ci.xarmlink5_Ic = ip.lf_xarmlink5.tensor6D;
ci.xarmlink6_Ic = ip.lf_xarmlink6.tensor6D;
ci.base_link_Ic = ip.lf_base_link.tensor6D;

%
% Leafs-to-root pass to update the composite inertia of
%     each link, for the current configuration:
%

if strcmp(transformsType, 'motion')  % we have transforms for motion vectors

% Contribution of link xarmlink6
ci.xarmlink5_Ic = ci.xarmlink5_Ic + xf.fr_xarmlink6_XM_fr_xarmlink5' * ci.xarmlink6_Ic * xf.fr_xarmlink6_XM_fr_xarmlink5;


% Contribution of link xarmlink5
ci.xarmlink4_Ic = ci.xarmlink4_Ic + xf.fr_xarmlink5_XM_fr_xarmlink4' * ci.xarmlink5_Ic * xf.fr_xarmlink5_XM_fr_xarmlink4;


% Contribution of link xarmlink4
ci.xarmlink3_Ic = ci.xarmlink3_Ic + xf.fr_xarmlink4_XM_fr_xarmlink3' * ci.xarmlink4_Ic * xf.fr_xarmlink4_XM_fr_xarmlink3;


% Contribution of link xarmlink3
ci.xarmlink2_Ic = ci.xarmlink2_Ic + xf.fr_xarmlink3_XM_fr_xarmlink2' * ci.xarmlink3_Ic * xf.fr_xarmlink3_XM_fr_xarmlink2;


% Contribution of link xarmlink2
ci.xarmlink1_Ic = ci.xarmlink1_Ic + xf.fr_xarmlink2_XM_fr_xarmlink1' * ci.xarmlink2_Ic * xf.fr_xarmlink2_XM_fr_xarmlink1;


% Contribution of link xarmlink1
ci.base_link_Ic = ci.base_link_Ic + xf.fr_xarmlink1_XM_fr_base_link' * ci.xarmlink1_Ic * xf.fr_xarmlink1_XM_fr_base_link;


else % we have transforms for force vectors

% Contribution of link xarmlink6
ci.xarmlink5_Ic = ci.xarmlink5_Ic + xf.fr_xarmlink5_XF_fr_xarmlink6 * ci.xarmlink6_Ic * xf.fr_xarmlink5_XF_fr_xarmlink6';


% Contribution of link xarmlink5
ci.xarmlink4_Ic = ci.xarmlink4_Ic + xf.fr_xarmlink4_XF_fr_xarmlink5 * ci.xarmlink5_Ic * xf.fr_xarmlink4_XF_fr_xarmlink5';


% Contribution of link xarmlink4
ci.xarmlink3_Ic = ci.xarmlink3_Ic + xf.fr_xarmlink3_XF_fr_xarmlink4 * ci.xarmlink4_Ic * xf.fr_xarmlink3_XF_fr_xarmlink4';


% Contribution of link xarmlink3
ci.xarmlink2_Ic = ci.xarmlink2_Ic + xf.fr_xarmlink2_XF_fr_xarmlink3 * ci.xarmlink3_Ic * xf.fr_xarmlink2_XF_fr_xarmlink3';


% Contribution of link xarmlink2
ci.xarmlink1_Ic = ci.xarmlink1_Ic + xf.fr_xarmlink1_XF_fr_xarmlink2 * ci.xarmlink2_Ic * xf.fr_xarmlink1_XF_fr_xarmlink2';


% Contribution of link xarmlink1
ci.base_link_Ic = ci.base_link_Ic + xf.fr_base_link_XF_fr_xarmlink1 * ci.xarmlink1_Ic * xf.fr_base_link_XF_fr_xarmlink1';


end
