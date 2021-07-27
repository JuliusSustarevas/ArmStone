function [tau base_link_a] = inverseDynamics(ip, xm, base_link_v, gravity, qd, qdd, q)

if nargin == 7   % the joint status is also an argument
    xm = updateMotionTransforms(xm, q);
end


% First pass, link 'xarmlink1'
xarmlink1_v = zeros(6,1);
xarmlink1_a = zeros(6,1);

xarmlink1_v = xm.fr_xarmlink1_XM_fr_base_link * base_link_v;
xarmlink1_v(3) += qd(1);

vcross = vcross_mx(xarmlink1_v);

xarmlink1_a = vcross(:,3) * qd(1);
xarmlink1_a(3) += qdd(1);

xarmlink1_f = ip.lf_xarmlink1.tensor6D * xarmlink1_a + (-vcross' * ip.lf_xarmlink1.tensor6D * xarmlink1_v);

% First pass, link 'xarmlink2'
xarmlink2_v = zeros(6,1);
xarmlink2_a = zeros(6,1);

xarmlink2_v = xm.fr_xarmlink2_XM_fr_xarmlink1 * xarmlink1_v;
xarmlink2_v(3) += qd(2);

vcross = vcross_mx(xarmlink2_v);

xarmlink2_a = xm.fr_xarmlink2_XM_fr_xarmlink1 * xarmlink1_a + (vcross(:,3) * qd(2));
xarmlink2_a(3) += qdd(2);

xarmlink2_f = ip.lf_xarmlink2.tensor6D * xarmlink2_a + (-vcross' * ip.lf_xarmlink2.tensor6D * xarmlink2_v);

% First pass, link 'xarmlink3'
xarmlink3_v = zeros(6,1);
xarmlink3_a = zeros(6,1);

xarmlink3_v = xm.fr_xarmlink3_XM_fr_xarmlink2 * xarmlink2_v;
xarmlink3_v(3) += qd(3);

vcross = vcross_mx(xarmlink3_v);

xarmlink3_a = xm.fr_xarmlink3_XM_fr_xarmlink2 * xarmlink2_a + (vcross(:,3) * qd(3));
xarmlink3_a(3) += qdd(3);

xarmlink3_f = ip.lf_xarmlink3.tensor6D * xarmlink3_a + (-vcross' * ip.lf_xarmlink3.tensor6D * xarmlink3_v);

% First pass, link 'xarmlink4'
xarmlink4_v = zeros(6,1);
xarmlink4_a = zeros(6,1);

xarmlink4_v = xm.fr_xarmlink4_XM_fr_xarmlink3 * xarmlink3_v;
xarmlink4_v(3) += qd(4);

vcross = vcross_mx(xarmlink4_v);

xarmlink4_a = xm.fr_xarmlink4_XM_fr_xarmlink3 * xarmlink3_a + (vcross(:,3) * qd(4));
xarmlink4_a(3) += qdd(4);

xarmlink4_f = ip.lf_xarmlink4.tensor6D * xarmlink4_a + (-vcross' * ip.lf_xarmlink4.tensor6D * xarmlink4_v);

% First pass, link 'xarmlink5'
xarmlink5_v = zeros(6,1);
xarmlink5_a = zeros(6,1);

xarmlink5_v = xm.fr_xarmlink5_XM_fr_xarmlink4 * xarmlink4_v;
xarmlink5_v(3) += qd(5);

vcross = vcross_mx(xarmlink5_v);

xarmlink5_a = xm.fr_xarmlink5_XM_fr_xarmlink4 * xarmlink4_a + (vcross(:,3) * qd(5));
xarmlink5_a(3) += qdd(5);

xarmlink5_f = ip.lf_xarmlink5.tensor6D * xarmlink5_a + (-vcross' * ip.lf_xarmlink5.tensor6D * xarmlink5_v);

% First pass, link 'xarmlink6'
xarmlink6_v = zeros(6,1);
xarmlink6_a = zeros(6,1);

xarmlink6_v = xm.fr_xarmlink6_XM_fr_xarmlink5 * xarmlink5_v;
xarmlink6_v(3) += qd(6);

vcross = vcross_mx(xarmlink6_v);

xarmlink6_a = xm.fr_xarmlink6_XM_fr_xarmlink5 * xarmlink5_a + (vcross(:,3) * qd(6));
xarmlink6_a(3) += qdd(6);

xarmlink6_f = ip.lf_xarmlink6.tensor6D * xarmlink6_a + (-vcross' * ip.lf_xarmlink6.tensor6D * xarmlink6_v);

%
% The force exerted on the floating base by the links
%
vcross = vcross_mx(base_link_v);
base_link_f = - vcross' * ip.lf_base_link.tensor6D * base_link_v;


%
% Pass 2. Compute the composite inertia and the spatial forces
%
ci = compositeInertia(ip, xm, 'motion');
xarmlink5_f = xarmlink5_f + xm.fr_xarmlink6_XM_fr_xarmlink5' * xarmlink6_f;
xarmlink4_f = xarmlink4_f + xm.fr_xarmlink5_XM_fr_xarmlink4' * xarmlink5_f;
xarmlink3_f = xarmlink3_f + xm.fr_xarmlink4_XM_fr_xarmlink3' * xarmlink4_f;
xarmlink2_f = xarmlink2_f + xm.fr_xarmlink3_XM_fr_xarmlink2' * xarmlink3_f;
xarmlink1_f = xarmlink1_f + xm.fr_xarmlink2_XM_fr_xarmlink1' * xarmlink2_f;
base_link_f = base_link_f + xm.fr_xarmlink1_XM_fr_base_link' * xarmlink1_f;

%
% The base acceleration due to the force due to the movement of the links
%
base_link_a = - inverse(ci.base_link_Ic) * base_link_f; % TODO inverse

%
% Pass 3. Compute the joint forces while propagating back the floating base acceleration
%
tau = zeros(6, 1);
xarmlink1_a = xm.fr_xarmlink1_XM_fr_base_link * base_link_a;
tau(1) = ci.xarmlink1_Ic(3,:) * xarmlink1_a + xarmlink1_f(3);

xarmlink2_a = xm.fr_xarmlink2_XM_fr_xarmlink1 * xarmlink1_a;
tau(2) = ci.xarmlink2_Ic(3,:) * xarmlink2_a + xarmlink2_f(3);

xarmlink3_a = xm.fr_xarmlink3_XM_fr_xarmlink2 * xarmlink2_a;
tau(3) = ci.xarmlink3_Ic(3,:) * xarmlink3_a + xarmlink3_f(3);

xarmlink4_a = xm.fr_xarmlink4_XM_fr_xarmlink3 * xarmlink3_a;
tau(4) = ci.xarmlink4_Ic(3,:) * xarmlink4_a + xarmlink4_f(3);

xarmlink5_a = xm.fr_xarmlink5_XM_fr_xarmlink4 * xarmlink4_a;
tau(5) = ci.xarmlink5_Ic(3,:) * xarmlink5_a + xarmlink5_f(3);

xarmlink6_a = xm.fr_xarmlink6_XM_fr_xarmlink5 * xarmlink5_a;
tau(6) = ci.xarmlink6_Ic(3,:) * xarmlink6_a + xarmlink6_f(3);


base_link_a = base_link_a + gravity;

function vc = vcross_mx(v)
    vc = [   0    -v(3)  v(2)   0     0     0    ;
             v(3)  0    -v(1)   0     0     0    ;
            -v(2)  v(1)  0      0     0     0    ;
             0    -v(6)  v(5)   0    -v(3)  v(2) ;
             v(6)  0    -v(4)   v(3)  0    -v(1) ;
            -v(5)  v(4)  0     -v(2)  v(1)  0    ];
