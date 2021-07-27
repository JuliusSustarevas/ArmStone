function out = inertiaProperties()

% Inertia parameters as written in the .kindsl model file
out.base_link.mass = 2.7;
out.base_link.tensor = ...
    [[  0.00494875,	-(3.5E-6),	-(-1.25E-5)];
     [-(3.5E-6),	  0.00494174 ,	-(-1.67E-6)];
     [-(-1.25E-5),	-(-1.67E-6),	  0.002219]];
out.base_link.com = [0.0; 0.0; 0.0];

out.xarmlink1.mass = 2.16;
out.xarmlink1.tensor = ...
    [[  0.00539427,	-(-1.095E-5),	-(-1.635E-6)];
     [-(-1.095E-5),	  0.0048979 ,	-(-7.93E-4)];
     [-(-1.635E-6),	-(-7.93E-4),	  0.00311573]];
out.xarmlink1.com = [0.0; 0.0; 0.0];

out.xarmlink2.mass = 1.71;
out.xarmlink2.tensor = ...
    [[  0.0248674,	-(0.00430651),	-(6.7797E-4)];
     [-(0.00430651),	  0.00485548 ,	-(-0.00457245)];
     [-(6.7797E-4),	-(-0.00457245),	  0.02387827]];
out.xarmlink2.com = [0.0; 0.0; 0.0];

out.xarmlink3.mass = 1.384;
out.xarmlink3.tensor = ...
    [[  0.0053694,	-(-0.0014185),	-(9.2094E-4)];
     [-(-0.0014185),	  0.0032423 ,	-(0.00169178)];
     [-(9.2094E-4),	-(0.00169178),	  0.00501731]];
out.xarmlink3.com = [0.0; 0.0; 0.0];

out.xarmlink4.mass = 1.115;
out.xarmlink4.tensor = ...
    [[  0.00439263,	-(-5.028E-5),	-(-1.374E-5)];
     [-(-5.028E-5),	  0.0040077 ,	-(-4.5338E-4)];
     [-(-1.374E-5),	-(-4.5338E-4),	  0.00110321]];
out.xarmlink4.com = [0.0; 0.0; 0.0];

out.xarmlink5.mass = 1.275;
out.xarmlink5.tensor = ...
    [[  0.001202758,	-(-4.92428E-4),	-(3.9147E-4)];
     [-(-4.92428E-4),	  0.0022876 ,	-(1.235E-4)];
     [-(3.9147E-4),	-(1.235E-4),	  0.0026866]];
out.xarmlink5.com = [0.0; 0.0; 0.0];

out.xarmlink6.mass = 1.1596;
out.xarmlink6.tensor = ...
    [[  0.0041169566,	-(0.0),	-(0.0)];
     [-(0.0),	  0.0041197343 ,	-(-6.003373E-6)];
     [-(0.0),	-(-6.003373E-6),	  0.0013922557]];
out.xarmlink6.com = [0.0; 0.0; 0.0];


% Now the same inertia parameters expressed in the link frame (may be equal or not to
%  the previous ones, depending on the model file)
out.lf_base_link.mass = 2.7;
out.lf_base_link.tensor = ...
    [[  0.008853708,	-(3.5E-6),	-(0.017956674)];
     [-(3.5E-6),	  0.0915342 ,	-(-1.67E-6)];
     [-(0.017956674),	-(-1.67E-6),	  0.084906496]];
out.lf_base_link.com = [0.175; 0.0; 0.03803];
com = out.lf_base_link.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_base_link.mass;
out.lf_base_link.tensor6D = [out.lf_base_link.tensor, block; block', out.lf_base_link.mass*eye(3)];

out.lf_xarmlink1.mass = 2.16;
out.lf_xarmlink1.tensor = ...
    [[  0.0073428256,	-(-1.272444E-4),	-(5.5907403E-5)];
     [-(-1.272444E-4),	  0.0052897725 ,	-(-0.0015675207)];
     [-(5.5907403E-5),	-(-0.0015675207),	  0.0046896925]];
out.lf_xarmlink1.com = [-0.002; 0.02692; -0.01332];
com = out.lf_xarmlink1.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_xarmlink1.mass;
out.lf_xarmlink1.tensor6D = [out.lf_xarmlink1.tensor, block; block', out.lf_xarmlink1.mass*eye(3)];

out.lf_xarmlink2.mass = 1.71;
out.lf_xarmlink2.tensor = ...
    [[  0.105124444,	-(-0.008613624),	-(0.0027224403)];
     [-(-0.008613624),	  0.008948017 ,	-(-0.016962022)];
     [-(0.0027224403),	-(-0.016962022),	  0.10430682]];
out.lf_xarmlink2.com = [0.03531; -0.21398; 0.03386];
com = out.lf_xarmlink2.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_xarmlink2.mass;
out.lf_xarmlink2.tensor6D = [out.lf_xarmlink2.tensor, block; block', out.lf_xarmlink2.mass*eye(3)];

out.lf_xarmlink3.mass = 1.384;
out.lf_xarmlink3.tensor = ...
    [[  0.021654077,	-(0.008669333),	-(0.0022883206)];
     [-(0.008669333),	  0.0099000055 ,	-(0.003859303)];
     [-(0.0022883206),	-(0.003859303),	  0.027372088]];
out.lf_xarmlink3.com = [0.06781; 0.10749; 0.01457];
com = out.lf_xarmlink3.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_xarmlink3.mass;
out.lf_xarmlink3.tensor6D = [out.lf_xarmlink3.tensor, block; block', out.lf_xarmlink3.mass*eye(3)];

out.lf_xarmlink4.mass = 1.115;
out.lf_xarmlink4.tensor = ...
    [[  0.0058518867,	-(-5.631638E-5),	-(-7.797278E-6)];
     [-(-5.631638E-5),	  0.004725969 ,	-(-0.0011829191)];
     [-(-7.797278E-6),	-(-0.0011829191),	  0.0018442962]];
out.lf_xarmlink4.com = [-2.1E-4; 0.025779976; -0.025379976];
com = out.lf_xarmlink4.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_xarmlink4.mass;
out.lf_xarmlink4.tensor6D = [out.lf_xarmlink4.tensor, block; block', out.lf_xarmlink4.mass*eye(3)];

out.lf_xarmlink5.mass = 1.275;
out.lf_xarmlink5.tensor = ...
    [[  0.0016447763,	-(7.4014865E-4),	-(7.6726405E-4)];
     [-(7.4014865E-4),	  0.0060817497 ,	-(2.4680307E-4)];
     [-(7.6726405E-4),	-(2.4680307E-4),	  0.006847581]];
out.lf_xarmlink5.com = [0.05428; 0.01781; 0.00543];
com = out.lf_xarmlink5.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_xarmlink5.mass;
out.lf_xarmlink5.tensor6D = [out.lf_xarmlink5.tensor, block; block', out.lf_xarmlink5.mass*eye(3)];

out.lf_xarmlink6.mass = 1.1596;
out.lf_xarmlink6.tensor = ...
    [[  0.0108265085,	-(0.0),	-(0.0)];
     [-(0.0),	  0.010829282 ,	-(-6.6778057E-7)];
     [-(0.0),	-(-6.6778057E-7),	  0.0013922598]];
out.lf_xarmlink6.com = [0.0; 6.048977E-5; 0.07606634];
com = out.lf_xarmlink6.com;
block = [  0,    -com(3),  com(2);
         com(3),  0,      -com(1);
        -com(2),  com(1),  0 ] * out.lf_xarmlink6.mass;
out.lf_xarmlink6.tensor6D = [out.lf_xarmlink6.tensor, block; block', out.lf_xarmlink6.mass*eye(3)];


% Same inertial properties expressed in a frame with origin in the COM of the link
%  oriented as the default link-frame (the COM coordinates in such a frame should
%  always be [0,0,0] ).
out.com_base_link.mass = 2.7;
out.com_base_link.tensor = ...
    [[  0.00494875,	-(3.5E-6),	-(-1.2500212E-5)];
     [-(3.5E-6),	  0.004941739 ,	-(-1.67E-6)];
     [-(-1.2500212E-5),	-(-1.67E-6),	  0.002218999]];
out.com_base_link.com = [0.0; 0.0; 0.0];

out.com_xarmlink1.mass = 2.16;
out.com_xarmlink1.tensor = ...
    [[  0.0053942706,	-(-1.0949996E-5),	-(-1.6349986E-6)];
     [-(-1.0949996E-5),	  0.0048979 ,	-(-7.929999E-4)];
     [-(-1.6349986E-6),	-(-7.929999E-4),	  0.00311573]];
out.com_xarmlink1.com = [0.0; 0.0; 0.0];

out.com_xarmlink2.mass = 1.71;
out.com_xarmlink2.tensor = ...
    [[  0.0248674,	-(0.00430651),	-(6.7797E-4)];
     [-(0.00430651),	  0.0048554805 ,	-(-0.00457245)];
     [-(6.7797E-4),	-(-0.00457245),	  0.023878269]];
out.com_xarmlink2.com = [0.0; 0.0; 0.0];

out.com_xarmlink3.mass = 1.384;
out.com_xarmlink3.tensor = ...
    [[  0.0053694006,	-(-0.0014185002),	-(9.209401E-4)];
     [-(-0.0014185002),	  0.0032423004 ,	-(0.00169178)];
     [-(9.209401E-4),	-(0.00169178),	  0.0050173104]];
out.com_xarmlink3.com = [0.0; 0.0; 0.0];

out.com_xarmlink4.mass = 1.115;
out.com_xarmlink4.tensor = ...
    [[  0.00439263,	-(-5.028E-5),	-(-1.374E-5)];
     [-(-5.028E-5),	  0.0040077 ,	-(-4.5338005E-4)];
     [-(-1.374E-5),	-(-4.5338005E-4),	  0.00110321]];
out.com_xarmlink4.com = [0.0; 0.0; 0.0];

out.com_xarmlink5.mass = 1.275;
out.com_xarmlink5.tensor = ...
    [[  0.001202758,	-(-4.92428E-4),	-(3.9147004E-4)];
     [-(-4.92428E-4),	  0.0022876002 ,	-(1.235E-4)];
     [-(3.9147004E-4),	-(1.235E-4),	  0.0026865997]];
out.com_xarmlink5.com = [0.0; 0.0; 0.0];

out.com_xarmlink6.mass = 1.1596;
out.com_xarmlink6.tensor = ...
    [[  0.0041169566,	-(0.0),	-(0.0)];
     [-(0.0),	  0.0041197343 ,	-(-6.003373E-6)];
     [-(0.0),	-(-6.003373E-6),	  0.0013922557]];
out.com_xarmlink6.com = [0.0; 0.0; 0.0];

