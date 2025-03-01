% Fixed wing parameters
% SI units unless specified otherwise

%   Copyright 2022 The MathWorks, Inc.

uavData = struct;

uavData.geometry = struct;
uavData.geometry.span = 2.795;
uavData.geometry.chord = 0.35134168157423973;
uavData.geometry.S = uavData.geometry.span *uavData.geometry.chord ;
uavData.geometry.elarm = 1.21;
uavData.geometry.mass = 8.1646626600000012;
uavData.inertia = struct;
uavData.inertia.Ixx = 4.12;
uavData.inertia.Iyy = 9.58;
uavData.inertia.Izz = 9.85;

uavData.aero = struct;
uavData.aero.CL0 = 0.38;
uavData.aero.CLa = 6;
uavData.aero.CLa_dot = 2.64;
uavData.aero.CLq = 7.4;
uavData.aero.CLDe = 0.24;
uavData.aero.CLDf = 0.4;
uavData.aero.CD0 = 0.022;
uavData.aero.A1 = 0.007;
uavData.aero.Apolar = 0.057;
uavData.aero.CYb = -1.098;
uavData.aero.CYDr = 0.143;
uavData.aero.Clb = -0.296;
uavData.aero.Clp = -1.96;
uavData.aero.Clr = 0.103;
uavData.aero.ClDa = 0.1695;
uavData.aero.ClDr = 0.106;
uavData.aero.Cm0 = 0.3;
uavData.aero.Cma = -1.239;
uavData.aero.Cma_dot = -7;
uavData.aero.Cmq = -2.4;
uavData.aero.CmDe = -3.2;
uavData.aero.CmDf = -0.021;
uavData.aero.Cnb = 0.277;
uavData.aero.Cnp = -0.0889;
uavData.aero.Cnr = -0.19997;
uavData.aero.CnDa = -0.023;
uavData.aero.CnDr = -0.1997;

uavData.engine = struct;
uavData.engine.MinThK = 0.077;
uavData.engine.ThK = 2.17;
uavData.engine.TFact = 0.2;
uavData.ic = struct;
uavData.ic.Pos_0 = [0; 0; 50];
uavData.ic.Euler_0 = [0; 0; 1.5707963267948966];
uavData.ic.Omega_0 = [0; 0; 0];
uavData.ic.PQR_0 = [0; 0; 0];
uavData.ic.Vb_0 = [18; 0; 0];
uavData.ic.gsLL = [42.2996388 -71.3517757];
uavData.ic.gsH = 150;

uavData.cntrl.elevatorGain  = -25*pi/180;
uavData.cntrl.aieleronGain  = 25*pi/180;
uavData.cntrl.rudderGain = -25*pi/180;
uavData.cntrl.throttleGain  = 100;

uavData.contact.naturalFreq = 20;
uavData.contact.spring = uavData.geometry.mass*(uavData.contact.naturalFreq*2*pi)^2;
uavData.contact.damper = 2*uavData.geometry.mass*sqrt(uavData.contact.spring/uavData.geometry.mass);
uavData.contact.rollingFriction = 0.2;
uavData.contact.vd = 0.02;
uavData.contact.slidingFriction = 0.8;
uavData.contact.gLimit = 100;

% LocalWords:  uav
