%%Generalparametersandconversiongains

% controllersamplingtime
Ts=1e-2;

% gravityacc[m/s^2]
g=9.81;

% conversiongains
rpm2rads=2*pi/60; % [rpm] −>[rad/s]
rads2rpm=60/2/pi; % [rad/s]−>[rpm]
rpm2degs=360/60; % [rpm] −>[deg/s]
degs2rpm=60/360; % [deg/s]−>[rpm]
deg2rad=pi/180; % [deg] −>[rad]
rad2deg=180/pi; % [rad] −>[deg]
g2ms2=g; % [acc_g]−>[m/s^2]
ms22g=1/g; % [ms^2]−>[acc_g]
ozin2Nm=0.706e-2; % [oz*inch]−>[N*m]

% robotinitialcondition
x0=[...
0,... % gam(0)
5*deg2rad,... % th(0)
0,... % dot_gam(0)
0]; % dot_th(0)

%%DCmotordata

% motorid:brushedDCgearmotorPololu30:137Dx68Lmm

% electromechanicalparams
mot.UN =12; % nominalvoltage
mot.taus=110/30 * ozin2Nm; % stalltorque@nomvoltage
mot.Is =5; % stallcurrent@nomvoltage
mot.w0 =350 * 30 * rpm2rads; % no−loadspeed@nomvoltage
mot.I0 =0.3; % no−loadcurrent@nomvoltage
mot.R =mot.UN/mot.Is; % armatureresistance
mot.L =NaN; % armatureinductance
mot.Kt =mot.taus/mot.Is; % torqueconstant
mot.Ke =(mot.UN-mot.R*mot.I0)/(mot.w0); % back−EMFconstant
mot.eta =NaN; % motorefficiency
mot.PN =NaN; % nominaloutputpower
mot.IN =NaN; % nominalcurrent
mot.tauN=NaN; % nominaltorque

% dimensions
mot.rot.h =30.7e-3; % rotorheight
mot.rot.r =0.9 * 17e-3; % rotorradius

mot.stat.h =68.1e-3; % statorheight
mot.stat.r =17e-3; % statorradius

% centerofmass(CoM)position
mot.rot.xb =0; % (left)rotCoMx−posinbodyframe
mot.rot.yb =42.7e-3; % (left)rotCoMy−posinbodyframe
mot.rot.zb =-7e-3; % (left)rotCoMz−posinbodyframe

mot.stat.xb=0; % (left)statCoMx−posinbodyframe
mot.stat.yb=52.1e-3; % (left)statCoMy−posinbodyframe
mot.stat.zb=-7e-3; % (left)statCoMz−posinbodyframe

% mass
mot.m =0.215; % totalmotormass
mot.rot.m =0.35 * mot.m; % rotormass
mot.stat.m =mot.m-mot.rot.m; % statormass

% momentofinertias(MoI)wrtprincipalaxes
mot.rot.Ixx =mot.rot.m/12 * (3*mot.rot.r^2+mot.rot.h^2); % MoIalongrdir
mot.rot.Iyy =mot.rot.m/2 * mot.rot.r^2; % MoIalonghdim
mot.rot.Izz =mot.rot.Ixx; % MoIalongrdir

mot.stat.Ixx=mot.stat.m/12 * (3*mot.stat.r^2+mot.stat.h^2);% MoIalongrdir
mot.stat.Iyy=mot.stat.m/2 * mot.stat.r^2; % MoIalonghdir
mot.stat.Izz=mot.stat.Ixx; % MoIalongrdir

% viscousfrictioncoeff(motorside)
mot.B =mot.Kt*mot.I0/mot.w0;

%% Gearboxdata

gbox.N=30; % reductionratio
gbox.B=0.025; % viscousfrictioncoeff(loadside)

%% Batterydata

% electricaldata
batt.UN=11.1; % nominalvoltage

% dimensions
batt.w=136e-3; % batterypackwidth
batt.h=26e-3; % batterypackheight
batt.d=44e-3; % batterypackdepth

% centerofmass(CoM)position
batt.xb=0; % CoMx−posinbodyframe
batt.yb=0; % CoMy−posinbodyframe
batt.zb=44e-3; % CoMz−posinbodyframe

% mass
batt.m=0.320;

% momentofinertias(MoI)wrtprincipalaxes
batt.Ixx=batt.m/12 * (batt.w^2+batt.h^2); % MoIalongddim
batt.Iyy=batt.m/12 * (batt.d^2+batt.h^2); % MoIalongwdim
batt.Izz=batt.m/12 * (batt.w^2+batt.d^2); % MoIalonghdim

%% H−bridgePWMvoltagedriverdata

drv.Vbus=batt.UN; % H−bridgeDCbusvoltage
drv.pwm.bits=8; % PWMresolution[bits]
drv.pwm.levels=2^drv.pwm.bits; % PWMlevels
drv.dutymax=drv.pwm.levels-1; % maxdutycyclecode
drv.duty2V=drv.Vbus/drv.dutymax; % dutycyclecode(0−255)tovoltage
drv.V2duty=drv.dutymax/drv.Vbus; % voltagetodutycyclecode(0−255)

%% Wheeldata

% dimensions
wheel.h=26e-3; % wheelheight
wheel.r=68e-3/2; % wheelradius

% centerofmass(CoM)position
wheel.xb=0; % (left)wheelCoMx−posinbodyframe
wheel.yb=100e-3; % (left)wheelCoMy−posinbodyframe
wheel.zb=0; % (left)wheelCoMz−posinbodyframe

% mass
wheel.m=50e-3;

% momentofinertias(MoI)wrtprincipalaxes
wheel.Ixx=wheel.m/12 * (3*wheel.r^2+wheel.h^2); % MoIalongrdim
wheel.Iyy=wheel.m/2 * wheel.r^2; % MoIalonghdim
wheel.Izz=wheel.Ixx; % MoIalongrdim

% viscousfrictioncoeff
wheel.B=0.0015;

%% Chassisdata

% dimensions
chassis.w=160e-3; % framewidth
chassis.h=119e-3; % frameheight
chassis.d=80e-3; % framedepth

% centerofmass(CoM)position
chassis.xb=0; % CoMx−posinbodyframe
chassis.yb=0; % CoMx−posinbodyframe
chassis.zb=80e-3; % CoMx−posinbodyframe

% mass
chassis.m=0.456;

% momentofinertias(MoI)wrtprincipalaxes
chassis.Ixx=chassis.m/12 * (chassis.w^2+chassis.h^2); % MoIalongddim
chassis.Iyy=chassis.m/12 * (chassis.d^2+chassis.h^2); % MoIalongwdim
chassis.Izz=chassis.m/12 * (chassis.w^2+chassis.d^2); % MoIalonghdim

%% Bodydata

% mass
body.m=chassis.m+batt.m+2*mot.stat.m;

% centerofmass(CoM)position
body.xb=0; % CoMx−posinbodyframe
body.yb=0; % CoMy−posinbodyframe
body.zb=(1/body.m) * (chassis.m*chassis.zb+... % CoMz−posinbodyframe
batt.m*batt.zb+2*mot.stat.m*mot.stat.zb);

% momentofinertias(MoI)wrtprincipalaxes
body.Ixx=chassis.Ixx+chassis.m*(body.zb-chassis.zb)^2+... % MoIalongddim
batt.Ixx+batt.m*(body.zb-batt.zb)^2+...
2*mot.stat.Ixx+...
2*mot.stat.m*(mot.stat.yb^2+(body.zb-mot.stat.zb)^2);

body.Iyy=chassis.Iyy+chassis.m*(body.zb-chassis.zb)^2+... % MoIalongwdim
batt.Iyy+batt.m*(body.zb-batt.zb)^2+...
2*mot.stat.Iyy+...
2*mot.stat.m*(body.zb-mot.stat.zb)^2;

body.Izz=chassis.Izz+batt.Izz+... % MoIalonghdim
2*mot.stat.Izz+2*mot.stat.m*mot.stat.yb^2;

%% Sensorsdata−Hall−effectencoder

% Hall−effectencoder
sens.enc.ppr=16*4; % pulsesperrotationatmotorside(w/quadraturedecoding)
sens.enc.pulse2deg=360/sens.enc.ppr;
sens.enc.pulse2rad=2*pi/sens.enc.ppr;
sens.enc.deg2pulse=sens.enc.ppr/360;
sens.enc.rad2pulse=sens.enc.ppr/2/pi;

%% Sensorsdata−MPU6050(accelerometer+gyro)

% centerofmass(CoM)position
sens.mpu.xb=0;
sens.mpu.yb=0;
sens.mpu.zb=13.5e-3;

% MPU6050embeddedaccelerometerspecs
sens.mpu.acc.bits=16;
sens.mpu.acc.fs_g=16; % full−scalein"g"units
sens.mpu.acc.fs=sens.mpu.acc.fs_g * g2ms2; % full−scalein[m/s^2]
sens.mpu.acc.g2LSB=floor(2^(sens.mpu.acc.bits-1)/sens.mpu.acc.fs_g); % sensitivity[LSB/g]
sens.mpu.acc.ms22LSB=sens.mpu.acc.g2LSB * ms22g; % sensitvity[LSB/(m/s^2)]
sens.mpu.acc.LSB2g=sens.mpu.acc.fs_g/2^(sens.mpu.acc.bits-1); % outquantization[g/LSB]
sens.mpu.acc.LSB2ms2=sens.mpu.acc.LSB2g * g2ms2; % outquantization[ms2/LSB]
sens.mpu.acc.bw=94; % outlow−passfilterBW[Hz]
sens.mpu.acc.noisestd=400e-6*sqrt(100); % outputnoisestd[g−rms]
sens.mpu.acc.noisevar=sens.mpu.acc.noisestd^2; % outputnoisevar[g^2]

% MPU6050embdeddedgyroscopespecs
sens.mpu.gyro.bits=16;
sens.mpu.gyro.fs_degs=250; % fullscalein[deg/s(dps)]
sens.mpu.gyro.fs=sens.mpu.gyro.fs_degs * deg2rad; % fullscalein[rad/s]
sens.mpu.gyro.degs2LSB=floor(2^(sens.mpu.gyro.bits-1)/sens.mpu.gyro.fs_degs);% sensitivity[LSB/degs]
sens.mpu.gyro.rads2LSB=sens.mpu.gyro.degs2LSB * rad2deg; % sensitivity[LSB/rads]
sens.mpu.gyro.LSB2degs=sens.mpu.gyro.fs_degs/2^(sens.mpu.gyro.bits-1); % outquantization[degs/LSB]
sens.mpu.gyro.LSB2rads=sens.mpu.gyro.LSB2degs * deg2rad; % outquantization[rads/LSB]
sens.mpu.gyro.bw=98; % outlow−passfilterBW[Hz]
sens.mpu.gyro.noisestd=5e-3*sqrt(100); % outputnoisestd[degs−rms]
sens.mpu.gyro.noisevar=sens.mpu.acc.noisestd^2; % outputnoisevar[degs^2]

%%Voltage to torque
ua2tau = (2*gbox.N*mot.Kt/mot.R)*[1; -1];

%%M matrix elelments

M11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy + (body.m+ 2*wheel.m + 2*mot.rot.m)*wheel.r^2;
M12_const = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy;
M12_lin = M12_const + (body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r;
M22 = body.Iyy + 2*(1-gbox.N)^2*mot.rot.Iyy + body.m*body.zb^2 + 2*mot.rot.m*(mot.rot.zb)^2;

%%C matrix elements
C11 = 0;
C21 = 0;
C22 = 0;
C12 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r;

%%Fv matrix elements
Fv11 = 2*(gbox.B+wheel.B);
Fv12 = -2*gbox.B;
Fv22 = 2*gbox.B;

Fv = [Fv11, Fv12;
      Fv12, Fv22];
Fv_pr = Fv + (2*gbox.N^2*mot.Kt*mot.Ke/mot.R)*[1, -1;
                                              -1, 1];

%%g matrix elements
g22 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*g;

%% Complementary Filter params
fc = 0.35;
Tc = 1/(2*pi*fc);
T = 1e-02;
C_filter = T/(Tc + T);