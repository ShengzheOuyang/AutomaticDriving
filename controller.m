function [U] = controller(X)
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% state vector
x=X(1); % x position
y=X(2); % y position
v=X(3); % velocity (strictly positive)
beta=X(4); % side slip angle
psi=X(5); % yaw angle
omega=X(6); % yaw rate
x_dot=X(7); % longitudinal velocity
y_dot=X(8); % lateral velocity
psi_dot=X(9); % yaw rate (redundant)
varphi_dot=X(10); % wheel rotary frequency (strictly positive)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% racetrack
load('racetrack.mat','t_r'); % load right  boundary from *.mat file
load('racetrack.mat','t_l'); % load left boundary from *.mat file
t_r_x=t_r(:,1); % x coordinate of right racetrack boundary
t_r_y=t_r(:,2); % y coordinate of right racetrack boundary
t_l_x=t_l(:,1); % x coordinate of left racetrack boundary
t_l_y=t_l(:,2); % y coordinate of left racetrack boundary

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE FEEDBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta=0; % steering angle
G=1; %gear 1 ... 5
Fb=0; % braking force
zeta=0.5; %braking force distribution
phi=0.2; % gas pedal position

%% reference trajectory

t_ref_x = (t_r_x + t_l_x)/2;
t_ref_y = (t_r_y + t_l_y)/2;

t_ref(:,1) = t_ref_x;
t_ref(:,2) = t_ref_y;

%% reference point
% point amount
n=length(t_ref);

% find next point
index=1;
d_min = (x-t_ref_x(index))^2+(y-t_ref_x(index))^2;
min = index;
for i = index:n
    d=(x-t_ref_x(i))^2+(y-t_ref_y(i))^2;
    if d<d_min
        d_min=d;
        min=i;
    end
end
index = min;

% forecast
if x<0 && y>=230 && y<280
    index_f=5;
elseif y>=380
    index_f=8;
elseif x>0 && y>=280 && y<320
    index_f=20;
elseif y<60
    index_f=10;
else
    index_f=15;
end

% guarantee index_ref<n
index_ref = index + index_f;
if index_ref > n
    index_ref = index_ref - n;
end

x_ref = t_ref_x(index_ref);
y_ref = t_ref_y(index_ref);


%% reference velocity

if x<=0 && y>=0 && y<220
    v_ref = 25;
    % link S
elseif x<=0 && y>=220 && y<260
    v_ref = 5;
elseif x<=0 && y>=260 && y<390
    v_ref = 20;
    % oben C
elseif x<=0 && y>=390
    v_ref = 10;
elseif x>0 && y>=400
    v_ref = 10;
elseif x>0 && y>=320 && y<400
    v_ref = 15;
    % rechts H
elseif x>0 && y>=280 && y<320
    v_ref = 5;
elseif x>0 && y>=70 && y<280
    v_ref = 25;
    % unter L
elseif x>0 && y>=20 && y<70
    v_ref = 5;
    % unter J
elseif x>20 && y<20
    v_ref = 10;
elseif x<=20 && y<0
    v_ref = 5;
else
    v_ref = 10;
end


%% reference yaw angle

% calculate psi_ref
x_err  = x_ref - x;
y_err  = y_ref - y;
t_err  = [x_err;y_err];
psi_ref = acos(x_err/norm(t_err));

if y_err<0
    psi_ref = -psi_ref;
end

% guarantee -pi<psi_ref<pi
psi_err = psi_ref - psi;
if(psi_err>pi)
    psi_err = psi_err-2*pi;
end
if(psi_err<-pi)
    psi_err = psi_err+2*pi;
end


%% Regelung
% Regelung der Lenkwinkel: P-Regler
Kp_delta = 2; %1~3
delta = psi_err*Kp_delta;

% guarantee -0.53<delta<0.53
if -0.53 < delta && delta < 0.53
elseif delta <= -0.53
    delta = -0.53;
elseif delta >= 0.53
    delta = 0.53;
end

% Geschwindigkeitsfehler
v_e = v - v_ref;

%Regelung der Fb: P-Regler
zeta = 1;
Kp_Fb= 1800;
Fb = Kp_Fb*(v_e);

%guarantee 0<Fb<15000
if Fb>0 && Fb<15000
elseif Fb>=15000
    Fb = 15000;
else
    Fb = 0;
end

%Regelung der G & phi: bang-bang controller
if v_e<0
    phi=1;
else
    phi=0;
end

if v<=8
    G = 1;
elseif v>8 && v<=16
    G = 2;
elseif v>16 && v<=24
    G = 3;
elseif v>24 && v<=30
    G = 4;
else
    G = 5;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U=[delta G Fb zeta phi]; % input vector
end


