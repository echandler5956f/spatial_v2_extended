clear

%% Test without rotors
disp('Regular revolutes')

model_reg = Arm6LinkModel();

q = rand(model_reg.NQ,1);
q = model_reg.normalizeConfVec(q);
qd = rand(model_reg.NV,1);
qdd = rand(model_reg.NV,1);

[tau_reg,out_reg] = ID(model_reg,q,qd,qdd);
[tau2_reg,out2_reg] = ID_Lee(model_reg,q,qd,qdd);
etau = norm(tau_reg-tau2_reg)

%% Test with rotors
disp('Revolutes with rotors')

model_rot = Arm6LinkRotorModel();

q = rand(model_rot.NQ,1);
q = model_rot.normalizeConfVec(q);
qd = rand(model_rot.NV,1);
qdd = rand(model_rot.NV,1);

[tau_rot,out_rot] = ID(model_rot,q,qd,qdd);
[tau2_rot,out2_rot] = ID_Lee(model_rot,q,qd,qdd);
etau = norm(tau_rot-tau2_rot)

%% Test with absolute triplet
disp('Absolute triplet')
model_abs = Arm6LinkAbsModel();

q = rand(model_abs.NQ,1);
q = model_abs.normalizeConfVec(q);
qd = rand(model_abs.NV,1);
qdd = rand(model_abs.NV,1);

[tau_abs,out_abs] = ID(model_abs,q,qd,qdd);
[tau2_abs,out2_abs] = ID_Lee(model_abs,q,qd,qdd);
etau = norm(tau_abs-tau2_abs)

%% Testing ID derivatives

% Fourier trajectory parameters
num_sample       = 100;     % number of samples of the trajectory
horizon          = 10;      % trajectory horizon
base_frequency   = pi*0.3;  % Fourier trajectory base frequency
sample_times      = linspace(0,horizon,num_sample);
sample_interval  = horizon/num_sample;              % dt for numerical integration
m         = 5;                       % num of trajectory coefs per joints
p_initial = rand(2*m,6) - 0.5;  % initial p
% params is ordered [a_1, b_1, a_2, b_2, ..., a_m, b_m] for Fourier Series: a_k*sin(kwt) + b_k*cos(kwt)

% Fourier trajectory generation with parameter p_initial
[q, qd, qdd]    = makeFourier(p_initial, base_frequency, sample_times);

[dq, dqd, dqdd] = getFourierDerivative(p_initial, base_frequency, sample_times);
% for each joint i and each time j, taking the derivative w.r.t all 2m
% parameters...where does extra i index come from?

test_index = 23;

[tau,out] = ID(model_abs,q(:,test_index),qd(:,test_index),qdd(:,test_index));

[dtau, dV, dVd] = ID_derivatives_Lee( model_abs, q(:,test_index), qd(:,test_index), qdd(:,test_index), ...
                        dq(:,:,test_index), dqd(:,:,test_index), dqdd(:,:,test_index), out.f );

disp('Done!')
