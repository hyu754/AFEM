%% Evaluate the accuracy of CG CUDA

%Read in LHS, RHS and Solution from CUDA
LHS = textread('LHS_out.txt');
LHS = LHS(:,[1:end-1])
RHS = textread('RHS_out.txt');
sln_CUDA = textread('sln_out.txt');

%Find solution using matlab's build in solver
sln_matlab = inv(LHS)*RHS;

% https://math.stackexchange.com/questions/803672/how-to-evaluate-the-accuracy-for-sparse-linear-system-solver
% Relative residual norm for CG
relative_norm_cuda = norm(RHS - LHS*sln_CUDA)/norm(RHS);
relative_norm_matlab = norm(RHS-LHS*sln_matlab)/norm(RHS);

disp(relative_norm_cuda)
disp(relative_norm_matlab)