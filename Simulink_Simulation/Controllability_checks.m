%% Controllability check
% finding controllability and observability matrices

% find matrix order
orderA = size(A);
orderB = size(B);

P_c = zeros(orderA(1),orderB(2)*orderA(2));
for i = 0:order(1)-1
    P_c(:,i*orderB(2)+1:(i+1)*orderB(2)) = A^i * B;
end

% find rank of P_c
if rank(P_c) == orderA(1)
    fprintf("System is Controllable \n")
else
    fprintf("System is not Controllable \n")
end

