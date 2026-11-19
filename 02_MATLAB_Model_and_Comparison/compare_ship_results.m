function C = compare_ship_results(results)
%COMPARE_SHIP_RESULTS State/trajectory metrics and frozen-state load metrics.
% M was independently integrated. For load-equation checks, reevaluate the
% MATLAB loads at each RECORDED XDYN state, not at the simulated state.
% This preserves the separation between trajectory drift and load-law error.

t=results.t; M=results.model; D=results.reference; par=results.parameters;
usePR=results.active.propellerRudder; useSail=results.active.sail;
scale=[1 1 180/pi 1 1 180/pi];
C.referenceDisplay=D.states.*scale;
C.modelDisplay=M.states.*scale;
% Scale the difference itself, retaining the source's numerical evaluation order.
C.stateError=M.states-D.states;
C.stateErrorDisplay=C.stateError.*scale;
A=error_statistics(C.stateErrorDisplay,zeros(size(C.stateErrorDisplay)));
range=max(C.referenceDisplay,[],1)-min(C.referenceDisplay,[],1);
A.NRMSE_percent=NaN(1,6);
valid=range>eps;
A.NRMSE_percent(valid)=100*A.RMSE(valid)./range(valid);
names={'x';'y';'psi';'u';'v';'r'};
units={'m';'m';'deg';'m/s';'m/s';'deg/s'};
C.stateMetrics=table(names,units,A.RMSE.',A.MAE.',A.MaxAbsError.', ...
    A.FinalError.',A.NRMSE_percent.','VariableNames', ...
    {'State','Unit','RMSE','MAE','MaxAbsError','FinalError','NRMSE_percent'});

C.planarSeparation=hypot(C.stateError(:,1),C.stateError(:,2));
finalSeparation=C.planarSeparation(end);
maxSeparation=max(C.planarSeparation);
meanSeparation=mean(C.planarSeparation);
rmsSeparation=sqrt(mean(C.planarSeparation.^2));
xdynLength=sum(hypot(diff(D.states(:,1)),diff(D.states(:,2))));
matlabLength=sum(hypot(diff(M.states(:,1)),diff(M.states(:,2))));
finalPercent=NaN;
if xdynLength>eps, finalPercent=100*finalSeparation/xdynLength; end
C.trajectoryMetrics=table(finalSeparation,maxSeparation,meanSeparation,rmsSeparation, ...
    xdynLength,finalPercent,matlabLength,'VariableNames', ...
    {'FinalPlanarSeparation_m','MaxPlanarSeparation_m','MeanPlanarSeparation_m', ...
     'RMSPlanarSeparation_m','XDYNPathLength_m','FinalSeparation_percent','MATLABPathLength_m'});

n=numel(t); C.frozenPropellerRudder=zeros(n,3); C.frozenSail=zeros(n,3);
for k=1:n
    [~,loads]=ship_dynamics(t(k),D.states(k,:).',par);
    C.frozenPropellerRudder(k,:)=loads.propellerRudder;
    C.frozenSail(k,:)=loads.sail;
end
names={'propellerRudder_Fx';'propellerRudder_Fy';'propellerRudder_Mz'; ...
       'sail_Fx';'sail_Fy';'sail_Mz'};
units={'N';'N';'N_m';'N';'N';'N_m'};
active=logical([repmat(usePR,1,3),repmat(useSail,1,3)]);
modelLoads=[C.frozenPropellerRudder C.frozenSail];
referenceLoads=[D.propellerRudder D.sail];
A=error_statistics(modelLoads(:,active),referenceLoads(:,active));
C.loadMetrics=table(names(active),units(active),A.RMSE.',A.MAE.', ...
    A.MaxAbsError.',A.FinalError.','VariableNames', ...
    {'Load','Unit','RMSE','MAE','MaxAbsError','FinalError'});
end

function A = error_statistics(model,reference)
%ERROR_STATISTICS Sample-based errors for matching numeric arrays.
% Error sign = MATLAB minus reference. Each column is treated independently.
% NRMSE [%] = 100*RMSE/reference range. Undefined for a constant reference.
% These are unweighted sample statistics, matching the uploaded source;
% they are not time-weighted integrals when CSV time intervals are unequal.

if ~isequal(size(model),size(reference)) || isempty(model)
    error('MMG:Metrics','Model and reference arrays must have the same nonempty size.');
end
if ~isreal(model) || ~isreal(reference) || any(~isfinite(model(:))) || any(~isfinite(reference(:)))
    error('MMG:Metrics','Metric inputs must be real and finite.');
end
A.error=model-reference;
A.RMSE=sqrt(mean(A.error.^2,1));
A.MAE=mean(abs(A.error),1);
A.MaxAbsError=max(abs(A.error),[],1);
A.FinalError=A.error(end,:);
referenceRange=max(reference,[],1)-min(reference,[],1);
A.NRMSE_percent=NaN(size(referenceRange));
valid=referenceRange>eps;
A.NRMSE_percent(valid)=100*A.RMSE(valid)./referenceRange(valid);
end
