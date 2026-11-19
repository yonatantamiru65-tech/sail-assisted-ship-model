function figures = show_results(results,makePlots,saveResults)
%SHOW_RESULTS One tabbed window for MATLAB and XDYN results.
% REPLACE ONLY show_results.m in your existing SIX-file model folder.
%
% To obtain the comparison, set these in RUN_MODEL.m BEFORE running it:
%   cfg.compareToXDYN = true;
%   cfg.referenceCSV  = '';  % Automatic filename for the selected actuators
%     propeller/rudder -> mmg_updated.csv
%     sail only        -> sail_only.csv
%     both             -> combined.csv
% Put the matching XDYN CSV beside RUN_MODEL.m, then run RUN_MODEL.m once.
% Its CSV angle/rate unit settings must match the reference file.
%
% To redisplay existing results without reintegrating:
%   figures = show_results(results,true,false);
% To display AND save:
%   figures = show_results(results,true,true);
%
% All plots AND metric tables appear in ONE window with horizontal tabs.
% Click a tab, use Previous/Next, or use the left/right arrow keys.
% This function does not launch XDYN, read a new CSV, or rerun the simulation.
% No reference loaded? Guidance is shown in the same window, not fake errors.
% Internal/exported states: radians. Displayed heading/yaw rate: degrees.

if nargin<2, makePlots=true; end
if nargin<3, saveResults=true; end
validateattributes(makePlots,{'logical','numeric'},{'scalar','binary'});
validateattributes(saveResults,{'logical','numeric'},{'scalar','binary'});

% Keep the existing integration and comparison functions unchanged.
results.t=results.t(:);
t=results.t;
if numel(t)<2 || any(~isfinite(t)) || any(diff(t)<=0)
    error('MMG:ResultsTime','Results need at least two increasing finite times.');
end
if ~isequal(size(results.model.states),[numel(t),6])
    error('MMG:ResultsSize','Model states must be N-by-6: [x y psi u v r].');
end
hasReference=isfield(results,'reference') && ~isempty(results.reference);
if hasReference
    D=results.reference;
    if ~isequal(D.t(:),t) || ~isequal(size(D.states),size(results.model.states))
        error('MMG:ReferenceGrid', ...
            'Rerun RUN_MODEL with compareToXDYN=true; reference/model grids must match.');
    end
    if ~isfield(results,'comparison') || isempty(results.comparison)
        results.comparison=compare_ship_results(results);
    end
    % Existing loadMetrics use MATLAB evaluated at XDYN states (frozen-state).
    % Add actual simulated-load metrics so BOTH comparisons are available.
    results.comparison=add_trajectory_load_metrics(results);
else
    results.comparison=[];
end

figures=struct();
if makePlots, figures=make_dashboard(results); end
if saveResults, save_outputs(results,figures); end
end

function C=add_trajectory_load_metrics(R)
% Actual loads follow the independent MATLAB trajectory; frozen loads do not.
C=R.comparison;
active=logical([repmat(R.active.propellerRudder,1,3),repmat(R.active.sail,1,3)]);
M=[R.model.propellerRudder R.model.sail];
D=[R.reference.propellerRudder R.reference.sail];
E=M(:,active)-D(:,active);
if ~isreal(E) || any(~isfinite(E(:)))
    error('MMG:LoadComparison','Active MATLAB/XDYN load columns must be finite and real.');
end
names={'propellerRudder_Fx';'propellerRudder_Fy';'propellerRudder_Mz'; ...
       'sail_Fx';'sail_Fy';'sail_Mz'};
units={'N';'N';'N_m';'N';'N';'N_m'};
C.trajectoryLoadError=E;
C.trajectoryLoadMetrics=table(names(active),units(active), ...
    sqrt(mean(E.^2,1)).',mean(abs(E),1).',max(abs(E),[],1).',E(end,:).', ...
    'VariableNames',{'Load','Unit','RMSE','MAE','MaxAbsError','FinalError'});
end

function F=make_dashboard(R)
% Only this figure is visible. Every plot is explicitly parented to a tab.
% Replace a previous dashboard, but leave unrelated MATLAB figures alone.
tag='MMG_Compact_Tabbed_Results';
old=findall(groot,'Type','figure','Tag',tag);
if ~isempty(old), delete(old); end
hasReference=~isempty(R.comparison);
if hasReference
    status='MATLAB vs XDYN | Solid: XDYN | Dashed: independent MATLAB';
else
    status='MATLAB ONLY | No XDYN comparison loaded: see Metrics / Run details';
end
F.window=figure('Name',[R.title ' | Results'],'Tag',tag, ...
    'NumberTitle','off','Color','w','Units','normalized', ...
    'Position',[0.04 0.08 0.92 0.84],'WindowStyle','normal','Visible','off');
text_label(F.window,{R.title;status},[0.015 0.935 0.97 0.058],12,true);
F.tabGroup=uitabgroup('Parent',F.window,'Units','normalized', ...
    'Position',[0.01 0.085 0.98 0.84],'TabLocation','top');
keys={'states','trajectory','actuatorLoads','stateErrors','loadErrors','metrics','summary'};
names={'States','XY + distance','Loads','State errors','Load errors','Metrics','Run details'};
order=gobjects(1,numel(keys));
for k=1:numel(keys)
    order(k)=uitab(F.tabGroup,'Title',names{k},'BackgroundColor','w');
    F.tabs.(keys{k})=order(k);
end
F.layouts=struct();
F.layouts.states=state_panels(F.tabs.states,R,false);
F.layouts.trajectory=trajectory_panels(F.tabs.trajectory,R);
F.layouts.actuatorLoads=load_panels(F.tabs.actuatorLoads,R,false);
if hasReference
    F.layouts.stateErrors=state_panels(F.tabs.stateErrors,R,true);
    F.layouts.loadErrors=load_panels(F.tabs.loadErrors,R,true);
    metric_tables(F.tabs.metrics,R);
else
    missing_reference(F.tabs.stateErrors,R);
    missing_reference(F.tabs.loadErrors,R);
    missing_reference(F.tabs.metrics,R);
end
summary_panel(F.tabs.summary,R);

% Navigation callbacks carry only a UI handle, never a copy of model results.
nav.group=F.tabGroup; nav.tabs=order;
nav.previous=uicontrol(F.window,'Style','pushbutton','String','< Previous', ...
    'Units','normalized','Position',[0.02 0.02 0.13 0.045], ...
    'FontSize',11,'Callback',@(src,evt) navigate(src,-1));
nav.next=uicontrol(F.window,'Style','pushbutton','String','Next >', ...
    'Units','normalized','Position',[0.85 0.02 0.13 0.045], ...
    'FontSize',11,'Callback',@(src,evt) navigate(src,1));
nav.label=text_label(F.window,'',[0.18 0.017 0.64 0.045],10,false);
set(nav.label,'HorizontalAlignment','center');
setappdata(F.window,'MMGNavigation',nav);
F.tabGroup.SelectionChangedFcn=@(src,evt) navigate(src,0);
F.window.WindowKeyPressFcn=@key_navigation;
F.tabGroup.SelectedTab=order(1);
navigate(F.window,0);
F.window.Visible='on';
drawnow;
end

function L=state_panels(parent,R,showError)
% Positions/heading in row 1; surge/sway/yaw rate in row 2.
t=R.t; hasReference=~isempty(R.comparison);
scale=[1 1 180/pi 1 1 180/pi]; M=R.model.states.*scale;
names={'North position x','East position y','Heading \psi', ...
       'Surge velocity u','Sway velocity v','Yaw rate r'};
labels={'x (m)','y (m)','\psi (deg)','u (m/s)','v (m/s)','r (deg/s)'};
L=tiledlayout(parent,2,3,'TileSpacing','compact','Padding','compact');
for j=1:6
    ax=nexttile(L); hold(ax,'on');
    if showError
        C=R.comparison;
        plot(ax,t,C.stateErrorDisplay(:,j),'-','LineWidth',1.4);
        yline(ax,0,':','HandleVisibility','off');
        title(ax,{names{j},sprintf('RMSE = %.5g %s', ...
            C.stateMetrics.RMSE(j),C.stateMetrics.Unit{j})});
        ylabel(ax,['Error: ' labels{j}]);
    else
        if hasReference
            plot(ax,t,R.comparison.referenceDisplay(:,j),'-','LineWidth',1.8, ...
                'DisplayName','XDYN CSV');
        end
        plot(ax,t,M(:,j),'--','LineWidth',1.5,'DisplayName','MATLAB');
        title(ax,names{j}); ylabel(ax,labels{j});
        if j==1, legend(ax,'Location','best'); end
    end
    style_axes(ax); xlim(ax,[t(1) t(end)]); xlabel(ax,'Time (s)');
end
if showError
    heading='State errors: MATLAB minus XDYN';
elseif hasReference
    heading='Six-state comparison: MATLAB vs XDYN';
else
    heading='Six-state MATLAB simulation (no reference loaded)';
end
title(L,{R.title;heading},'Interpreter','none');
end

function L=trajectory_panels(parent,R)
% Separation is sqrt((x_M-x_D)^2+(y_M-y_D)^2), NOT a path-length difference.
hasReference=~isempty(R.comparison); M=R.model.states; t=R.t;
L=tiledlayout(parent,1,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(L); hold(ax,'on');
if hasReference
    D=R.reference.states;
    plot(ax,D(:,1),D(:,2),'-','LineWidth',1.9,'DisplayName','XDYN CSV');
end
plot(ax,M(:,1),M(:,2),'--','LineWidth',1.5,'DisplayName','MATLAB');
plot(ax,M(1,1),M(1,2),'o','LineWidth',1.2,'MarkerSize',8,'DisplayName','Start');
if hasReference
    plot(ax,D(end,1),D(end,2),'s','LineWidth',1.5,'MarkerSize',9,'DisplayName','XDYN end');
end
plot(ax,M(end,1),M(end,2),'d','LineWidth',1.3,'MarkerSize',7,'DisplayName','MATLAB end');
style_axes(ax); axis(ax,'equal');
xlabel(ax,'North position x (m)'); ylabel(ax,'East position y (m)');
title(ax,'XY trajectory'); legend(ax,'Location','best');
ax=nexttile(L); hold(ax,'on');
if hasReference
    C=R.comparison;
    plot(ax,t,C.planarSeparation,'-','LineWidth',1.6);
    plot(ax,t(end),C.planarSeparation(end),'o','LineWidth',1.3,'MarkerSize',7);
    title(ax,{'Distance between MATLAB and XDYN positions', ...
        sprintf('Final = %.6g m | Maximum = %.6g m', ...
        C.planarSeparation(end),max(C.planarSeparation))});
    ylabel(ax,'Planar separation (m)');
else
    distance=[0;cumsum(hypot(diff(M(:,1)),diff(M(:,2))))];
    plot(ax,t,distance,'--','LineWidth',1.6);
    title(ax,{'MATLAB travelled path length','No XDYN comparison loaded'});
    ylabel(ax,'Cumulative path length (m)');
end
style_axes(ax); xlim(ax,[t(1) t(end)]); xlabel(ax,'Time (s)');
title(L,R.title,'Interpreter','none');
end

function L=load_panels(parent,R,showError)
% Solid: XDYN. Dashed: actual independently simulated MATLAB loads.
% Dotted: MATLAB re-evaluated at XDYN states (load-law diagnostic only).
% Use the original comparison's frozen-state quantities; do not reintegrate.
t=R.t; hasReference=~isempty(R.comparison);
active=[repmat(R.active.propellerRudder,1,3),repmat(R.active.sail,1,3)];
indices=find(active);
M=[R.model.propellerRudder R.model.sail];
if hasReference
    D=[R.reference.propellerRudder R.reference.sail];
    B=[R.comparison.frozenPropellerRudder R.comparison.frozenSail];
end
names={'Propeller + rudder: F_x','Propeller + rudder: F_y', ...
       'Propeller + rudder: M_z','Sail: F_x','Sail: F_y','Sail: M_z'};
labels={'Force (N)','Force (N)','Moment (N m)','Force (N)','Force (N)','Moment (N m)'};
L=tiledlayout(parent,numel(indices)/3,3,'TileSpacing','compact','Padding','compact');
for k=1:numel(indices)
    j=indices(k); ax=nexttile(L); hold(ax,'on');
    if showError
        plot(ax,t,M(:,j)-D(:,j),'--','LineWidth',1.5,'DisplayName','Trajectory-load error');
        plot(ax,t,B(:,j)-D(:,j),':','LineWidth',1.8,'DisplayName','Frozen-state load error');
        yline(ax,0,':','HandleVisibility','off');
        title(ax,{names{j},sprintf('Trajectory RMSE %.4g | Frozen RMSE %.4g', ...
            R.comparison.trajectoryLoadMetrics.RMSE(k),R.comparison.loadMetrics.RMSE(k))});
        ylabel(ax,['Error: ' labels{j}]);
    else
        if hasReference
            plot(ax,t,D(:,j),'-','LineWidth',1.8,'DisplayName','XDYN CSV');
        end
        plot(ax,t,M(:,j),'--','LineWidth',1.5,'DisplayName','MATLAB trajectory');
        if hasReference
            plot(ax,t,B(:,j),':','LineWidth',1.8,'DisplayName','MATLAB at XDYN state');
        end
        title(ax,names{j}); ylabel(ax,labels{j});
    end
    style_axes(ax); xlim(ax,[t(1) t(end)]); xlabel(ax,'Time (s)');
    if k==1, legend(ax,'Location','best'); end
end
if showError
    heading='Actuator errors: MATLAB minus XDYN (two evaluation methods)';
elseif hasReference
    heading='Actuator loads: XDYN, MATLAB trajectory, and frozen-state diagnostic';
else
    heading='Actuator loads along the MATLAB trajectory (no reference loaded)';
end
title(L,{R.title;heading},'Interpreter','none');
end

function metric_tables(parent,R)
% Table data are formatted for readability only; exported values retain precision.
C=R.comparison;
text_label(parent,'State errors: MATLAB minus XDYN',[0.02 0.925 0.96 0.045],12,true);
draw_table(parent,C.stateMetrics,[0.02 0.615 0.96 0.30]);
text_label(parent,'Actuator errors: trajectory loads and frozen-state diagnostics', ...
    [0.02 0.545 0.96 0.045],12,true);
A=C.trajectoryLoadMetrics; B=C.loadMetrics;
A=[table(repmat({'Trajectory'},height(A),1),'VariableNames',{'Evaluation'}),A];
B=[table(repmat({'Frozen state'},height(B),1),'VariableNames',{'Evaluation'}),B];
draw_table(parent,[A;B],[0.02 0.085 0.96 0.445]);
text_label(parent,{ ...
    'NRMSE (%) = 100 x RMSE / XDYN range; NaN means undefined for a constant reference.'; ...
    'Trajectory = independently simulated states. Frozen state = evaluated at XDYN states.'}, ...
    [0.02 0.01 0.96 0.063],10,false);
end

function summary_panel(parent,R)
% Final-state comparison, distance metrics and run provenance in the same window.
t=R.t; hasReference=~isempty(R.comparison); scale=[1 1 180/pi 1 1 180/pi];
cfg=R.settings;
if hasReference
    reference=['Reference: ' R.reference.csvFile];
else
    reference=['No XDYN CSV loaded. Expected filename: ' expected_csv(R)];
end
text_label(parent,{R.title; ...
    sprintf('Time %.6g to %.6g s | %d samples',t(1),t(end),numel(t)); ...
    sprintf('Propeller/rudder: %d | Sail: %d',R.active.propellerRudder,R.active.sail); ...
    sprintf('RPM %.6g | Rudder %.6g deg | Wind %.6g m/s toward %.6g deg', ...
        cfg.rpm,cfg.rudderAngleDeg,cfg.windSpeed,cfg.windDirectionDeg); ...
    reference},[0.02 0.77 0.96 0.205],11,false);
state={'x';'y';'psi';'u';'v';'r'}; unit={'m';'m';'deg';'m/s';'m/s';'deg/s'};
last=(R.model.states(end,:).*scale).';
T=table(state,unit,last,'VariableNames',{'State','Unit','MATLAB_final'});
if hasReference
    T.XDYN_final=(R.reference.states(end,:).*scale).';
    T.FinalError=R.comparison.stateErrorDisplay(end,:).';
end
text_label(parent,'Final-state values',[0.02 0.705 0.96 0.045],12,true);
draw_table(parent,T,[0.02 0.415 0.96 0.285]);
text_label(parent,'Trajectory metrics',[0.02 0.35 0.96 0.045],12,true);
if hasReference
    A=R.comparison.trajectoryMetrics;
    name={'Final planar separation';'Maximum planar separation';'Mean planar separation'; ...
          'RMS planar separation';'XDYN path length';'Final separation / XDYN path';'MATLAB path length'};
    unit={'m';'m';'m';'m';'m';'%';'m'};
    value=[A.FinalPlanarSeparation_m;A.MaxPlanarSeparation_m;A.MeanPlanarSeparation_m; ...
        A.RMSPlanarSeparation_m;A.XDYNPathLength_m;A.FinalSeparation_percent;A.MATLABPathLength_m];
else
    name={'MATLAB path length'}; unit={'m'};
    value=sum(hypot(diff(R.model.states(:,1)),diff(R.model.states(:,2))));
end
T=table(name,unit,value,'VariableNames',{'Metric','Unit','Value'});
draw_table(parent,T,[0.02 0.02 0.96 0.325]);
end

function missing_reference(parent,R)
% No file-picker popup and no invented reference. A real CSV is required.
text_label(parent,{ ...
    'XDYN COMPARISON HAS NOT BEEN LOADED';''; ...
    'In section 2 of RUN_MODEL.m, set:'; ...
    '    cfg.compareToXDYN = true;'; ...
    '    cfg.referenceCSV  = '''';';''; ...
    ['Place ' expected_csv(R) ' in the same folder as RUN_MODEL.m.']; ...
    'Then run RUN_MODEL.m once, not only the results section.';''; ...
    'Changing the switch does not update an already-computed results variable.'; ...
    'The reference must match your actuator selection, RPM, rudder and wind inputs.'; ...
    'Check cfg.csvAngleUnit and cfg.csvYawRateUnit before comparing.'}, ...
    [0.055 0.12 0.89 0.76],13,false);
end

function name=expected_csv(R)
if R.active.propellerRudder && R.active.sail
    name='combined.csv';
elseif R.active.propellerRudder
    name='mmg_updated.csv';
else
    name='sail_only.csv';
end
end

function h=text_label(parent,value,position,fontSize,bold)
h=uicontrol(parent,'Style','text','String',value,'Units','normalized', ...
    'Position',position,'BackgroundColor','w','HorizontalAlignment','left', ...
    'FontName','Arial','FontSize',fontSize);
if bold, set(h,'FontWeight','bold'); end
end

function h=draw_table(parent,T,position)
% Character-formatted numbers display small errors instead of rounding to 0.
% Full-precision numeric tables are still saved separately with writetable.
data=table2cell(T); widths=cell(1,width(T));
for j=1:width(T)
    maxChars=numel(T.Properties.VariableNames{j});
    for i=1:height(T)
        value=data{i,j};
        if isnumeric(value) && isscalar(value)
            data{i,j}=sprintf('%.7g',value);
        elseif isstring(value)
            data{i,j}=char(value);
        end
        if ischar(data{i,j}), maxChars=max(maxChars,numel(data{i,j})); end
    end
    widths{j}=max(85,min(320,7*maxChars+24));
end
h=uitable('Parent',parent,'Units','normalized','Position',position, ...
    'Data',data,'ColumnName',T.Properties.VariableNames,'RowName',[], ...
    'ColumnWidth',widths,'ColumnEditable',false(1,width(T)), ...
    'FontName','Arial','FontSize',10);
end

function style_axes(ax)
grid(ax,'on'); box(ax,'on');
set(ax,'FontName','Arial','FontSize',11,'LineWidth',0.8, ...
    'TickDir','out','GridAlpha',0.18,'Layer','top');
end

function navigate(src,delta)
fig=ancestor(src,'figure');
if isgraphics(src,'figure'), fig=src; end
if isempty(fig) || ~isappdata(fig,'MMGNavigation'), return; end
N=getappdata(fig,'MMGNavigation');
i=find(N.tabs==N.group.SelectedTab,1);
if isempty(i), i=1; end
j=min(max(i+delta,1),numel(N.tabs));
if j~=i, N.group.SelectedTab=N.tabs(j); end
set(N.previous,'Enable','on'); set(N.next,'Enable','on');
if j==1, set(N.previous,'Enable','off'); end
if j==numel(N.tabs), set(N.next,'Enable','off'); end
set(N.label,'String',sprintf('%d / %d  |  %s  |  Click a tab or use Previous / Next', ...
    j,numel(N.tabs),N.tabs(j).Title));
end

function key_navigation(src,event)
switch event.Key
    case 'leftarrow', navigate(src,-1);
    case 'rightarrow', navigate(src,1);
end
end

function save_outputs(R,F)
% Preserve CSV/MAT outputs. Save ONE editable dashboard and each plot tab.
% No source YAML or input CSV is changed. Repeating a run overwrites outputs.
out=R.settings.outputDir;
if ~isfolder(out), mkdir(out); end
t=R.t; M=R.model;
stateNames={'t_s','x_m','y_m','psi_rad','u_m_s','v_m_s','r_rad_s'};
loadNames={'t_s','propellerRudder_Fx_N','propellerRudder_Fy_N', ...
    'propellerRudder_Mz_Nm','sail_Fx_N','sail_Fy_N','sail_Mz_Nm'};
writetable(array2table([t M.states],'VariableNames',stateNames),fullfile(out,'matlab_states.csv'));
writetable(array2table([t M.propellerRudder M.sail],'VariableNames',loadNames), ...
    fullfile(out,'matlab_actuator_loads.csv'));
if ~isempty(R.comparison)
    C=R.comparison;
    writetable(C.stateMetrics,fullfile(out,'state_metrics.csv'));
    writetable(C.trajectoryMetrics,fullfile(out,'trajectory_metrics.csv'));
    writetable(C.loadMetrics,fullfile(out,'load_metrics.csv')); % Frozen-state, as before
    writetable(C.trajectoryLoadMetrics,fullfile(out,'trajectory_load_metrics.csv'));
    names={'t_s','x_error_m','y_error_m','psi_error_deg','u_error_m_s', ...
           'v_error_m_s','r_error_deg_s','planar_separation_m'};
    writetable(array2table([t C.stateErrorDisplay C.planarSeparation],'VariableNames',names), ...
        fullfile(out,'state_error_history.csv'));
    writetable(array2table([t R.reference.states],'VariableNames',stateNames), ...
        fullfile(out,'reference_states_normalized.csv'));
    writetable(array2table([t C.frozenPropellerRudder C.frozenSail],'VariableNames',loadNames), ...
        fullfile(out,'matlab_loads_at_xdyn_states.csv'));
end
results=R; %#ok<NASGU> Preserve the saved variable name used by previous code.
save(fullfile(out,'results.mat'),'results');
if ~isfield(F,'window') || ~isgraphics(F.window,'figure')
    fprintf('Results saved in: %s\n',out);
    return;
end

% Select each plot tab for rendering; restore the user's original tab afterward.
selected=F.tabGroup.SelectedTab;
restore=onCleanup(@() restore_tab(F.tabGroup,selected)); %#ok<NASGU>
names=fieldnames(F.layouts);
for k=1:numel(names)
    name=names{k}; F.tabGroup.SelectedTab=F.tabs.(name); drawnow;
    L=F.layouts.(name); base=fullfile(out,name);
    try
        exportgraphics(L,[base '.png'],'Resolution',300);
    catch ME
        warning('MMG:PNGExport','Could not export %s.png: %s',name,ME.message);
    end
    try
        exportgraphics(L,[base '.svg'],'ContentType','vector');
    catch
        % Older MATLAB releases cannot export SVG with exportgraphics.
        % A temporary INVISIBLE figure is used only for file export.
        try
            export_svg_legacy(L,[base '.svg']);
        catch ME
            warning('MMG:SVGExport','Could not export %s.svg: %s',name,ME.message);
        end
    end
end
restore_tab(F.tabGroup,selected);
try
    savefig(F.window,fullfile(out,'results_dashboard.fig'));
catch ME
    warning('MMG:FIGExport','Could not save tabbed dashboard: %s',ME.message);
end
% Keep metrics inside the window; only a short save location is printed.
fprintf('Results saved in: %s\n',out);
end

function restore_tab(group,tab)
if isgraphics(group) && isgraphics(tab), group.SelectedTab=tab; end
end

function export_svg_legacy(layout,filename)
% Do not expose an additional results window during legacy SVG export.
f=figure('Visible','off','Color','w','Position',[50 50 1320 760], ...
    'HandleVisibility','off','PaperPositionMode','auto');
cleanup=onCleanup(@() delete(f)); %#ok<NASGU>
copyobj(layout,f); drawnow;
print(f,filename,'-dsvg','-painters');
end
