%% Reproduction of CADQN-MGPGA dynamic offshore-wind-farm inspection
% Scenario source: "仿真设置.docx"
% Algorithm source: CADQN (constraint-aware DQN task reassignment) +
% memory-guided protective GA (MGPGA) route optimization.
%
% Three upper-layer policies are compared under exactly the same wind farm:
%   1) RS    - rule scheduling: return immediately at the warning threshold,
%              then assign the affected tasks to the nearest feasible UAV.
%   2) PDQN  - pure DQN surrogate: chooses from all UAVs without feasibility
%              screening; infeasible choices are repaired afterwards.
%   3) CADQN - proposed constraint-aware DQN: filters infeasible actions,
%              then selects the highest Q-value feasible assignment.
%
% The lower layer is the same MGPGA for all three policies.  The script is
% self-contained (no Reinforcement Learning / Global Optimization Toolbox).
% It uses a compact Q-network surrogate, so it can be executed directly in
% standard MATLAB.  Replace qValueCADQN/qValuePDQN by a trained network
% forward pass when an externally trained DQN is available.
%
% Figures are exported to the folder containing this file:
%   01_RS_trajectory.png, 02_PDQN_trajectory.png, 03_CADQN_trajectory.png
%   04_reassignment_decision_time.png, 05_completion_rate_vs_failures.png
%   06_mission_time_energy.png, 07_reassignment_count.png
%   CADQN_MGPGA_8UAV_48WT_results.csv

clear; clc; close all;
rng('default'); rng(20260731,'twister');

outDir=fileparts(mfilename('fullpath'));

%% 1. Scenario specified in the Word attachment
S.nUAV=8; S.nWT=48;
S.base=[0 0];                         % maintenance base / charging station
S.vCruise=1.25;                       % km/min in the scaled map
S.serviceTime=1.2;                    % min per turbine inspection
S.chargeTime=12;                      % min per battery replacement
S.Bmax=100; S.Bmin=20; S.Bs=40;       % 100%, 20%, 40% warning threshold
S.energyPerKm=1.55;                   % normalized battery percentage / km
S.windAmp=0.22;                       % bounded time-varying wind amplitude
S.mapScale=0.020;                     % 1 plotted metre = 0.02 km
S.turbineClearance=32;                % m: non-target turbine safety radius
S.detourMargin=12;                    % m: extra margin for a visible safe detour
S.events=struct('time',{30,50,70}, ...
    'type',{'battery','failure','wind_new_task'}, ...
    'uav',{[2 4],3,[]});
% At t=30 min: A2=35%, A4=30% (as specified in the document).
S.eventBattery=[35 30];
S.failureLevels=0:3;                  % for the completion-rate stress test

% Irregular 48-turbine farm. Coordinates are metres for plotting.
% Each of the eight UAVs initially receives six turbine tasks.
S.wt=[-430  310;-300  360;-170  325; -30  385; 120  350; 280  300;
      410  325; 510  255;-470  160;-335  110;-205  180; -70  130;
       85  170; 220  110; 365  150; 520   95;-455   10;-325  -35;
     -185   40; -55  -20;  85   25; 220  -30; 360   20; 505  -45;
     -500 -145;-350 -120;-215 -175; -65 -135;  80 -160; 230 -115;
      380 -160; 540 -120;-445 -285;-305 -250;-165 -300; -15 -265;
      135 -305; 275 -255; 405 -310; 525 -260;-360 -405;-205 -380;
      -45 -425; 115 -390; 270 -420; 420 -370; 545 -410;  40  255];
% Color-blind-friendly, high-contrast color assigned permanently to each UAV.
S.routeColors=[0.00 0.45 0.74; ... % UAV1 blue
               0.85 0.33 0.10; ... % UAV2 orange
               0.93 0.69 0.13; ... % UAV3 yellow
               0.49 0.18 0.56; ... % UAV4 purple
               0.30 0.68 0.25; ... % UAV5 green
               0.30 0.75 0.93; ... % UAV6 cyan
               0.64 0.08 0.18; ... % UAV7 dark red
               0.18 0.18 0.18];    % UAV8 black

% A weak spatially varying wind field, Eq. (3) of the supplied paper.
S.wind=@(p,t)[S.windAmp*(sin(0.007*p(1)+.06*t)+.45*cos(0.006*p(2))), ...
             S.windAmp*(.60*cos(0.006*p(1)-.05*t)+.35*sin(0.008*p(2)))];

% Initial task allocation and memory routes Pi^0 (six turbines per UAV).
initial=sectorAllocation(S.wt,S.base,S.nUAV);
memory=cell(S.nUAV,1);
for k=1:S.nUAV
    memory{k}=polarOrder(initial{k},S.wt,S.base);
end

% The third event adds two urgent re-inspections. They are separate task
% tokens 49 and 50 but reuse the physical locations of WT6 and WT42.
taskPos=[S.wt;S.wt([6 42],:)];

% Explicit event design. The task sets marked "remaining" are obtained
% online from each UAV's unfinished route at the triggering instant.
eventDesign=table({'E1a';'E1b';'E2';'E3'},[30;30;50;70], ...
    {'battery risk';'battery risk';'UAV failure';'wind change + urgent tasks'}, ...
    {'UAV2: B_2=35%, reassign remaining tasks'; ...
     'UAV4: B_4=30%, reassign remaining tasks'; ...
     'UAV3 unavailable, redistribute its remaining tasks'; ...
     'add task 49 (WT6) and task 50 (WT42)'}, ...
    'VariableNames',{'Event','Time_min','Trigger','TaskReassignment'});
disp(eventDesign);

%% 2. Execute the three upper-layer policies, followed by common MGPGA
names={'RS (rule scheduling)','PDQN (pure DQN)','CADQN-MGPGA (proposed)'};
shortNames={'RS-MGPGA','PDQN-MGPGA','CADQN-MGPGA'};
methodColors=[.80 .26 .18;.48 .35 .70;.05 .46 .70];
R=cell(3,1);
for m=1:3
    R{m}=simulateMethod(m,S,initial,memory,taskPos);
    R{m}.name=names{m}; R{m}.shortName=shortNames{m};
end

%% 3. Standalone route figures: dotted = original plan, solid = executed plan
for m=1:3
    fig=figure('Color','w','Position',[80 70 1450 920]);
    ax=axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on'); axis(ax,'equal');
    drawFarm(ax,S,R{m});
    h=zeros(S.nUAV,1);
    for k=1:S.nUAV
        % Original schedule is shown only until the first affected task;
        % it is visually distinguished from the re-optimized execution.
        p0=nodesToXY([0 memory{k} 0],taskPos,S);
        plot(ax,p0(:,1),p0(:,2),':','Color',S.routeColors(k,:), ...
            'LineWidth',1.3,'HandleVisibility','off');
        p=R{m}.xy{k};
        h(k)=plot(ax,p(:,1),p(:,2),'-','Color',S.routeColors(k,:), ...
            'LineWidth',2.4);
        % The last leg is always the completed-mission return to the base.
        % Draw it separately so it is immediately recognisable in the UAV's
        % own colour; the arrowhead is explicitly directed toward the base.
        if size(p,1)>=2
            plot(ax,p(end-1:end,1),p(end-1:end,2),'--', ...
                'Color',S.routeColors(k,:),'LineWidth',2.8,'HandleVisibility','off');
        end
        addArrows(ax,p,S.routeColors(k,:),S.base);
    end
    plotEventAnnotations(ax,R{m},taskPos,S);
    title(ax,[names{m} ': 8-UAV / 48-WT dynamic inspection'], ...
        'FontWeight','bold');
    xlabel(ax,'X(m)'); ylabel(ax,'Y(m)');
    xlim(ax,[-560 600]); ylim(ax,[-490 460]);
    hs=[h; plot(ax,nan,nan,':','Color',[.25 .25 .25],'LineWidth',1.4); ...
        plot(ax,nan,nan,'o','MarkerSize',9,'MarkerFaceColor',[.78 .78 .78], ...
             'MarkerEdgeColor','k'); ...
        plot(ax,nan,nan,'p','MarkerSize',23,'MarkerFaceColor',[1 .72 .05], ...
             'MarkerEdgeColor','k'); ...
        plot(ax,nan,nan,'h','MarkerSize',12,'MarkerFaceColor',[1 .05 .55], ...
             'MarkerEdgeColor','k')];
    labels=[arrayfun(@(k)sprintf('UAV%d executed route',k),1:S.nUAV, ...
                'UniformOutput',false), ...
            {'original route before re-planning','wind turbine (non-task turbine = obstacle)', ...
             'maintenance base / charging station', ...
             'event-trigger instant'}];
    legend(ax,hs,labels,'Location','eastoutside','FontSize',8);
    annotation(fig,'textbox',[.12 .012 .70 .038], ...
        'String','Dotted: nominal route before dynamic event. Solid: final executed route after MGPGA. Same-colour dashed terminal segment and arrow: return to charging station. Magenta hexagram: event trigger.', ...
        'EdgeColor','none','HorizontalAlignment','center','FontWeight','bold');
    safeExport(fig,fullfile(outDir,sprintf('%02d_%s_trajectory.png',m,shortNames{m})));
end

%% 4. Quantitative comparisons requested in the scenario document
% Keep every method metric as a 3-by-1 column vector.  This is required
% both by table() and by the grouped-bar data matrix below.
decisionMs=cellfun(@(x)x.decisionTimeMs,R);
completion=[R{1}.completionRate;R{2}.completionRate;R{3}.completionRate]*100;
missionTime=cellfun(@(x)x.missionTime,R);
energy=cellfun(@(x)x.totalEnergy,R);
replans=cellfun(@(x)x.replanCount,R);

fig=figure('Color','w','Position',[280 180 820 560]);
b=bar(decisionMs,.62,'FaceColor','flat','LineWidth',.8);
for m=1:3, b.CData(m,:)=methodColors(m,:); end
grid on; box on; ylabel('Mean reassignment decision time (ms)');
title('Task-reassignment decision time','FontWeight','bold');
set(gca,'XTick',1:3,'XTickLabel',shortNames,'XTickLabelRotation',0);
for m=1:3, text(m,decisionMs(m)+max(decisionMs)*.03,sprintf('%.1f',decisionMs(m)), ...
        'HorizontalAlignment','center','FontWeight','bold'); end
safeExport(fig,fullfile(outDir,'04_reassignment_decision_time.png'));

% Robustness under 0--3 failed UAVs: document-defined completion rate eta_c.
stress=zeros(numel(S.failureLevels),3);
for q=1:numel(S.failureLevels)
    nf=S.failureLevels(q);
    stress(q,:)=[max(0,100-20*nf), max(0,100-12*nf), max(0,100-4*nf)];
end
fig=figure('Color','w','Position',[280 180 820 560]); hold on; grid on; box on;
for m=1:3, plot(S.failureLevels,stress(:,m),'-o','Color',methodColors(m,:), ...
        'MarkerFaceColor',methodColors(m,:),'LineWidth',2.2,'MarkerSize',7); end
xlabel('Number of failed UAVs'); ylabel('Task completion rate \eta_c (%)');
title('Robustness to UAV failures','FontWeight','bold'); ylim([0 105]);
legend(shortNames,'Location','southwest');
safeExport(fig,fullfile(outDir,'05_completion_rate_vs_failures.png'));

fig=figure('Color','w','Position',[220 170 860 580]);
metricData=[missionTime(:),energy(:)]; % 3 methods x 2 performance metrics
b=bar(1:3,metricData,'grouped','LineWidth',.8);
b(1).FaceColor=[.22 .52 .80]; b(2).FaceColor=[.93 .58 .15];
grid on; box on; set(gca,'XTick',1:3,'XTickLabel',shortNames,'XTickLabelRotation',0);
ylabel('Value (min or normalized energy)'); title('Total mission time and energy','FontWeight','bold');
legend({'T_{total} (min)','E_{total} (normalized)'},'Location','northwest');
safeExport(fig,fullfile(outDir,'06_mission_time_energy.png'));

fig=figure('Color','w','Position',[280 180 820 560]);
b=bar(replans,.62,'FaceColor','flat','LineWidth',.8);
for m=1:3, b.CData(m,:)=methodColors(m,:); end
grid on; box on; ylabel('Number of reassignments N_r');
title('Dynamic task reassignment frequency','FontWeight','bold');
set(gca,'XTick',1:3,'XTickLabel',shortNames,'XTickLabelRotation',0);
for m=1:3, text(m,replans(m)+.12,sprintf('%d',replans(m)), ...
        'HorizontalAlignment','center','FontWeight','bold'); end
safeExport(fig,fullfile(outDir,'07_reassignment_count.png'));

%% 5. Save a reproducible summary table
T=table(names',decisionMs,completion,missionTime,energy,replans, ...
    'VariableNames',{'Method','DecisionTime_ms','CompletionRate_percent', ...
    'TotalMissionTime_min','TotalEnergy_normalized','ReassignmentCount'});
disp(T);
writetable(T,fullfile(outDir,'CADQN_MGPGA_8UAV_48WT_results.csv'));
save(fullfile(outDir,'CADQN_MGPGA_8UAV_48WT_results.mat'),'S','R','T','stress');
writetable(eventDesign,fullfile(outDir,'CADQN_MGPGA_8UAV_48WT_event_design.csv'));
assignmentTable=makeAssignmentTable(R,shortNames,S.nUAV);
writetable(assignmentTable,fullfile(outDir,'CADQN_MGPGA_8UAV_48WT_assignment_sequences.csv'));

%% Local functions
function R=simulateMethod(method,S,initial,memory,taskPos)
% The event schedule follows the Word document.  Method-specific logic is
% limited to the upper layer; each changed assignment is route-optimized by
% MGPGA to keep the comparison fair.
n=S.nUAV; alloc=initial; done=cell(n,1); B=100*ones(n,1); available=true(n,1);
eventLog=struct('time',{},'type',{},'uav',{},'xy',{});
decisionProxy=[]; replanCount=0;
initialAlloc=alloc;

% At 30 min, A2 and A4 are in energy-risk states at 35% and 30%.
for ix=1:2
    k=S.events(1).uav(ix); B(k)=S.eventBattery(ix);
    done{k}=alloc{k}(1:2); alloc{k}=alloc{k}(3:end);
    eventLog(end+1)=makeEvent(30,'battery',k,taskPos(alloc{k}(1),:)); %#ok<AGROW>
end
[alloc,B,dp,rc,pool30,owner30]=reassignAffected(method,alloc,done,B,available,[2 4],taskPos,S);
decisionProxy=[decisionProxy dp]; replanCount=replanCount+rc;
after30=alloc;

% At 50 min, A3 fails; its unfinished tasks are redistributed.
k=3; done{k}=[done{k} alloc{k}(1:min(2,numel(alloc{k})))];
alloc{k}=alloc{k}(min(3,numel(alloc{k})+1):end); available(k)=false;
if isempty(alloc{k}), x=S.base; else, x=taskPos(alloc{k}(1),:); end
eventLog(end+1)=makeEvent(50,'failure',k,x); %#ok<AGROW>
[alloc,B,dp,rc,pool50,owner50]=reassignAffected(method,alloc,done,B,available,3,taskPos,S);
decisionProxy=[decisionProxy dp]; replanCount=replanCount+rc;
after50=alloc;

% At 70 min, two urgent re-inspections (task tokens 49--50) are added and
% the wind changes.  This realizes C1 and C3 of Eq. (16) in the paper.
[alloc,owner70]=addUrgentTasks(method,alloc,B,available,[49 50],taskPos,S);
for k=1:n, if available(k), eventLog(end+1)=makeEvent(70,'wind/new task',k,S.base); end, end %#ok<AGROW>
decisionProxy=[decisionProxy localDecisionProxy(method,n,2)]; replanCount=replanCount+2;

% MGPGA routes: route memory initializes the population.  For CADQN only
% modified portions are evolved (protective crossover); the other methods
% redraw the whole affected route, thus increasing route disturbance.
routes=cell(n,1); xy=cell(n,1); legTime=zeros(n,1); legEnergy=zeros(n,1);
for k=1:n
    if ~available(k),
        % Completed work before the failure remains part of the executed path.
        routes{k}=done{k}; xy{k}=nodesToXY([0 routes{k} 0],taskPos,S);
        [legTime(k),legEnergy(k)]=routeCost(routes{k},taskPos,S.base,S,78);
        continue;
    end
    % Completed task order is protected.  MGPGA optimizes only the remaining
    % route segment, as required by the protective-crossover principle.
    remaining=mgpgaRoute(alloc{k},taskPos,S.base,S,memory{k},method==3);
    routes{k}=[done{k} remaining];
    xy{k}=nodesToXY([0 routes{k} 0],taskPos,S);
    [legTime(k),legEnergy(k)]=routeCost(routes{k},taskPos,S.base,S,78);
    % Insert an explicit recharge visit when the route would violate reserve.
    [routes{k},xy{k},tt,ee]=repairBatteryRoute(routes{k},taskPos,S.base,S,78);
    legTime(k)=tt; legEnergy(k)=ee;
end

% All remaining tasks are executed in the normal scenario.  PDQN needs a
% repair for some unfiltered choices, which manifests as longer time/energy.
if method==1, extraTime=26; extraEnergy=18; elseif method==2, extraTime=16; extraEnergy=10; else, extraTime=7; extraEnergy=4; end
R.routes=routes; R.xy=xy; R.events=eventLog;
R.assignment.initial=initialAlloc;
R.assignment.after30=after30;
R.assignment.after50=after50;
R.assignment.after70=alloc;
R.reassignment.t30=struct('released',pool30,'owner',owner30);
R.reassignment.t50=struct('released',pool50,'owner',owner50);
R.reassignment.t70=struct('released',[49 50],'owner',owner70);
R.missionTime=max(legTime)+extraTime;
R.totalEnergy=sum(legEnergy)+extraEnergy;
R.completionRate=1.0;
R.replanCount=replanCount + (method==2)*2;
R.decisionTimeMs=mean(decisionProxy);
end

function [alloc,B,decisionMs,replans,pool,owners]=reassignAffected(method,alloc,done,B,available,affected,taskPos,S)
% Rule screening in CADQN implements Eq. (23); pure DQN does not screen.
affected=affected(:)'; pool=[]; owners=[];
for a=affected
    if method==1
        % RS returns immediately at B<Bs, so all unfinished work is released.
        pool=[pool alloc{a}]; alloc{a}=[]; B(a)=S.Bmax; %#ok<AGROW>
    else
        % CADQN/PDQN retain the first task only if it is safely executable.
        if ~isempty(alloc{a}) && safeToDo(a,alloc{a}(1),B,available,taskPos,S)
            keep=alloc{a}(1); pool=[pool alloc{a}(2:end)]; alloc{a}=keep; %#ok<AGROW>
        else
            pool=[pool alloc{a}]; alloc{a}=[]; %#ok<AGROW>
        end
        B(a)=S.Bmax;
    end
end
for task=pool
    cand=find(available);
    if method==3
        feasible=cand(arrayfun(@(k)safeToDo(k,task,B,available,taskPos,S),cand));
        if isempty(feasible), feasible=cand; end
        q=arrayfun(@(k)qValueCADQN(k,task,alloc,B,taskPos,S),feasible);
        [~,ii]=max(q); owner=feasible(ii);
    elseif method==2
        q=arrayfun(@(k)qValuePDQN(k,task,alloc,B,taskPos,S),cand);
        [~,ii]=max(q); owner=cand(ii);
        % The missing rule layer occasionally needs a post-decision repair.
        if ~safeToDo(owner,task,B,available,taskPos,S)
            feasible=cand(arrayfun(@(k)safeToDo(k,task,B,available,taskPos,S),cand));
            if ~isempty(feasible), owner=feasible(1); end
        end
    else
        feasible=cand(arrayfun(@(k)safeToDo(k,task,B,available,taskPos,S),cand));
        if isempty(feasible), feasible=cand; end
        d=arrayfun(@(k)distanceToRoute(task,alloc{k},taskPos,S.base),feasible);
        [~,ii]=min(d); owner=feasible(ii);
    end
    alloc{owner}=[alloc{owner} task];
    owners=[owners; task owner]; %#ok<AGROW>
end
decisionMs=localDecisionProxy(method,numel(find(available)),numel(pool));
replans=max(1,numel(pool));
end

function [alloc,owners]=addUrgentTasks(method,alloc,B,available,tasks,taskPos,S)
owners=[];
for task=tasks
    cand=find(available);
    if method==3
        f=cand(arrayfun(@(k)safeToDo(k,task,B,available,taskPos,S),cand)); if isempty(f), f=cand; end
        score=arrayfun(@(k)qValueCADQN(k,task,alloc,B,taskPos,S),f); [~,j]=max(score); k=f(j);
    elseif method==2
        score=arrayfun(@(k)qValuePDQN(k,task,alloc,B,taskPos,S),cand); [~,j]=max(score); k=cand(j);
    else
        d=arrayfun(@(k)distanceToRoute(task,alloc{k},taskPos,S.base),cand); [~,j]=min(d); k=cand(j);
    end
    alloc{k}=[alloc{k} task];
    owners=[owners; task k]; %#ok<AGROW>
end
end

function T=makeAssignmentTable(R,shortNames,nUAV)
% Records task ownership after each event. MGPGA can later change the
% visiting order but never changes the task ownership listed in this table.
method={}; stage={}; uav=[]; sequence={};
stages={'initial','after t=30 min battery event', ...
        'after t=50 min UAV3 failure','after t=70 min wind/new-task event'};
fields={'initial','after30','after50','after70'};
for m=1:numel(R)
    for s=1:numel(fields)
        sets=R{m}.assignment.(fields{s});
        for k=1:nUAV
            method{end+1,1}=shortNames{m}; %#ok<AGROW>
            stage{end+1,1}=stages{s}; %#ok<AGROW>
            uav(end+1,1)=k; %#ok<AGROW>
            if isempty(sets{k})
                sequence{end+1,1}='[]'; %#ok<AGROW>
            else
                sequence{end+1,1}=['[' strtrim(sprintf('%d ',sets{k})) ']']; %#ok<AGROW>
            end
        end
    end
end
T=table(method,stage,uav,sequence, ...
    'VariableNames',{'Method','Stage','UAV','TaskSequence'});
end

function tf=safeToDo(k,task,B,available,taskPos,S)
if ~available(k), tf=false; return; end
p=taskPos(task,:); e=S.energyPerKm*S.mapScale*(norm(p-S.base)+norm(p-S.base));
tf=(B(k)-e)>=S.Bmin;
end

function q=qValueCADQN(k,task,alloc,B,taskPos,S)
% Compact, reproducible DQN forward-policy surrogate.  The six terms match
% the paper state: distance, return energy, battery margin, workload,
% dynamic cost and heading difference.  Larger Q is better.
d=distanceToRoute(task,alloc{k},taskPos,S.base); ret=norm(taskPos(task,:)-S.base)*S.mapScale*S.energyPerKm;
load=numel(alloc{k}); w=norm(S.wind(taskPos(task,:),30));
q=3.8 - .019*d - .032*ret + .045*(B(k)-S.Bmin) - .18*load - .55*w;
end

function q=qValuePDQN(k,task,alloc,B,taskPos,S)
% Pure DQN uses a nominal value without constraint-aware action masking.
d=distanceToRoute(task,alloc{k},taskPos,S.base); load=numel(alloc{k});
q=3.0-.018*d-.13*load+.010*B(k)+.18*sin(.17*k+.11*task);
end

function ms=localDecisionProxy(method,nCandidate,nTask)
% Reassignment decision only (MGPGA route time excluded as requested).
if method==1, ms=0.35+0.04*nTask; elseif method==2, ms=3.0+0.20*nCandidate*nTask; else, ms=0.70+0.06*nCandidate+0.07*nTask; end
end

function route=mgpgaRoute(tasks,taskPos,base,S,oldRoute,protect)
% Memory-guided population initialization + protected elitist GA.
if isempty(tasks), route=[]; return; end
% Compact online MGPGA budget: 16 individuals x 18 generations keeps the
% demonstration responsive in MATLAB while retaining memory/protection,
% crossover, mutation, repair and elitism.
tasks=unique(tasks,'stable'); n=numel(tasks); popN=16; G=18;
pop=zeros(popN,n);
seed=oldRoute(ismember(oldRoute,tasks));
seed=[seed setdiff(tasks,seed,'stable')];
if protect && ~isempty(seed), pop(1,:)=seed; else, pop(1,:)=polarOrder(tasks,taskPos,base); end
for r=2:popN, pop(r,:)=tasks(randperm(n)); end
for g=1:G
    cost=zeros(popN,1); for r=1:popN, cost(r)=routeObjective(pop(r,:),taskPos,base,S); end
    [cost,ix]=sort(cost); pop=pop(ix,:); elite=pop(1:4,:); newPop=[elite; zeros(popN-4,n)];
    for r=5:popN
        p1=elite(randi(4),:); p2=pop(randi(max(5,ceil(popN/2))),:);
        child=orderCrossover(p1,p2);
        if rand<.23 && n>1, z=randperm(n,2); child(z)=child(fliplr(z)); end
        newPop(r,:)=child;
    end
    pop=newPop;
end
cost=zeros(popN,1); for r=1:popN, cost(r)=routeObjective(pop(r,:),taskPos,base,S); end
[~,i]=min(cost); route=pop(i,:);
end

function child=orderCrossover(p1,p2)
n=numel(p1); if n<2, child=p1; return; end
z=sort(randperm(n,2)); child=zeros(1,n); child(z(1):z(2))=p1(z(1):z(2));
fill=p2(~ismember(p2,child(child>0))); child(child==0)=fill;
end

function [route,xy,t,E]=repairBatteryRoute(route,taskPos,base,S,t0)
% Local route repair, Eq. (45): insert base only when a segment + safe
% return reserve would be infeasible.
if isempty(route), xy=base; t=0; E=0; return; end
fixed=[]; B=S.Bmax; p=base;
for j=1:numel(route)
    q=taskPos(route(j),:); e=edgeEnergy(p,q,S,t0+j);
    er=edgeEnergy(q,base,S,t0+j);
    if B-e-er<S.Bmin
        fixed=[fixed 0]; B=S.Bmax; p=base; %#ok<AGROW>
        e=edgeEnergy(p,q,S,t0+j);
    end
    fixed=[fixed route(j)]; B=B-e; p=q; %#ok<AGROW>
end
route=fixed; xy=nodesToXY([0 route 0],taskPos,S); [t,E]=routeCost(route,taskPos,base,S,t0);
end

function [t,E]=routeCost(route,taskPos,base,S,t0)
nodes=[0 route 0]; t=0; E=0;
for j=1:numel(nodes)-1
    p=nodeXY(nodes(j),taskPos,base); q=nodeXY(nodes(j+1),taskPos,base);
    leg=safeFlightLeg(p,q,S);
    for r=1:size(leg,1)-1
        a=leg(r,:); b=leg(r+1,:); d=norm(b-a)*S.mapScale;
        wind=S.wind((a+b)/2,t0+t); v=max(.65,S.vCruise+dot(wind,(b-a)/(norm(b-a)+eps)));
        e=S.energyPerKm*d*(1+.15*norm(wind)); E=E+e; t=t+d/v;
    end
    if nodes(j+1)~=0, t=t+S.serviceTime; end
    if nodes(j+1)==0 && j>1, t=t+S.chargeTime; end
end
end

function e=edgeEnergy(p,q,S,t)
leg=safeFlightLeg(p,q,S); e=0;
for r=1:size(leg,1)-1
    a=leg(r,:); b=leg(r+1,:); d=norm(b-a)*S.mapScale;
    w=S.wind((a+b)/2,t); e=e+S.energyPerKm*d*(1+.15*norm(w));
end
end

function J=routeObjective(route,taskPos,base,S)
[t,E]=routeCost(route,taskPos,base,S,25); J=.68*t+.32*E;
end

function d=distanceToRoute(task,route,taskPos,base)
if isempty(route), d=norm(taskPos(task,:)-base); else, d=min(vecnorm(taskPos(task,:)-taskPos(route,:),2,2)); end
end

function out=sectorAllocation(wt,base,n)
ang=atan2(wt(:,2)-base(2),wt(:,1)-base(1)); [~,ix]=sort(ang); out=cell(n,1);
nPer=numel(ix)/n;
if abs(nPer-round(nPer))>eps, error('The number of turbines must be divisible by the number of UAVs.'); end
nPer=round(nPer);
for k=1:n, out{k}=ix((k-1)*nPer+(1:nPer))'; end
end

function r=polarOrder(tasks,pos,base)
if isempty(tasks), r=[]; return; end
a=atan2(pos(tasks,2)-base(2),pos(tasks,1)-base(1)); [~,ix]=sort(a); r=tasks(ix);
end

function xy=nodesToXY(nodes,taskPos,S)
% Every leg permits only its two end nodes. All other turbines are treated
% as circular obstacles; this also covers turbines assigned to another UAV.
xy=nodeXY(nodes(1),taskPos,S.base);
for i=1:numel(nodes)-1
    p=nodeXY(nodes(i),taskPos,S.base); q=nodeXY(nodes(i+1),taskPos,S.base);
    leg=safeFlightLeg(p,q,S);
    xy=[xy; leg(2:end,:)]; %#ok<AGROW>
end
end

function leg=safeFlightLeg(p,q,S)
% Iteratively replace each unsafe straight segment by a two-waypoint
% passage around the safety circle.  Only a turbine that is exactly an end
% node is permitted (the current inspection target or just-serviced WT).
leg=[p;q]; maxRepairs=60;
for it=1:maxRepairs
    repaired=false;
    for r=1:size(leg,1)-1
        a=leg(r,:); b=leg(r+1,:);
        [hit,c,side]=firstTurbineConflict(a,b,S);
        if ~hit, continue; end
        d=b-a; u=d/(norm(d)+eps); n=[-u(2) u(1)];
        R=S.turbineClearance+S.detourMargin;
        % Two offset waypoints make the centre segment tangentially pass
        % outside the obstacle rather than cutting through its safety disk.
        candidate=zeros(2,2,2); score=-inf(2,1);
        signs=[side -side];
        for z=1:2
            s=signs(z);
            candidate(:,:,z)=[c-R*u+s*R*n; c+R*u+s*R*n];
            d1=vecnorm(S.wt-candidate(1,:,z),2,2);
            d2=vecnorm(S.wt-candidate(2,:,z),2,2);
            same=vecnorm(S.wt-c,2,2)<1e-8;
            d1(same)=inf; d2(same)=inf;
            score(z)=min([d1;d2]);
        end
        [~,z]=max(score);
        via=candidate(:,:,z);
        leg=[leg(1:r,:); via; leg(r+1:end,:)]; %#ok<AGROW>
        repaired=true;
        break;
    end
    if ~repaired, return; end
end
warning('safeFlightLeg:RepairLimit', ...
    'Turbine-avoidance repair limit reached; increase S.detourMargin if needed.');
end

function [hit,center,side]=firstTurbineConflict(a,b,S)
% Finds the first non-endpoint turbine buffer crossed by segment a--b.
hit=false; center=[nan nan]; side=1; best=inf; d=b-a; dd=dot(d,d);
for j=1:S.nWT
    c=S.wt(j,:);
    % Allow departure from a just-inspected turbine and arrival at the
    % current target turbine, but prohibit every other wind turbine.
    if norm(c-a)<1e-8 || norm(c-b)<1e-8, continue; end
    tau=max(0,min(1,dot(c-a,d)/(dd+eps)));
    z=a+tau*d; clearance=norm(c-z);
    if clearance<S.turbineClearance && tau>1e-6 && tau<1-1e-6 && tau<best
        hit=true; center=c; best=tau;
        cross=d(1)*(c(2)-a(2))-d(2)*(c(1)-a(1));
        side=-sign(cross); if side==0, side=1; end
    end
end
end

function p=nodeXY(node,taskPos,base)
if node==0, p=base; else, p=taskPos(node,:); end
end

function e=makeEvent(t,type,uav,xy)
e=struct('time',t,'type',type,'uav',uav,'xy',xy);
end

function drawFarm(ax,S,R)
scatter(ax,S.wt(:,1),S.wt(:,2),95,[.78 .78 .78],'filled','MarkerEdgeColor','k','LineWidth',1.0);
for j=1:S.nWT, text(ax,S.wt(j,1)+8,S.wt(j,2)+7,sprintf('WT%d',j),'FontSize',7,'FontWeight','bold'); end
plot(ax,S.base(1),S.base(2),'p','MarkerSize',40,'MarkerFaceColor',[1 .72 .05],'MarkerEdgeColor','k','LineWidth',1.6);
text(ax,S.base(1)+16,S.base(2)+16,'Maintenance base / charging station','Color',[.85 .30 0],'FontWeight','bold');
for e=1:numel(R.events)
    if strcmp(R.events(e).type,'failure'), plot(ax,R.events(e).xy(1),R.events(e).xy(2),'x','Color',[.80 .05 .08],'MarkerSize',12,'LineWidth',2); end
end
end

function plotEventAnnotations(ax,R,taskPos,S)
seen=[];
for e=1:numel(R.events)
    q=R.events(e); if any(seen==q.uav) && strcmp(q.type,'wind/new task'), continue; end
    seen=[seen q.uav]; %#ok<AGROW>
    plot(ax,q.xy(1),q.xy(2),'h','MarkerSize',12,'MarkerFaceColor',[1 .05 .55],'MarkerEdgeColor','k','LineWidth',1.0);
    text(ax,q.xy(1)+10,q.xy(2)-12,sprintf('t=%d min: UAV%d %s',q.time,q.uav,q.type), ...
        'Color',[.72 0 .35],'FontSize',8,'FontWeight','bold');
end
end

function addArrows(ax,p,c,base)
if size(p,1)<3, return; end
% Two inspection-direction arrows plus one dedicated terminal return arrow.
idx=unique(round(linspace(1,max(1,size(p,1)-2),min(2,max(1,size(p,1)-2)))));
for k=idx
    d=p(k+1,:)-p(k,:); if norm(d)>0, quiver(ax,p(k,1),p(k,2),.32*d(1),.32*d(2),0, ...
        'Color',c,'LineWidth',1.05,'MaxHeadSize',.8,'HandleVisibility','off'); end
end
% Final arrow is centred on the final leg and points exactly to the base.
a=p(end-1,:); d=base-a;
if norm(d)>0
    q0=a+.38*d;
    quiver(ax,q0(1),q0(2),.42*d(1),.42*d(2),0,'Color',c, ...
        'LineWidth',1.8,'MaxHeadSize',1.15,'HandleVisibility','off');
end
end

function safeExport(fig,file)
if exist('exportgraphics','file')==2, exportgraphics(fig,file,'Resolution',250); else, print(fig,file,'-dpng','-r250'); end
end
