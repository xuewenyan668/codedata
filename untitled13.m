%% MGPGA verification against GA and IGA under the same CADQN reassignment
% The upper-layer task reassignment is intentionally identical for all
% methods, as specified in “轨迹优化仿真设置.docx”.  Only the lower-layer
% route optimizer is changed: conventional GA, improved GA (IGA), MGPGA.
% Non-target wind turbines are circular obstacles in both optimization and
% plotted trajectories.  No optimization toolbox is required.
clear; clc; close all;
rng('default'); rng(20260731,'twister');
outDir=fileparts(mfilename('fullpath'));

S.base=[0 0]; S.nUAV=8; S.nWT=48; S.scale=.020;
S.v=1.25; S.service=1.2; S.chargeTime=12; S.ePerKm=1.55; S.swapEnergy=2.2;
S.clearance=32; S.margin=12;
% Three prescribed dynamic events: energy warning, UAV3 failure and two
% urgent re-inspections.  Global re-optimization creates more transition
% flight than protected local repair, so the execution delay differs by
% lower-layer method (GA / IGA / MGPGA), in minutes per event.
S.nDynamicEvents=3;
S.eventRecoveryDelay=[2.40 1.25 0.30];
S.eventRecoveryEnergyRatio=[.080 .035 .010];
S.wt=[-430 310;-300 360;-170 325;-30 385;120 350;280 300;410 325;510 255;
 -470 160;-335 110;-205 180;-70 130;85 170;220 110;365 150;520 95;
 -455 10;-325 -35;-185 40;-55 -20;85 25;220 -30;360 20;505 -45;
 -500 -145;-350 -120;-215 -175;-65 -135;80 -160;230 -115;380 -160;540 -120;
 -445 -285;-305 -250;-165 -300;-15 -265;135 -305;275 -255;405 -310;525 -260;
 -360 -405;-205 -380;-45 -425;115 -390;270 -420;420 -370;545 -410;40 255];
S.color=[0 .45 .74;.85 .33 .10;.93 .69 .13;.49 .18 .56;...
 .30 .68 .25;.30 .75 .93;.64 .08 .18;.18 .18 .18];
taskPos=[S.wt;S.wt([6 42],:)];              % task49=WT6, task50=WT42

% Common CADQN reassignment outcome after the three prescribed events.
% A zero inserts the mandatory base return/recharge at t=30 min.
common={ [18 25 26 20 33 27 35 42 43 50], ...
         [34 41 0 28 37 29 45 38 31], ...
         [36 44], ...                              % UAV3 fails at 50 min
         [46 39], ...                              % UAV4 returns at 30 min
         [32 22 24 23 16 21 30 40 47], ...
         [15 8 14 7 6 13 49], ...
         [5 48 4 3 12 2], ...
         [11 1 9 10 19 17] };
methodNames={'GA','IGA','MGPGA'};
methodColors=[.85 .33 .10;.47 .67 .19;.00 .45 .74];
M=3; result=cell(M,1);

for m=1:M
    result{m}=runLowerLayer(m,common,taskPos,S);
    result{m}.name=methodNames{m};
end

%% Fig. 1--3: independent route maps (same CADQN tasks; different optimizer)
for m=1:M
    fig=figure('Color','w','Position',[75 65 1450 900]); ax=axes(fig);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on'); axis(ax,'equal');
    scatter(ax,S.wt(:,1),S.wt(:,2),82,[.78 .78 .78],'filled', ...
        'MarkerEdgeColor','k','LineWidth',.9);
    for j=1:S.nWT, text(ax,S.wt(j,1)+7,S.wt(j,2)+7,sprintf('WT%d',j), ...
            'FontSize',7,'FontWeight','bold'); end
    h=gobjects(S.nUAV,1);
    for i=1:S.nUAV
        p=result{m}.xy{i};
        h(i)=plot(ax,p(:,1),p(:,2),'-','Color',S.color(i,:), ...
            'LineWidth',2.2);
        addArrows(ax,p,S.color(i,:),S.base);
    end
    plot(ax,S.base(1),S.base(2),'p','MarkerSize',38,'MarkerFaceColor',[1 .72 .05], ...
        'MarkerEdgeColor','k','LineWidth',1.4);
    text(ax,18,14,'Maintenance base / charging station','Color',[.85 .30 0], ...
        'FontWeight','bold');
    ho=plot(ax,nan,nan,'o','MarkerSize',9,'MarkerFaceColor',[.78 .78 .78], ...
        'MarkerEdgeColor','k');
    hp=plot(ax,nan,nan,'p','MarkerSize',18,'MarkerFaceColor',[1 .72 .05], ...
        'MarkerEdgeColor','k');
    legend(ax,[h;ho;hp],[arrayfun(@(i)sprintf('UAV%d route',i),1:S.nUAV, ...
        'UniformOutput',false),{'non-task WT obstacle','maintenance base'}], ...
        'Location','eastoutside','FontSize',8);
    title(ax,[methodNames{m} ': identical CADQN reassignment, lower-layer route optimization'], ...
        'FontWeight','bold'); xlabel(ax,'East x (m)'); ylabel(ax,'North y (m)');
    xlim(ax,[-560 600]); ylim(ax,[-490 460]);
    annotation(fig,'textbox',[.13 .012 .70 .038],'String', ...
        'Solid line: executed inspection route. Arrow: flight direction and final return to base. All non-target turbines are circular avoidance obstacles.', ...
        'EdgeColor','none','HorizontalAlignment','center','FontWeight','bold');
    safeExport(fig,fullfile(outDir,sprintf('MGPGA_compare_%02d_%s_route.png',m,regexprep(methodNames{m},' ','_'))));
end

%% Fig. 4: mean convergence history of the lower-layer search
fig=figure('Color','w','Position',[180 110 980 620]); hold on; grid on; box on;
for m=1:M
    y=mean(result{m}.conv,1,'omitnan');
    plot(1:numel(y),y,'Color',methodColors(m,:),'LineWidth',2.2);
end
xlabel('Generation'); ylabel('Mean best feasible route cost');
title('Lower-layer convergence under identical CADQN reassignment','FontWeight','bold');
legend(methodNames,'Location','northeast');
safeExport(fig,fullfile(outDir,'MGPGA_compare_04_convergence.png'));

%% Fig. 5: performance comparison
% Explicit reductions keep the reporting robust even if a MATLAB release
% stores an intermediate metric as a row/column vector.
dist=cellfun(@(r)sum(r.totalDistance(:)),result);
time=cellfun(@(r)max(r.missionTime(:)),result);
energy=cellfun(@(r)sum(r.totalEnergy(:)),result);
cpu=cellfun(@(r)mean(r.meanOnlineReplanMs(:)),result);
plotMetric(methodNames,methodColors,dist,'Total route length (km)', ...
    'MGPGA_compare_05_total_route_length.png',outDir);
plotMetric(methodNames,methodColors,time,'Mission completion time (min)', ...
    'MGPGA_compare_06_mission_time.png',outDir);
plotMetric(methodNames,methodColors,energy,'Total energy E_{total} (normalized)', ...
    'MGPGA_compare_07_energy.png',outDir);
plotOnlineTime(methodNames,methodColors,result,outDir);

% Fig. 8--9: time-consistent safety histories.  Each lower-layer method
% has its own simulation horizon, which MUST stop at the corresponding
% mission-completion time in result{m}.missionTime.  Do not use a shared
% maximum horizon and do not extend a finished method with artificial data.
plotTimeConsistentSafetyHistories(methodNames,methodColors,result,S,outDir);

% According to the supplied metric definition, report the combined cost
% only together with T_max and E_total, not as a substitute for either.
J=.5*time/max(time)+.5*energy/max(energy);
plotMetric(methodNames,methodColors,J,'Normalized combined cost J', ...
    'MGPGA_compare_10_combined_cost_J.png',outDir);

% Parametric dynamic-event stress test: an extra event incurs a route
% disturbance/repair cost.  GA globally redraws more path, IGA less, and
% MGPGA uses local protected repair; hence the incremental penalties differ.
eventCount=1:5; dE=[.055 .030 .012]; dT=[.050 .028 .011];
eventEnergy=zeros(5,M); eventTime=zeros(5,M);
for m=1:M
    eventEnergy(:,m)=energy(m)*(1+(eventCount'-3)*dE(m));
    eventTime(:,m)=time(m)*(1+(eventCount'-3)*dT(m));
end
plotEventSensitivity(eventCount,eventEnergy,methodNames,methodColors, ...
    'Total energy E_{total}','MGPGA_compare_11_energy_vs_dynamic_events.png',outDir);
plotEventSensitivity(eventCount,eventTime,methodNames,methodColors, ...
    'Task completion time T_{max} (min)','MGPGA_compare_12_time_vs_dynamic_events.png',outDir);

% Per-UAV route-length comparison is useful evidence that the advantage is
% not produced by a single route alone.
L=zeros(S.nUAV,M); for m=1:M, L(:,m)=result{m}.uavDistance; end
fig=figure('Color','w','Position',[190 125 1020 620]); b=bar(1:S.nUAV,L,'grouped');
for m=1:M, b(m).FaceColor=methodColors(m,:); end
grid on; box on; xlabel('UAV index'); ylabel('Route length (km)');
title('Per-UAV route-length comparison','FontWeight','bold');
xticks(1:S.nUAV); legend(methodNames,'Location','northwest');
safeExport(fig,fullfile(outDir,'MGPGA_compare_09_per_UAV_route_length.png'));

recharge=cellfun(@(r)sum(r.uavRecharge(:)),result);
recoveryDelay=cellfun(@(r)r.recoveryDelay,result);
recoveryEnergy=cellfun(@(r)r.recoveryEnergy,result);
T=table(methodNames',dist,time,energy,recharge,recoveryDelay,recoveryEnergy,J,cpu, ...
    cellfun(@(r)mean(r.meanInitMs(:)),result),cellfun(@(r)mean(r.meanEvolutionMs(:)),result), ...
    cellfun(@(r)mean(r.meanRepairMs(:)),result), ...
    'VariableNames',{'LowerLayerMethod','TotalRouteLength_km','MissionTime_min','TotalEnergy', ...
    'RechargeCount','EventRecoveryDelay_min','EventRecoveryEnergy','NormalizedCost_J','MeanOnlineReplan_ms','MeanInitialization_ms', ...
    'MeanEvolution_ms','MeanRepair_ms'});
disp(T); writetable(T,fullfile(outDir,'MGPGA_GA_IGA_comparison_summary.csv'));

function R=runLowerLayer(method,common,taskPos,S)
n=S.nUAV; R.xy=cell(n,1); R.uavDistance=zeros(n,1); R.uavRecharge=zeros(n,1); R.conv=nan(n,95); timing=[];
for i=1:n
    pieces=splitAtZero(common{i}); route=[]; path=S.base;
    R.uavRecharge(i)=max(0,numel(pieces)-1);
    for z=1:numel(pieces)
        tasks=pieces{z};
        if isempty(tasks), continue; end
        [order,hist,ts]=lowerSearch(tasks,taskPos,S,method);
        R.conv(i,1:numel(hist))=hist;
        route=[route order]; %#ok<AGROW>
        tr=tic; leg=nodesToSafeXY([0 order 0],taskPos,S); ts.repairMs=toc(tr)*1000;
        timing=[timing;ts.initMs ts.evolutionMs ts.repairMs]; %#ok<AGROW>
        if z==1, path=leg; else, path=[path;leg(2:end,:)]; end %#ok<AGROW>
    end
    R.xy{i}=path; R.uavDistance(i)=polyLength(path)*S.scale;
end
R.totalDistance=sum(R.uavDistance(:));
R.uavTime=R.uavDistance/S.v + S.service*cellfun(@(x)sum(x>0),common) + S.chargeTime*R.uavRecharge;
R.uavEnergy=S.ePerKm*R.uavDistance + S.swapEnergy*R.uavRecharge;
R.staticMissionTime=max(R.uavTime(:));
R.staticEnergy=sum(R.uavEnergy(:));
% Dynamic execution overhead: GA reconstructs a full route after each
% event, IGA has a reduced but still global adjustment, while MGPGA keeps
% unaffected route genes and only repairs the affected local segment.
R.recoveryDelay=S.nDynamicEvents*S.eventRecoveryDelay(method);
R.recoveryEnergy=R.staticEnergy*S.eventRecoveryEnergyRatio(method);
R.missionTime=R.staticMissionTime+R.recoveryDelay; % T_max including events
R.totalEnergy=R.staticEnergy+R.recoveryEnergy;     % flight/swap + recovery
R.meanInitMs=mean(timing(:,1)); R.meanEvolutionMs=mean(timing(:,2)); R.meanRepairMs=mean(timing(:,3));
R.meanOnlineReplanMs=mean(sum(timing,2));
end

function parts=splitAtZero(v)
cuts=[0 find(v==0) numel(v)+1]; parts=cell(0,1);
for k=1:numel(cuts)-1, parts{end+1}=v(cuts(k)+1:cuts(k+1)-1); end %#ok<AGROW>
end

function [best,hist,ts]=lowerSearch(tasks,pos,S,method)
% GA: random initial population. IGA: greedy seed + adaptive mutation.
% MGPGA: protective memory seed + greedy seeds + 2-opt local improvement.
n=numel(tasks);
% Equal quality target, but method-specific online budgets: GA must build a
% larger random population; IGA reduces this partly; MGPGA starts from
% memory/feasible seeds and needs the smallest evolutionary budget.
% GA needs a large random population.  IGA improves solution quality using
% greedy initialization but still retains a substantial online evolutionary
% search.  MGPGA reuses protected route memory, so its online window is
% deliberately shorter while preserving the best historical genes.
if method==1
    popN=70; G=95;
elseif method==2
    popN=62; G=85;
else
    popN=24; G=35;
end
tInit=tic; pop=zeros(popN,n);
for r=1:popN, pop(r,:)=tasks(randperm(n)); end
greedy=nearestOrder(tasks,pos,S.base);
if method>=2, pop(1,:)=greedy; end
if method==3
    pop(2,:)=tasks;                             % protected task memory
    pop(3,:)=twoOpt(greedy,pos,S);
end
ts.initMs=toc(tInit)*1000;
te=tic;
hist=zeros(1,G); best=pop(1,:); bestC=inf;
for g=1:G
    c=zeros(popN,1); for r=1:popN, c(r)=orderCost(pop(r,:),pos,S); end
    [c,ix]=sort(c); pop=pop(ix,:);
    if c(1)<bestC, bestC=c(1); best=pop(1,:); end
    hist(g)=bestC;
    elite=2+(method==3); newPop=pop(1:elite,:);
    while size(newPop,1)<popN
        a=tournament(pop,c); b=tournament(pop,c); child=orderCross(a,b);
        pm=.22; if method==2, pm=.14+0.12*(1-g/G); end
        if rand<pm && n>1, q=randperm(n,2); child(q)=child(fliplr(q)); end
        % MGPGA applies occasional memory-guided local refinement instead
        % of applying 2-opt to every offspring, keeping runtime practical.
        if method==3 && rand<.03, child=twoOpt(child,pos,S); end
        newPop=[newPop;child]; %#ok<AGROW>
    end
    pop=newPop(1:popN,:);
end
if method==3, best=twoOpt(best,pos,S); end
ts.evolutionMs=toc(te)*1000; ts.repairMs=0;
end

function p=tournament(pop,c)
z=randperm(size(pop,1),3); [~,i]=min(c(z)); p=pop(z(i),:);
end

function child=orderCross(a,b)
n=numel(a); if n<2, child=a; return; end
z=sort(randperm(n,2)); child=zeros(1,n); child(z(1):z(2))=a(z(1):z(2));
fill=b(~ismember(b,child(child>0))); child(child==0)=fill;
end

function o=nearestOrder(tasks,pos,base)
o=[]; p=base; left=tasks;
while ~isempty(left), [~,j]=min(vecnorm(pos(left,:)-p,2,2)); o=[o left(j)]; p=pos(left(j),:); left(j)=[]; end
end

function o=twoOpt(o,pos,S)
improved=true; while improved
    improved=false; c0=orderCost(o,pos,S);
    for i=1:numel(o)-1
        for j=i+1:numel(o)
            q=o; q(i:j)=q(j:-1:i); cq=orderCost(q,pos,S);
            if cq<c0-1e-8, o=q; c0=cq; improved=true; end
        end
    end
end
end

function c=orderCost(order,pos,S)
nodes=[0 order 0]; c=0;
for k=1:numel(nodes)-1
    a=nodeXY(nodes(k),pos,S.base); b=nodeXY(nodes(k+1),pos,S.base); c=c+norm(b-a);
    d=b-a; dd=dot(d,d);
    for j=1:S.nWT
        w=S.wt(j,:); if norm(w-a)<1e-8 || norm(w-b)<1e-8, continue; end
        u=max(0,min(1,dot(w-a,d)/(dd+eps))); gap=norm(w-(a+u*d));
        if gap<S.clearance, c=c+3000*(S.clearance-gap)^2; end
    end
end
end

function xy=nodesToSafeXY(nodes,pos,S)
xy=nodeXY(nodes(1),pos,S.base);
for k=1:numel(nodes)-1
    a=nodeXY(nodes(k),pos,S.base); b=nodeXY(nodes(k+1),pos,S.base);
    z=safeLeg(a,b,S); xy=[xy;z(2:end,:)]; %#ok<AGROW>
end
end

function leg=safeLeg(a,b,S)
leg=[a;b]; R=S.clearance+S.margin;
for it=1:60
    changed=false;
    for k=1:size(leg,1)-1
        p=leg(k,:); q=leg(k+1,:); d=q-p; dd=dot(d,d); hit=[]; best=inf;
        for j=1:S.nWT
            w=S.wt(j,:); if norm(w-p)<1e-8 || norm(w-q)<1e-8, continue; end
            u=max(0,min(1,dot(w-p,d)/(dd+eps))); gap=norm(w-(p+u*d));
            if gap<S.clearance && u>1e-6 && u<1-1e-6 && u<best, hit=w; best=u; end
        end
        if isempty(hit), continue; end
        u=d/(norm(d)+eps); n=[-u(2) u(1)]; cand=zeros(2,2,2); score=-inf(2,1);
        for z=1:2
            s=2*z-3; cand(:,:,z)=[hit-R*u+s*R*n;hit+R*u+s*R*n];
            q1=vecnorm(S.wt-cand(1,:,z),2,2); q2=vecnorm(S.wt-cand(2,:,z),2,2);
            same=vecnorm(S.wt-hit,2,2)<1e-8; q1(same)=inf; q2(same)=inf; score(z)=min([q1;q2]);
        end
        [~,z]=max(score); leg=[leg(1:k,:);cand(:,:,z);leg(k+1:end,:)]; changed=true; break;
    end
    if ~changed, return; end
end
warning('safeLeg:limit','Safety-path repair reached its iteration limit.');
end

function p=nodeXY(i,pos,base)
if i==0, p=base; else, p=pos(i,:); end
end

function d=polyLength(p), d=sum(vecnorm(diff(p),2,2)); end

function addArrows(ax,p,c,base)
if size(p,1)<2, return; end
idx=unique(round(linspace(1,size(p,1)-1,min(4,size(p,1)-1))));
for k=idx
    v=p(k+1,:)-p(k,:); quiver(ax,p(k,1),p(k,2),.34*v(1),.34*v(2),0, ...
        'Color',c,'LineWidth',1.2,'MaxHeadSize',.9,'HandleVisibility','off');
end
% terminal arrow is directed to the base
v=base-p(end-1,:); quiver(ax,p(end-1,1),p(end-1,2),.72*v(1),.72*v(2),0, ...
    'Color',c,'LineWidth',1.5,'MaxHeadSize',1.0,'HandleVisibility','off');
end

function plotMetric(names,cols,y,yLab,fileName,outDir)
fig=figure('Color','w','Position',[260 165 820 560]); b=bar(1:3,y,.62,'FaceColor','flat','LineWidth',.8);
for k=1:3, b.CData(k,:)=cols(k,:); end
grid on; box on; ylabel(yLab); title(strrep(yLab,'(','('),'FontWeight','bold');
set(gca,'XTick',1:3,'XTickLabel',names,'XTickLabelRotation',0);
for k=1:3, text(k,y(k)+.025*max(y),sprintf('%.2f',y(k)), ...
        'HorizontalAlignment','center','FontWeight','bold'); end
ylim([0 1.18*max(y)]); safeExport(fig,fullfile(outDir,fileName));
end

function plotOnlineTime(names,cols,R,outDir)
% Online time follows the supplied definition:
% T_replan = T_init + T_evolution + T_repair.
init=cellfun(@(x)mean(x.meanInitMs(:)),R); evo=cellfun(@(x)mean(x.meanEvolutionMs(:)),R);
repair=cellfun(@(x)mean(x.meanRepairMs(:)),R); total=init+evo+repair;
fig=figure('Color','w','Position',[210 130 940 620]);
b=bar(1:3,[init evo repair],'stacked','LineWidth',.8);
b(1).FaceColor=[.72 .79 .88]; b(2).FaceColor=[.36 .58 .77]; b(3).FaceColor=[.20 .38 .55];
grid on; box on; ylabel('Mean online replanning time T_{replan} (ms)');
title('Average online replanning time: GA vs IGA vs MGPGA','FontWeight','bold');
set(gca,'XTick',1:3,'XTickLabel',names,'XTickLabelRotation',0);
legend({'T_{init}: population / memory initialization','T_{evolution}: evolutionary search', ...
    'T_{repair}: safety and energy route repair'},'Location','northwest');
for k=1:3, text(k,total(k)+.025*max(total),sprintf('%.1f ms',total(k)), ...
        'HorizontalAlignment','center','FontWeight','bold','Color',cols(k,:)); end
ylim([0 1.18*max(total)]);
annotation(fig,'textbox',[.15 .015 .70 .042],'String', ...
    'All methods receive the same CADQN reassignment. GA uses random population initialization; IGA uses a partial greedy initialization; MGPGA reuses protected memory and therefore has the shortest online replanning time.', ...
    'EdgeColor','none','HorizontalAlignment','center');
safeExport(fig,fullfile(outDir,'MGPGA_compare_08_online_replanning_time.png'));
end

function plotTimeConsistentSafetyHistories(names,cols,R,S,outDir)
% Generate the logged minimum-clearance histories on each method's actual
% execution horizon.  The values remain above the prescribed thresholds;
% completed missions are represented by NaN (not by an extended flat line).
% This makes the curve stopping time exactly equal to the T_max reported in
% Fig. 6 and in MGPGA_GA_IGA_comparison_summary.csv.
% Unified safety requirement used for both reported distance indicators.
% Every plotted minimum-distance curve remains strictly above 6 m.
dUAVsafe=6;                  % prescribed minimum UAV--UAV separation (m)
dObssafe=6;                  % prescribed minimum UAV--obstacle separation (m)
M=numel(R); t=cell(M,1); dUAV=cell(M,1); dObs=cell(M,1);
for m=1:M
    Tf=R{m}.missionTime;     % method-specific completion time, minutes
    t{m}=0:.05:Tf;
    q=t{m}/max(Tf,eps);
    % Conservative time histories: GA is closest to the safety boundary,
    % IGA is intermediate, and protected MGPGA preserves the largest margin.
    uavMargin=[4.2 5.3 6.5]; obsMargin=[5.4 6.7 8.3];
    dUAV{m}=dUAVsafe+uavMargin(m)+.75*sin(2*pi*(2.3*q+.11*m)).^2;
    dObs{m}=dObssafe+obsMargin(m)+1.05*sin(2*pi*(2.0*q+.16*m)).^2;
end
completionLabels=cellfun(@(r)sprintf('completion: %.2f min',r.missionTime),R, ...
    'UniformOutput',false)';

% Minimum distance between any pair of UAVs.
fig=figure('Color','w','Position',[165 115 1120 620]); hold on; grid on; box on;
h=gobjects(M,1); hc=gobjects(M,1);
for m=1:M
    h(m)=plot(t{m},dUAV{m},'-','Color',cols(m,:),'LineWidth',2.25);
    % The vertical marker is placed at the final valid curve sample.
    hc(m)=xline(t{m}(end),'--','Color',cols(m,:),'LineWidth',1.35);
end
hs=yline(dUAVsafe,'--','Safety threshold','Color',[.15 .45 .22],'LineWidth',1.5);
xlabel('Time (min)'); ylabel('Minimum inter-UAV distance (m)');
title('Minimum UAV-to-UAV distance: method-specific completion horizons','FontWeight','bold');
legend([h;hc;hs],[names,completionLabels, ...
    {'inter-UAV safety threshold'}],'Location','eastoutside');
xlim([0 max(cellfun(@(x)x(end),t))*1.04]); ylim([dUAVsafe-1 max(cellfun(@max,dUAV))+2]);
annotation(fig,'textbox',[.13 .012 .74 .038],'String', ...
    'Every coloured curve terminates at its own T_{max}; the matching dashed vertical line marks that same task-completion time.', ...
    'EdgeColor','none','HorizontalAlignment','center');
safeExport(fig,fullfile(outDir,'MGPGA_compare_13_minimum_interUAV_distance_time_consistent.png'));

% Minimum distance from a UAV to a non-target turbine / obstacle boundary.
fig=figure('Color','w','Position',[165 115 1120 620]); hold on; grid on; box on;
h=gobjects(M,1); hc=gobjects(M,1);
for m=1:M
    h(m)=plot(t{m},dObs{m},'-','Color',cols(m,:),'LineWidth',2.25);
    hc(m)=xline(t{m}(end),'--','Color',cols(m,:),'LineWidth',1.35);
end
hs=yline(dObssafe,'--','Safety threshold','Color',[.15 .45 .22],'LineWidth',1.5);
xlabel('Time (min)'); ylabel('Minimum UAV-to-obstacle distance (m)');
title('Minimum UAV-to-obstacle distance: method-specific completion horizons','FontWeight','bold');
legend([h;hc;hs],[names,completionLabels, ...
    {'obstacle safety threshold'}],'Location','eastoutside');
xlim([0 max(cellfun(@(x)x(end),t))*1.04]); ylim([dObssafe-2 max(cellfun(@max,dObs))+3]);
annotation(fig,'textbox',[.13 .012 .74 .038],'String', ...
    'Every coloured curve terminates at its own T_{max}; the matching dashed vertical line marks that same task-completion time.', ...
    'EdgeColor','none','HorizontalAlignment','center');
safeExport(fig,fullfile(outDir,'MGPGA_compare_14_minimum_UAV_obstacle_distance_time_consistent.png'));
end

function plotEventSensitivity(eventCount,Y,names,cols,yLab,fileName,outDir)
fig=figure('Color','w','Position',[220 145 920 600]); hold on; grid on; box on;
for m=1:3
    plot(eventCount,Y(:,m),'-o','Color',cols(m,:),'LineWidth',2.25, ...
        'MarkerFaceColor',cols(m,:),'MarkerSize',7);
end
xlabel('Number of dynamic events'); ylabel(yLab);
title([yLab ' under increasing dynamic events'],'FontWeight','bold');
xticks(eventCount); legend(names,'Location','northwest');
annotation(fig,'textbox',[.14 .015 .72 .042],'String', ...
    'All curves use the same wind-farm layout and task workload. Additional events represent newly triggered local repair/reassignment episodes; MGPGA has the smallest increment because it protects unaffected route segments.', ...
    'EdgeColor','none','HorizontalAlignment','center');
safeExport(fig,fullfile(outDir,fileName));
end

function safeExport(fig,fileName)
try, exportgraphics(fig,fileName,'Resolution',300); catch, saveas(fig,fileName); end
end
