%%
run ../startup.m
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
%% Load Configuration
load('FourdofActuator.mat')
% L = Link();
% S = Linkage(L);

%% Test & Plot
TendonLength = [20 0 20 0];    % 4dof
TendonLengthFull = zeros(1,8);  % expand to 8 dof
for i=1:length(TendonLength)
    TendonLengthFull(2*i-1) = TendonLength(i);
    TendonLengthFull(2*i)   = TendonLength(i);
end
a = NodlgStatics(S,zeros(S.ndof,1),-TendonLengthFull);

centers = plotqGetPosition(S,a,'FaceAlpha',0.1);
for i=1:length(centers)
    x = centers(i,1);
    y = centers(i,2);
    z = centers(i,3);
    scatter3(x,y,z,'MarkerEdgeColor','k','MarkerFaceColor','r')
    ls = [x,y,z];
    fprintf("%d,%d,%d \n",ls(1),ls(2),ls(3))
end

%% Plot for Testing
S.plotq0()

%% Initalize Logger
Interpolate = [];
Extrapolate = [];

Interpolatejsonname  = sprintf('Interpolate.json'); 
Extrapolatejsonname  = sprintf('Extrapolate.json'); 
Interpolatejson      = fopen(Interpolatejsonname,'w');
Extrapolatejson      = fopen(Extrapolatejsonname,'w');
%%

count = 0;
while 1
    % Get random TendonLength
    TendonLength = 10*rand(1,4);    % 4dof
    TendonLengthFull = zeros(1,8);  % expand to 8 dof
    for i=1:length(TendonLength)
        TendonLengthFull(2*i-1) = TendonLength(i);
        TendonLengthFull(2*i)   = TendonLength(i);
    end

    % Statics
    a = NodlgStatics(S,zeros(S.ndof,1),-TendonLengthFull); % Notice (-) means pulling
    
    % Get marker points
    centers = GetPosition(S,a,'FaceAlpha',0.1);
    
    % Save to matrix
    isInInterpolateReigion = TendonLength <= 10*ones(1,4);
    if all(isInInterpolateReigion)
        Interpolatejson      = fopen(Interpolatejsonname,'a+');
        fprintf(Interpolatejson, '\n');
        s = struct("actuation", TendonLength, "position", centers); 
%         encodedJSON = jsonencode(s);
%         fprintf(Interpolatejson, encodedJSON);
        count = count+1;
    end
    
    if count == 20000
        break
    end
end

%%
count = 0;
while 1
    % Get random TendonLength
    TendonLength = 15*rand(1,4);    % 4dof
    TendonLengthFull = zeros(1,8);  % expand to 8 dof
    for i=1:length(TendonLength)
        TendonLengthFull(2*i-1) = TendonLength(i);
        TendonLengthFull(2*i)   = TendonLength(i);
    end

    % Statics
    a = NodlgStatics(S,zeros(S.ndof,1),-TendonLengthFull); % Notice (-) means pulling
    
    % Get marker points
    centers = GetPosition(S,a,'FaceAlpha',0.1);
    
    % Save to matrix
    isInInterpolateReigion = TendonLength <= 10*ones(1,4);
    if ~all(isInInterpolateReigion)
        Extrapolatejson      = fopen(Extrapolatejsonname,'a+');
        fprintf(Extrapolatejson, '\n');
        s = struct("actuation", TendonLength, "position", centers); 
%         encodedJSON = jsonencode(s);
%         fprintf(Extrapolatejson, encodedJSON);
        count = count+1;
    end
    
    
    if count == 5000
        break
    end
end


%%

%% Save

