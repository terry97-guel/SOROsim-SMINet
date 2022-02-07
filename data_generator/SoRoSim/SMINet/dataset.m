%%
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
%%
L = Link();
S = Linkage(L);

%%
centers = plotqGetPosition(S,a,'FaceAlpha',0.1);
for i=length(centers)
    x = centers(i,1);
    y = centers(i,2);
    z = centers(i,3);
    scatter3(x,y,z,'MarkerEdgeColor','k','MarkerFaceColor','r')
    ls = [x,y,z];
end

%%
a = NodlgStatics(S,zeros(S.ndof,1),[20,0,0]);
%%
S.plotq0()