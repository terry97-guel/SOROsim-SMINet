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

%%
xyz = zeros(1000,33);
l = 0;
for i = 7:8:80
    for j = 1:8:80
        for k = 1:8:80
            a = NodlgStatics(S,zeros(S.ndof,1),[-i, -j, -k]);
            centers = plotqGetPosition(S,a,'FaceAlpha',0.1);
            l = l+1;
            xyz(l,:) = [i,j,k,centers*100];
        end
    end
end
%% Save

writematrix(xyz, 'data711.txt')
