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
for ii = 1:2:8
    for jj = 1:2:8
        for kk = 1:2:8
            xyz = zeros(1000,33);
            l = 0;
            for i = ii:8:80
                for j = jj:8:80
                    for k = kk:8:80
                        a = NodlgStatics(S,zeros(S.ndof,1),[-i, -j, -k]);
                        centers = plotqGetPosition(S,a,'FaceAlpha',0.1);
                        l = l+1;
                        xyz(l,:) = [i,j,k,centers*100];
                    end
                end
            end
            writematrix(xyz, ['data',num2str(ii),num2str(jj),num2str(kk),'.txt'])
        end
    end
end
%% Save

