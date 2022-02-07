%%
addpath ../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/

ccc

folder = 'data/';
urdf_path = strcat(folder,'test_urdf.xml');
% upload data

priAngle = dlmread(strcat(folder,'priAngle.txt'));
revAngle = dlmread(strcat(folder,'revAngle.txt'));
targetPose = dlmread(strcat(folder,'targetPose.txt'));
outputPose = dlmread(strcat(folder,'outputPose.txt')); 
branchLs = dlmread(strcat(folder,'branchLs.txt'));
IOScale= dlmread(strcat(folder,'IOScale.txt'));

n_joint = length(branchLs);
nData = length(priAngle);
inputSigma = IOScale(1,:);
inputMean = IOScale(2,:);
outputSigma = IOScale(3,:);
outputMean = IOScale(4,:);
robot = 'test';

x = [];
y = [];
z = [];
for i= (1:length(targetPose))
    coordinate = mod(i,3);
    if coordinate == 1
        x = [x;targetPose(:,i)];
    elseif coordinate == 2
        y = [y;targetPose(:,i)];
    elseif coordinate == 0
        z = [z;targetPose(:,i)];
    end
end

margin = 0.05 * [min(x)-max(x),max(x)-min(x),min(y)-max(y),max(y)-min(y),min(z)-max(z),max(z)-min(z)];
fig_size = [min(x),max(x),min(y),max(y),min(z),max(z)] + margin;
chain = get_chain_from_urdf(robot,'urdf_path',urdf_path);
schain = chain2schain(chain);


%%
% set figure
fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
    'view_info',[210,50],'axis_info',fig_size,'AXIS_EQUAL',0,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0,'DataAspectRatio',outputSigma/10);

targetpose = targetPose;
plot_traj(targetpose(:,1:3),'fig_idx',1,'subfig_idx',2,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,4:6),'fig_idx',1,'subfig_idx',3,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,7:9),'fig_idx',1,'subfig_idx',4,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,10:12),'fig_idx',1,'subfig_idx',5,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,13:15),'fig_idx',1,'subfig_idx',6,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,16:18),'fig_idx',1,'subfig_idx',7,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,19:21),'fig_idx',1,'subfig_idx',8,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,22:24),'fig_idx',1,'subfig_idx',9,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,25:27),'fig_idx',1,'subfig_idx',10,'tlc','b','tlw',1.5,'tls','-');
plot_traj(targetpose(:,28:30),'fig_idx',1,'subfig_idx',11,'tlc','b','tlw',1.5,'tls','-');

tick = 0; run_mode = 'STOP'; j=1; max_j = nData; %268 
tfc = 'k';
g_key = ''; chainplot = chain;
while 1
    g_key = get(gcf,'CurrentKey');
    
    if isequal(run_mode,'RUN')
        % Run something
        tick = tick +1;
        j = fix(tick/1);
        
        % get outputPose
        output_p = outputPose(1:j,:);
        pri_angle = priAngle(tick,:);
        rev_angle = revAngle(tick,:);
        
        dq = zeros(20,1);
        ddq = zeros(20,1);
        schain = update_schain_q_dq_ddq(schain,schain.rev_joint_names,rev_angle,dq,ddq,'IGNORE_LIMIT',1);
        schain = update_schain_q_dq_ddq(schain,schain.pri_joint_names,pri_angle,dq,ddq,'IGNORE_LIMIT',1);
        chainplot = schain2chain(schain);
        
        % plot trajectory of outputPose
        plot_traj(output_p(2:end,1:3),'fig_idx',1,'subfig_idx',21,'tlc','r','tlw',1,'tls','--'); 
        plot_traj(output_p(2:end,4:6),'fig_idx',1,'subfig_idx',22,'tlc','r','tlw',1,'tls','--'); 
        plot_traj(output_p(2:end,7:9),'fig_idx',1,'subfig_idx',23,'tlc','r','tlw',1,'tls','--'); 
        plot_traj(output_p(2:end,10:12),'fig_idx',1,'subfig_idx',24,'tlc','r','tlw',1,'tls','--'); 
        plot_traj(output_p(2:end,13:15),'fig_idx',1,'subfig_idx',25,'tlc','r','tlw',1,'tls','--');
        plot_traj(output_p(2:end,16:18),'fig_idx',1,'subfig_idx',26,'tlc','r','tlw',1,'tls','--');
        plot_traj(output_p(2:end,19:21),'fig_idx',1,'subfig_idx',27,'tlc','r','tlw',1,'tls','--');
        plot_traj(output_p(2:end,22:24),'fig_idx',1,'subfig_idx',28,'tlc','r','tlw',1,'tls','--');
        plot_traj(output_p(2:end,25:27),'fig_idx',1,'subfig_idx',29,'tlc','r','tlw',1,'tls','--');
        plot_traj(output_p(2:end,28:30),'fig_idx',1,'subfig_idx',30,'tlc','r','tlw',1,'tls','--');
        
        
    else
        pause(1e-1);
    end
    
    % Animate
    if mod(tick,1) == 0
        fig = plot_chain(chainplot,'fig_idx',1,'subfig_idx',41,'fig_pos',[0.5,0.35,0.5,0.6],...
            'view_info',[68,16],'axis_info',[-1.0,+1.0,-1.0,+1.0,0,+1.2],'USE_ZOOMRATE',1,...
            'PLOT_LINK',1,'llc','k','llw',1,...
            'PLOT_JOINT_AXIS',0,'jal',0.05,'jalw',2,'jals','-',...
            'PLOT_JOINT_NAME',0,'jnfs',9);
        
        plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',1,'subfig_idx',42,...
            'PLOT_AXIS',1,'all',0.5,'alw',3,'PLOT_SPHERE',0,...
            'text_str','World','text_fs',10,'text_interp','latex'); % world coordinate
        
        
        title_str = sprintf('[%s][%d] SMInet ([r]:run [s]:stop [q]:quit)',run_mode,tick);
        plot_title(title_str,'fig_idx',1,'tfc',tfc,'tfs',20);
        drawnow; if ~ishandle(fig), break; end
    end
    
    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'       % press 'q' to quit
                break;
            case 's'       % press 's' to stop
                run_mode = 'STOP';
                tfc      = 'k';
            case 'r'       % press 'r' to run
                run_mode = 'RUN';
                tfc      = 'b';
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    % Terminate condition
    if j >= max_j
        break;
    end
end
