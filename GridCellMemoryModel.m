function GridCellMemoryModel

    clear all

    showlearning=0;  % whether to show learning at each time step (simulation is very slow when turned on)
    MakeMovie=0;
    PlotPlaceCells=0;  % whether to show positions of place cell spikes
    ChangeBox=0;      % whether to change box after one set of recordings before a second set of recordings
    BoxShape=1;    % 1 is square box and 2 is round box 3 is radially arm maze (but radial arm maze should probably use different inputs) 4 is trapezoid
    HeightRatio=1;  % whether box is rectange or square (ratio of height to width); .25 used for global remapping simulation
    WallDist=1/2;          % distance to walls of box (dimensions run from -1 to 1)
    InnerWall=0;  % used for circular track
    ArmWidth=.02;  % used for radial arm maze

    % parameters

    LearnRate=1;            % learning rate
    ConsolidateThresh=.8;   % BCM threshold determines spacing of consolidation -- .75 for 3 layers and .8 for 6 layers -- .9 for 2D close grid
    ActThresh=.9;           % activation threhold determines rate of new memory formation and extent of surround for consolidation -- .86 for 3 layers and .9 for 6 layers -- .92 for 2D close grid
    Feedback=3;             % strength of feedback from winning memory onto boundary/HD cells
    silent=95;              % proportion of activation values that do not produce a spike -- 95 for 3D and 90 for 2D

    % population coding and box geometry

    Means=[-2/3:2/3:2/3];  % defines preferred dimension value for each of the 3 cells within a basis set

    NDim=3;                % set to 2 or 3 depending on whether the simulation includes head direction

    % animal behavior and experimental variables
   
    PreLearnSteps=100000;               % amount of prior experience with box before recording
    RecordSteps=10000;                  % duration of recording session
    st=.05;                             % step size determines sampling rate (.05 default)
    momentum=.7;                        % how much the anima continues to go in the same direction (.7 default)
    Steps=PreLearnSteps+(ChangeBox+1)*RecordSteps;    % total number of time steps
    FigureOffset=0;                     % used to have second recarding for ChangeBox
    StepOffset=PreLearnSteps;           % used to have second recarding for ChangeBox


    % initialize variables

    begin=1;        % determine begin and end indices for each dimension within the input/weight vector
    for i=1:NDim+1
        Nb(i,:)=[begin begin+2];
        begin=begin+3;
    end
    
    W=[];                                       % initialize weights
    PosStep=zeros(RecordSteps,NDim);               % position and head direction for each recording step
    ActStep=zeros(RecordSteps,Nb(NDim+1,2)+1);  % activation at each recording step
    MemActStep=zeros(1,RecordSteps);            % create array for recording place cell spikes
    
%     Nreps=100;
%     for PreTrain=1:11
%         GridScores=zeros(Nreps,5); % last one is number of place cells
%         PreLearnSteps=(PreTrain-1)*10000
%         Steps=PreLearnSteps+RecordSteps;
%        for Reps=1:Nreps

 %   for FigureOffset=0:10:190
            if MakeMovie==1
                vidObj = VideoWriter('Explore.avi');
                open(vidObj);
            end

            ExploreBox; % Call main routine

            if MakeMovie==1
                close(vidObj);
            end
        
            PlotResults % Call routine to plot results

            NumMemories=size(W,1)
%         end
%         FileName=sprintf("GridScores%d.csv",PreTrain-1)
%         csvwrite(FileName,GridScores);
%         GridScores
%    end

    function ExploreBox % main routine for navigation, learning, and recording

        goal=2*rand-1; % initial goal direction (polar angle)
        hd=2*rand-1;   % initial head direction (polar angle)
        wall=0;
        if BoxShape==1 % square box
            Xpos=(2*rand-1).*WallDist; % random starting point
            Ypos=(2*rand-1).*WallDist*HeightRatio;
        elseif BoxShape==2 % round box
            TH=2*pi*rand;
        %    R=rand*WallDist;
            R=InnerWall + rand*(WallDist-InnerWall);
            [Xpos,Ypos]=pol2cart(TH,R);
        elseif BoxShape>2 % for other shapes just start in middle rather than trying to find valid random position
            Xpos=0;
            Ypos=0;
        end

        if NDim==3
            pos=[Xpos Ypos hd]; % initial X-Y-HD position
        else
            pos=[Xpos Ypos]; % initial X-Y position
        end

        W=Pos2Weight(pos); % create first memory
               
        for t=1:Steps   
            if t==PreLearnSteps+RecordSteps+1 && ChangeBox==1

                PlotResults % Call routine to plot results

                pos([1:2])=[0 0]; % place animal in center
                WallDist=1/2;
                HeightRatio=1;
                BoxShape=2;

                PosStep=zeros(RecordSteps,NDim);               % position and head direction for each recording step
                ActStep=zeros(RecordSteps,Nb(NDim+1,2)+1);  % activation at each recording step
                MemActStep=zeros(1,RecordSteps);            % create array for recording place cell spikes

                FigureOffset=FigureOffset+10;                        % to make sure new final figures don't erase initial ones
                StepOffset=PreLearnSteps+RecordSteps;  % to start a new recording at step 1
                    
            end

            inp=Pos2Weight(pos); % find input vector corresponding to current position            
            
            act=CalcAct(W,inp); % activate any existing memories based on current position

            [winact win]=max(act); % find winning memory

            if t>PreLearnSteps   % if recording, record position and activation values, including most active place cell (representing a context cell that receives feedback from all place cells)
                PosStep(t-StepOffset,:)=pos;
                ActStep(t-StepOffset,:)=[(winact*Feedback*(W(win,:))+inp) winact*Feedback];
            end

            if showlearning==1  % plot currect positions of all egocentric cells              
                AllPos=Weight2Pos(W);
                figure(1);
                hold off
                if NDim>2
                    plot3(AllPos(:,1),AllPos(:,2),AllPos(:,3)','.k','MarkerSize',30)
                    hold on
                    plot3(pos(1),pos(2),pos(3),'xb','MarkerSize',10,'LineWidth',2);
                    for z=-2/3:2/3:2/3
                        if BoxShape==1
                            plot3([-WallDist WallDist WallDist -WallDist -WallDist]',HeightRatio*[-WallDist -WallDist WallDist WallDist -WallDist]',[z z z z z]','-k');
                        elseif BoxShape==2
                            th = 0:pi/50:2*pi;
                            xunit = WallDist * cos(th);
                            yunit = WallDist * sin(th);
                            plot3(xunit, yunit, z*ones(size(xunit)),'-k');
                            if InnerWall>0
                                xunit = InnerWall * cos(th);
                                yunit = InnerWall * sin(th);
                                plot3(xunit, yunit, z*ones(size(xunit)),'-k');
                            end
                        end
                    end
                    view(0,0);
                    axis([-1 1 -1 1 -1 1]);
                else
                    plot(AllPos(:,1),AllPos(:,2),'.k','MarkerSize',30);
                    hold on
                    plot(pos(1),pos(2),'xb','MarkerSize',10,'LineWidth',2);
                    if BoxShape==1
                        plot([-WallDist WallDist WallDist -WallDist -WallDist]',HeightRatio*[-WallDist -WallDist WallDist WallDist -WallDist]','-k');
                    elseif BoxShape==2
                        th = 0:pi/50:2*pi;
                        xunit = WallDist * cos(th);
                        yunit = WallDist * sin(th);
                        plot(xunit, yunit,'-k');
                        if InnerWall>0
                            xunit = InnerWall * cos(th);
                            yunit = InnerWall * sin(th);
                            plot(xunit, yunit,'-k');
                        end
                    end
                    axis([-1 1 -1 1]);
                end
                grid off
                axis square
                axis off
                drawnow limitrate;   

                if MakeMovie==1
                    currFrame = getframe;
                    writeVideo(vidObj,currFrame);
                end
            end  
            

            if sum(act>ActThresh)<1; % if no cells above threshold, form new  memory
                W=[W;inp];
                MemActStep=[MemActStep;zeros(1,RecordSteps)]; % add another column to memory recordings
                if t>PreLearnSteps 
                    MemActStep(size(W,1),t-StepOffset)=1;      % record spike for new memory
                end
            elseif sum(act>ActThresh)==1     % only one active so record spike, but no consolidation
                if t>PreLearnSteps 
                    MemActStep(win,t-StepOffset)=1; % record spike for winner
                end
            elseif sum(act>ActThresh)>1 % check to see if there is more than one active memory, in which case consolidate

                act(win)=ActThresh; % this is just a trick to make sure that the winning memory is not part of surround memories


                SurSet=(act-ActThresh)>0; % identify active surrounding memories
                if t>PreLearnSteps 
                    MemActStep(win,t-StepOffset)=1; % record spike for winner
                    MemActStep(SurSet,t-StepOffset)=1; % record spike for surround
                end
                WinAct=CalcAct(W(SurSet,:),W(win,:))-ConsolidateThresh; % use values of winning memory to activate surround memories, then subtract consolidation threshold (BCM threshold modulation between LTP and LTD)

                % BCM rule: WinAct values will be positive or negative depending on
                % whehter they activation is above or below consolidation threshold and
                % this is used in consolidation to specify LTP versus LTD
                % for the differences between the winning memory and
                % surround memories

                W(win,:) = consolidate(W(win,:),W(SurSet,:),WinAct,LearnRate); % consolidation
            
                if showlearning==1 % plot active set and new position of winner
                    AllPos=Weight2Pos(W);
                    WinPos=Weight2Pos(W(win,:));
                    SurSetInd=find(SurSet);
                    hold on
                    if NDim>2
                        for i=1:size(WinAct,1)
                            if WinAct(i)>0
                                plot3(AllPos(SurSetInd(i),1),AllPos(SurSetInd(i),2),AllPos(SurSetInd(i),3),'.r','MarkerSize',30);
                            else
                                plot3(AllPos(SurSetInd(i),1),AllPos(SurSetInd(i),2),AllPos(SurSetInd(i),3),'.g','MarkerSize',30);
                            end
                        end
                        plot3(WinPos(1),WinPos(2),WinPos(3),'.','MarkerSize',30,'MarkerEdgeColor',[1 .75 .25]);
                        view(0,0);
                        axis([-1 1 -1 1 -1 1]);
                    else
                        for i=1:size(WinAct,1)
                            if WinAct(i)>0
                                plot(AllPos(SurSetInd(i),1),AllPos(SurSetInd(i),2),'.r','MarkerSize',30);
                            else
                                plot(AllPos(SurSetInd(i),1),AllPos(SurSetInd(i),2),'.g','MarkerSize',30);
                            end
                        end
                        plot(WinPos(1),WinPos(2),'.','MarkerSize',30,'MarkerEdgeColor',[1 .75 .25]);
                        axis([-1 1 -1 1]);
                    end
                    grid off
                    axis square
                    axis off
                    drawnow;
                    if MakeMovie==1
                        currFrame = getframe;
                        writeVideo(vidObj,currFrame);
                    end
                    hold off
                end
            end    

            % move to the next location
            if wall==1 % if wall encountered on last step, goal direction determines next position
                Whd=0; 
            else       % else, next position depends on weighted sum of goal state and current head direction
                Whd=momentum;
            end
            Xpos=pos(1)+((1-Whd)*st*cos(pi*goal)) + (Whd*st*cos(pi*hd)); % step direction is weighted sum of momentum and randomly selected goal
            Ypos=pos(2)+((1-Whd)*st*sin(pi*goal)) + (Whd*st*sin(pi*hd)); 

            wall=0;
            if BoxShape==2
                [TH,R]=cart2pol(Xpos,Ypos);
                if R>=WallDist
                    wall=1;
                elseif R<=InnerWall
                    wall=1;
                end
            elseif BoxShape==1
                if Xpos<=-WallDist || Xpos>=WallDist || Ypos<=-HeightRatio*WallDist || Ypos>=HeightRatio*WallDist
                    wall=1;
                end
            elseif BoxShape==3
                wall=1;
                if abs(Xpos)<ArmWidth
                    wall=0;
                elseif abs(Ypos)<ArmWidth
                    wall=0;
                elseif abs(Xpos-Ypos)<sqrt(2)*ArmWidth
                    wall=0;
                elseif abs(Xpos+Ypos)<sqrt(2)*ArmWidth
                    wall=0;
                end
                [TH,R]=cart2pol(Xpos,Ypos);
                if R>=WallDist
                    wall=1;
                end
            elseif BoxShape==4
                if Xpos<=-WallDist || Xpos>=WallDist || Ypos<=(-(3/16)*Xpos)-(5/32) || Ypos>=(+(3/16)*Xpos)+(5/32)
                    wall=1;
                end

            end

            if wall==1 % don't allow step outside of wall
                Xpos=pos(1);
                Ypos=pos(2);
            end
          
            if Xpos-pos(1)~=0 && Ypos-pos(2)~=0 % if position has moved, find body angle (which is assumed to be HD)
                hd=atan2(Ypos-pos(2),Xpos-pos(1));
                hd=hd/pi;
            end
            goal=2*rand-1; % at every time step, choose a random goal direction
            if NDim==3
                pos=[Xpos Ypos hd];
            else
                pos=[Xpos Ypos];
            end
        end
    end

    function NewW=consolidate(WWin,WSur,WinAct,LearnRate)
        delta=WinAct'*(ones(size(WinAct))*WWin - WSur); % difference between weights of winner and weights of surround, multiplied by WinAct
        NewW = WWin + LearnRate.*sum(delta,1); % update weights of winner
        NewW=NormWeights(NewW); % make sure new weights correspond to real values in terms of the dimensions
    end

    function W=NormWeights(W)
        TempPos=Weight2Pos(W); % find dimension values that are closest to weights
        W=Pos2Weight(TempPos); % convert these dimension values to weights
    end
    
    function ActTot=CalcAct(W,inp)     % standard neural network activation but divide by number of dimensions (+1 because X/Y is trilinear)   
        ActTot=(W*inp')./(NDim+1);
    end

    function pos=Weight2Pos(W)  % from weights, find positions     

        % this calculate exact values using arcsine and arc cosine
        % approximate values can be found using weighted sum, although with
        % may steps of consolidation, approximate values drift
        
        W=(W*(3/sqrt(2)))-1; % convert from 0-1 to -1 to +1 scale

%         W(W>1)=2-W(W>1);   % weight bounce
%         W(W<-1)=-2-W(W<-1); % weight bounce

        W(W>1)=1;       % keep weight inbounds
        W(W<-1)=-1;     % keep weight inbounds
        for d=1:NDim+1
            Ws=W(:,[Nb(d,1):Nb(d,2)]);
            for i=1:size(Ws,1)
                [val ind]=maxk(Ws(i,:),2,2);
                if ind(1)==1 && ind(2)==3
                    TriPos(i,d)=mean([asin(Ws(i,1)) asin(Ws(i,2)) acos(Ws(i,3))])/pi-(6/6);
                elseif ind(1)==1 && ind(2)==2
                    TriPos(i,d)=mean([acos(Ws(i,1)) asin(Ws(i,2)) acos(Ws(i,3))])/pi-(5/6);
                elseif ind(1)==2 && ind(2)==1
                    TriPos(i,d)=mean([acos(Ws(i,1)) asin(Ws(i,2)) asin(Ws(i,3))])/pi-(2/6);
                elseif ind(1)==2 && ind(2)==3
                    TriPos(i,d)=mean([acos(Ws(i,1)) acos(Ws(i,2)) asin(Ws(i,3))])/pi-(1/6);
                elseif ind(1)==3 && ind(2)==2
                    TriPos(i,d)=mean([asin(Ws(i,1)) acos(Ws(i,2)) asin(Ws(i,3))])/pi+(2/6);
                elseif ind(1)==3 && ind(2)==1
                    TriPos(i,d)=mean([asin(Ws(i,1)) acos(Ws(i,2)) acos(Ws(i,3))])/pi+(3/6);
                end
            end
        end
        pos(:,[1:2])=Tri2Cart(TriPos(:,[1:3]));
        if NDim>2
            pos(:,[3:NDim])=TriPos(:,[4:NDim+1]);
        end
    end

    function W=Pos2Weight(pos)  % from positions, find weights        
        W=[];
        TriPos=Cart2Tri(pos(:,[1 2]));
        if NDim>2       
            TriPos(:,[4:NDim+1])=pos(:,[3:NDim]);
        end
        for d=1:NDim+1
            Locs=(TriPos(d)*ones(1,3)) - Means;
            b=(sqrt(2)/3)*(cos(pi*Locs)+1); % shifted sine wave plus one, then mulitiplied by square root of 2 divided by 3
            W=[W b];
        end
    end      

    function TriPos=Cart2Tri(CartPos)
    
        x=CartPos(:,1);
        y=CartPos(:,2);

    % convert Cartesian coordinates to Trilinear coordinates
        e=x;
        f=(x-sqrt(3).*y)./2;
        g=(x+sqrt(3).*y)./2;
        TriPos=[e f g];
    end

    function CartPos=Tri2Cart(TriPos)
    
    % convert Trilinear coordinates to Cartesian coordinates
    
        e=TriPos(:,1);
        f=TriPos(:,2);
        g=TriPos(:,3);
        % use 3 combinations of 2 trilinear coordinates then average to
        % come up with best estimate of X/Y
    
        x1=e;
        y1=(e-2.*f)./sqrt(3);
    
        x2=e;
        y2=(-e+2.*g)./sqrt(3);
    
        x3=g+f;
        y3=(g-f)./sqrt(3);
    
        x=mean([x1 x2 x3],2);
        y=mean([y1 y2 y3],2);

        CartPos=[x y];
    end

    function PlotResults

        map=parula(100);
        colormap(map);
        pos=Weight2Pos(W);   % find positions of memories for plotting purposes
        if FigureOffset==0 && ChangeBox==1
            csvwrite('BeforeRemap.csv',pos);
        elseif FigureOffset>0 && ChangeBox==1
            csvwrite('AfterRemap.csv',pos);
            BeforePos=csvread('BeforeRemap.csv');
        end
    
        figure(FigureOffset+1);           % plot positions of memories
        hold off
        if NDim>2
            map=hsv(100);
            colormap(map);
            for i=1:size(pos,1)
                plot3(pos(i,1),pos(i,2),pos(i,3)','.','Color',map(round(((pos(i,3)+1)/2)*99)+1,:),'MarkerSize',30)
                hold on
                if FigureOffset>0
                    if i<=size(BeforePos,1)
                        plot3([BeforePos(i,1) pos(i,1)],[BeforePos(i,2) pos(i,2)],[BeforePos(i,3) pos(i,3)],'-','Color',map(round(((BeforePos(i,3)+1)/2)*99)+1,:));
                    end
                end
            end
            map=parula(100);
            colormap(map);
            for z=-2/3:2/3:2/3
                if BoxShape==1
                    plot3([-WallDist WallDist WallDist -WallDist -WallDist]',HeightRatio*[-WallDist -WallDist WallDist WallDist -WallDist]',[z z z z z]','-k');
                elseif BoxShape==2
                    th = 0:pi/50:2*pi;
                    xunit = WallDist * cos(th);
                    yunit = WallDist * sin(th);
                    plot3(xunit, yunit, z*ones(size(xunit)),'-k');
                    if InnerWall>0
                        xunit = InnerWall * cos(th);
                        yunit = InnerWall * sin(th);
                        plot3(xunit, yunit, z*ones(size(xunit)),'-k');
                    end
                end
            end
            set(gca, 'XTick', []);
            set(gca, 'YTick', []);
            set(gca, 'ZTick', []);
            axis([-1 1 -1 1 -1 1]);
            view(2);
        else
            plot(pos(:,1),pos(:,2),'.k','MarkerSize',30);
            hold on
            if BoxShape==1
                plot([-WallDist WallDist WallDist -WallDist -WallDist]',HeightRatio*[-WallDist -WallDist WallDist WallDist -WallDist]','-k');
            elseif BoxShape==2
                th = 0:pi/50:2*pi;
                xunit = WallDist * cos(th);
                yunit = WallDist * sin(th);
                plot(xunit, yunit,'-k');
                if InnerWall>0
                    xunit = InnerWall * cos(th);
                    yunit = InnerWall * sin(th);
                    plot(xunit, yunit,'-k');
                end
            end
            axis([-1 1 -1 1]);
            axis off
        end
        grid off
        axis square
        axis off
        
         figure(FigureOffset+2);           % response of cell common to whole set of memories
        thresh=FiringMap(ActStep(:,Nb(NDim+1,2)+1)); % find appropriate firing threshold    
         figure(FigureOffset+3);
%         GridScores(Reps,4)=AutoCorr(ActStep(:,Nb(NDim+1,2)+1),thresh);
%         GridScores(Reps,5)=size(W,1);
        AutoCorr(ActStep(:,Nb(NDim+1,2)+1),thresh);
        
    
        % plot results from recording
        
        figcount=FigureOffset+4;
        for d=1:NDim+1         % response of each cell in basis set of each dimension       
            Nend=3;
            for i=1:Nend
                figure(figcount);
                subplot_tight(1,Nend,i);
                thresh=FiringMap(ActStep(:,Nb(d,1)+i-1));
                if d==4
                    figure(figcount+1);
                    subplot_tight(1,Nend,i);
        %           GridScores(Reps,i)= AutoCorr(ActStep(:,Nb(d,1)+i-1),thresh);   
                   AutoCorr(ActStep(:,Nb(d,1)+i-1),thresh);    
                end
            end
            figcount=figcount+1;
        end

        if PlotPlaceCells==1
            countcats=[0 0];
            for m=1:size(W,1)
                if (sum(MemActStep(m,:)>0)/RecordSteps)<.025   % .025 for border memories
                    figure(100)
                    countcats(1)=countcats(1)+1
                    if countcats(1)==1
                        hold off
                    end
                else
                    countcats(2)=countcats(2)+1
                    figure(100+countcats(2))
            %        figure(100);
            %        subplot(size(W,1),1,m);
                    hold off
                end
                figcount=figcount+1;
                % plot positions of spikes for memory m
                if NDim>2
                    map=hsv(100);
                    colormap(map);
                    for s=1:RecordSteps
                        if MemActStep(m,s)>0
                            colors=map(round(((PosStep(s,3)+1)/2)*99)+1,:);
                            plot3(PosStep(s,1),PosStep(s,2),PosStep(s,3),'.','Color',colors,'MarkerSize',10)
                            hold on
                        end
                    end
                    map=parula(100);
                    colormap(map);
                    for z=-2/3:2/3:2/3
                        if BoxShape==1
                            plot3([-WallDist WallDist WallDist -WallDist -WallDist]',HeightRatio*[-WallDist -WallDist WallDist WallDist -WallDist]',[z z z z z]','-k');
                        elseif BoxShape==2
                            th = 0:pi/50:2*pi;
                            xunit = WallDist * cos(th);
                            yunit = WallDist * sin(th);
                            plot3(xunit, yunit, z*ones(size(xunit)),'-k');
                            if InnerWall>0
                                xunit = InnerWall * cos(th);
                                yunit = InnerWall * sin(th);
                                plot3(xunit, yunit, z*ones(size(xunit)),'-k');
                            end
                        end
                    end
                    set(gca, 'XTick', []);
                    set(gca, 'YTick', []);
                    set(gca, 'ZTick', []);
                    axis([-WallDist WallDist -HeightRatio*WallDist HeightRatio*WallDist -1 1]);
                    view(2);
                else
                    plot(PosStep(SpikeInd,1),PosStep(SpikeInd,2),'.r','MarkerSize',10);
                    hold on
                    if BoxShape==1
                        plot([-WallDist WallDist WallDist -WallDist -WallDist]',HeightRatio*[-WallDist -WallDist WallDist WallDist -WallDist]','-k');
                    elseif BoxShape==2
                        th = 0:pi/50:2*pi;
                        xunit = WallDist * cos(th);
                        yunit = WallDist * sin(th);
                        plot(xunit, yunit,'-k');
                        if InnerWall>0
                            xunit = InnerWall * cos(th);
                            yunit = InnerWall * sin(th);
                            plot(xunit, yunit, '-k');
                        end
                    end
                    axis([-WallDist WallDist -WallDist WallDist]);
                    axis off
                end
                grid off
          %      axis square
                axis off
            end
            countcats
        end


        function thresh=FiringMap(act)   % based on firing threshold, plot response
            hold off
           thresh=prctile(act,silent);  % find corresponding threshold 
           plot(PosStep(:,1), PosStep(:,2) ,'-k','Color',[.75 .75 .75]);
           hold on
           plot(PosStep(act>=thresh,1), PosStep(act>=thresh,2) ,'.r','MarkerSize',10);      
           axis square
           grid off
           set(gca, 'XTick', []);
           set(gca, 'YTick', []);
           axis([-WallDist WallDist -WallDist WallDist]);
       %    axis([-.5 .5  -.5  .5 ]);
           axis off
           view(2);
        end   
        
        function GridScore=AutoCorr(act,thresh)   % based on firing threshold, plot response       
    
            sigma=.0006;   % width of smoothing
            hold off
            data=PosStep(act>=thresh,[1:2]);
            Xspacing=1/50;
            Yspacing=1/50;
            [X,Y] = meshgrid(-1:Xspacing:1,-1:Yspacing:1);
            density = zeros(size(X));
            for s=1:size(data,1);           % step through spikes
                density(:) = density(:)+mvnpdf([(X(:)-data(s,1)) (Y(:)-data(s,2))],[0 0],sigma.*eye(2));
            end
            density=density./size(data,1);     
            surf(xcorr2(density));
            shading interp
           axis square
           grid off
           set(gca, 'XTick', []);
           set(gca, 'YTick', []);
           set(gca, 'ZTick', []);
           axis([0 200 0 200 0 sum(sum(density.*density))]);
           axis off
           view(2);

           [TH,R]=cart2pol(X,Y);    
           auto_density=xcorr2(density);

            Xspacing=1/100;    % X for autocorrelation
            Yspacing=1/100;    % Y for autocorrelation
            [X_auto,Y_auto] = meshgrid(-1:Xspacing:1,-1:Yspacing:1);
            [TH_auto,R_auto]=cart2pol(X_auto,Y_auto); 
            R_auto=abs(R_auto);  % need to flip the sign of negative radii

            LocalMax=.01;
            PriorCorr=1000000000;
            MaxFound=0;
            WasIncreasing=0;
            while MaxFound==0
                LocalMax=LocalMax+.01;
                NewCorr=mean(auto_density(R_auto>LocalMax-.02 & R_auto<LocalMax+.02));
                if NewCorr<PriorCorr && WasIncreasing==1 % decrease after increase
                    MaxFound=1;
                elseif NewCorr>PriorCorr && WasIncreasing==0
                    WasIncreasing=1;
                end
                if LocalMax>1/3;  % to prevent rmax outside of box
                    MaxFound=1;
                    LocalMax=(1/3); 
                end
                PriorCorr=NewCorr;
            end
            
            rmin=.5*LocalMax; % between center and first peak
            rmax=1.5*LocalMax;

           for r=1:2 % group 1 with 60 and 120 rotation
               rotate=r*(pi/3); % 60 and 120
               TH1=TH+rotate;
               [X1,Y1]=pol2cart(TH1,R);
                density1 = zeros(size(X1));
                for s=1:size(data,1);           % step through spikes
                    density1(:) = density1(:)+mvnpdf([(X1(:)-data(s,1)) (Y1(:)-data(s,2))],[0 0],sigma.*eye(2));
                end
                density1=density1./size(data,1);    
                auto_density1=xcorr2(density1);
                rho1(r)=corr(auto_density(R_auto>rmin & R_auto<rmax),auto_density1(R_auto>rmin & R_auto<rmax));
           end

           for r=1:3 % group 1 with 60 and 120 rotation
               rotate=(pi/6)+((r-1)*(pi/3)); % 30, 90, 150
               TH2=TH+rotate;
               [X2,Y2]=pol2cart(TH2,R);
                density2 = zeros(size(X2));
                for s=1:size(data,1);           % step through spikes
                    density2(:) = density2(:)+mvnpdf([(X2(:)-data(s,1)) (Y2(:)-data(s,2))],[0 0],sigma.*eye(2));
                end
                density2=density2./size(data,1);    
                auto_density2=xcorr2(density2);
                rho2(r)=corr(auto_density(R_auto>rmin & R_auto<rmax),auto_density2(R_auto>rmin & R_auto<rmax));
           end
           GridScore=min(rho1)-max(rho2)
        end  

    end
end