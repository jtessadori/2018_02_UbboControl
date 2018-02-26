classdef MI_ET
    properties
        fs=512; % WARNING: DO NOT change this. Amplifier acquisition rate can only be changed from its panel in Simulink
        rawData;
        cursorPos;
        targetPos;
        figureParams;
        ETdata;
        modelName;
        timeTriggeredEvents;
        clsfr;
        outputLog;
        recLength
        bufferData;
        nTargets;
        currTrial;
        actualTarget;
        actualTargetType;
        inTarget;
        selCounter=0;
        selThreshold=.6;
        selDecay=.1;
        learningRate=1e-4;
    end
    properties (Dependent)
        currTime;
    end
    properties (Hidden)
        CDlength;
        isExpClosed=0;
        isTraining=0;
        tform; % Calibration transformation from pixel coordinates of gaze to image coordinates
        gazePosBuffer; % Buffer of recent gaze points, per target
    end
    methods
        %% Constructor
        function obj=MI_ET(varargin)
            % If an argument is passed, it must be a structure matching
            % clsfr template (i.e. basically, an output of another
            % session). Some parameters (e.g. sampling frequency of
            % amplifier and buffer window length) cannot be changed
            % directly from here, so please do make sure that they're
            % matching with the current settings of relevant Simulink
            % model.
            
            % Set length of initial countdown, in seconds
            obj.CDlength=15;
            
            % Set desired length of recording. 
            obj.recLength=600;
            
            % Set colors for different objects
            obj.figureParams.bg=[.05,.05,.05];
            obj.figureParams.targetColor{1}=[0,.4,0];
            obj.figureParams.targetColor{2}=[0,0,.4];
            obj.figureParams.cursorColor=[.4,0,.1];
            
            % Set shape and pos for cursor
            obj.figureParams.cursorShape.X=[-.05,.05,.05,-.05];
            obj.figureParams.cursorShape.Y=[-.05,-.05,.05,.05];
            obj.cursorPos=[0,0];
            
            % Set possible target centers and shape
            obj.figureParams.targetRadius=.2;
            targetX=(1:3)'*ones(1,3);
            targetY=ones(3,1)*(1:3);
            targetX=(targetX-2)*(1-obj.figureParams.targetRadius);
            targetY=(targetY-2)*(1-obj.figureParams.targetRadius);
            obj.nTargets=numel(targetX);
            obj.targetPos=cell(obj.nTargets,1);
            for currT=1:obj.nTargets
                obj.targetPos{currT}=[targetX(currT),targetY(currT)];
            end
            obj.figureParams.targetShape.X=cos(linspace(0,2*pi,40))*obj.figureParams.targetRadius;
            obj.figureParams.targetShape.Y=sin(linspace(0,2*pi,40))*obj.figureParams.targetRadius;
            
            % Normalization buffer length
            obj.bufferData.bufferLength=30; % Data will be normalized in mean and SD over this amount of seconds
            
            % Default parameters
            if nargin==0
                clsfrDefault.fs=obj.fs;
                clsfrDefault.ARmodelOrder=14; % From: "Noninvasive Electroencephalogram Based Control of a Robotic Arm for Reach and Grasp Tasks"
                %             clsfrDefault.bandLims=[8,12,18,25]; % Limits of band of interest - i.e. 8-to-12 and 18-to-25
                %             clsfrDefault.bandLims=ceil((1:.5:24.5)); % Limits of band of interest - single Hertz bands from 1 to 25
                clsfrDefault.bandLims=[10,14]; % Cited work
                % WARNING: the following line DOESN'T DO ANYTHING; it is just a
                % reference. Buffer length has to be changed in the Simulink
                % model (specifically, in the triggeredBuffer mask
                % initialization)
                clsfrDefault.winLength=.4; % Window length, in seconds. Also, cited work
                clsfrDefault.relChannels=1:16; % Use all channels
%                 clsfrDefault.relChannels=[7,11]; % C3 and C4 in 16 el setup
                clsfrDefault.nChannels=length(clsfrDefault.relChannels);
                obj.clsfr=clsfrDefault;
            else
                obj.clsfr=varargin{1}.clsfr;
            end
            
            % Initialize a few things
            obj.outputLog.time=[];
            obj.outputLog.cursorPos=[];
            obj.outputLog.feats=[];
            obj.outputLog.isTraining=[];
            obj.outputLog.targetsReached.time=[];
            obj.outputLog.actualTarget=[];
            obj.outputLog.isInTarget=[];
            obj.outputLog.selCounter=[];
            obj.outputLog.currEst=[];
            obj.outputLog.actualTargetType=[];
            obj.outputLog.coeffsMat=[];
            
            % Ask user whether to start experiment right away
            clc;
            if ~strcmpi(input('Start experiment now? [Y/n]\n','s'),'n')
                obj=runExperiment(obj);
            end
        end
        
        % Other methods
        function obj=runExperiment(obj)
            % Variables on base workspace will be used to trigger closing
            % of experiment
            assignin('base','isExpClosing',0);
            
            % Start eye-tracking
            MI_ET.startEyeTracking;
            
            % Ask user whether to calibrate eye-tracking. Cursor should not
            % be visible during calibration
            persistent calibrationTransform
            if ~isempty(calibrationTransform)
                userChoice=input('Perform gaze tracking calibration? [y/N]\n','s');
            else
                userChoice='y';
            end
                        
            % Opens black figure as background
            obj=createExpFigure(obj);
            
            % Perform calibration, if needed
            if strcmpi(userChoice,'y');
                obj=obj.performETcalibration;
                calibrationTransform=obj.tform;
            else
                obj.tform=calibrationTransform;
            end
            
            % Sets name of Simulink model to be used for acquisition
            obj.modelName='SimpleAcquisition_16ch_2014a_RT';
            
            % Randomly select a target and target type
            obj.actualTarget=1;
            obj.targetPos=cell(1);
            obj.targetPos{obj.actualTarget}=rand(1,2)*2*(1-obj.figureParams.targetRadius)-1+obj.figureParams.targetRadius;
            obj.actualTargetType=(randn>0)+1;
            
            % Prepares Simulink model (i.e. starts recording, basically)
            obj.prepareSimulinkModel;
            
            % Generates array of time triggered events
            obj.timeTriggeredEvents{1}=timeTriggeredEvent('expCallback',0);
            obj.timeTriggeredEvents{2}=timeTriggeredEvent('toggleTraining',0);
            
            % Shows a countdown
            obj.startCountdown(obj.CDlength);
            
            % Perform bulk of experiment
            obj=manageExperiment(obj);
            
            % Closes exp window and saves data
            obj.closeExp;
        end
        
        function obj=manageExperiment(obj)
            % Generate file name used to save experiment data
            fileName=datestr(now,30);
            while ~evalin('base','isExpClosing')&&obj.currTime<=(obj.recLength+obj.CDlength)
%                 try
                pause(0.001);
                for currTTevent=1:length(obj.timeTriggeredEvents);
                    obj=checkAndExecute(obj.timeTriggeredEvents{currTTevent},obj.currTime,obj);
                    pause(0.001);
                end
%                 catch ME
%                     assignin('base','isExpClosing',1);
%                     keyboard;
%                 end
            end
            obj.isExpClosed=1;
            delete(gcf);
            set_param(obj.modelName,'SimulationCommand','Stop');
            set_param(obj.modelName,'StartFcn','')
            obj.rawData=evalin('base','rawData');
            save(fileName,'obj');
            
            % Clear variables from base workspace
            evalin('base','clear listener*');
            evalin('base','clear toggleTraining');
        end
        
        function obj=createExpFigure(obj)
            % Set figure properties
            obj.figureParams.handle=gcf;
            set(obj.figureParams.handle,'Tag',mfilename,...
                'Toolbar','none',...
                'MenuBar','none',...
                'Units','normalized',...
                'Resize','off',...
                'NumberTitle','off',...
                'Name','',...
                'Color',obj.figureParams.bg,...
                'RendererMode','Manual',...
                'Renderer','OpenGL',...
                'WindowKeyPressFcn',@KeyPressed,...
                'CloseRequestFcn',@OnClosing,...
                'WindowButtonMotionFcn',@onMouseMove);
            
            % Create cursor outside of visible field
            obj.figureParams.cursor=patch(obj.figureParams.cursorShape.X+100,obj.figureParams.cursorShape.Y+100,obj.figureParams.cursorColor);
            
            % Create target
            obj.figureParams.target=patch(obj.figureParams.targetShape.X,obj.figureParams.targetShape.Y,obj.figureParams.targetColor{1});
            
            % Set and remove figure axis
            ylim([-1,1]);
            xlim([-1,1]);
            set(gcf,'units','normalized','position',[0,0,1,1]);
            axis square
            axis('off')
            
            % Remove box around figure
            %             undecorateFig;
        end
        
        function obj=expCallback(obj)
            global udpr
            % Evaluate cursor position from gaze
            coords=str2num(char(udpr.step)'); %#ok<ST2NM>
            if ~isempty(coords)
                obj.cursorPos=obj.tform.transformPointsForward(coords');
            end
            
            % Filter gaze coordinates
            persistent previousGazePos
            if isempty(previousGazePos)||sum(isnan(previousGazePos))
                previousGazePos=obj.cursorPos;
            end
            gazeSpeed=sqrt(sum((previousGazePos-obj.cursorPos).^2));
            obj.cursorPos=previousGazePos*(1-sqrt(gazeSpeed))+obj.cursorPos*sqrt(gazeSpeed);
            previousGazePos=obj.cursorPos;
            set(obj.figureParams.cursor,'XData',obj.figureParams.cursorShape.X+obj.cursorPos(1),'YData',obj.figureParams.cursorShape.Y+obj.cursorPos(2));
            
            % Check if cursor is within target
            if sqrt(sum((obj.cursorPos-obj.targetPos{obj.actualTarget}).^2))<obj.figureParams.targetRadius
                obj.inTarget=1;
            else
                obj.inTarget=0;
            end
            
            % Recover data buffer from base workspace (Simulink puts them
            % there)
            dataWindow=evalin('base','currData');
            dataTimeStamp=obj.currTime;
            
            % Update normalization buffer and normalize data
            obj=updateBufferData(obj,dataWindow,dataTimeStamp);
            fixDim=@(x)repmat(x,size(dataWindow,1),1);
            if obj.bufferData.SD>0 % Skip first window to prevent Infs
                dataWindow=(dataWindow-fixDim(obj.bufferData.mean))./fixDim(obj.bufferData.SD);
            end
            
            % If this is first iteration, compute laplacian filter weights
            if ~isfield(obj.clsfr,'lapFltrWeights')
                [~,obj.clsfr.lapFltrWeights]=MI_ET.applyLapFilter(dataWindow);
            end
            
            % Recover bandpower data from data buffer
            BP=MI_ET.preprocessData(dataWindow*obj.clsfr.lapFltrWeights);
            
            % If only a subset of feature has been chosen, remove the rest
            if isfield(obj.clsfr,'featsIdx')
                BP=BP(obj.clsfr.featsIdx);
            end
            
            % If training is ongoing, update linear map
            if obj.isTraining
                obj=obj.updateClsfr(BP);
            end
            
            % If cursor is within target, fills up selection counter
            if isfield(obj.clsfr,'mat')
                feats=cat(2,1,BP);
                currEst=1./(1+exp(-(feats*obj.clsfr.mat)));
            elseif isfield(obj.clsfr,'svm')
                [~,currEst]=predict(obj.clsfr.svm,BP);
                currEst=currEst(1);
            else
                currEst=0;
            end
            if obj.inTarget
                % Use EEG data when available, otherwise use gaze alone
                if isfield(obj.clsfr,'mat')||isfield(obj.clsfr,'svm')
                    switch obj.actualTargetType
                        case 1
                            obj.selCounter=max(0,obj.selCounter+.1*(currEst-obj.selThreshold)/(1-obj.selThreshold));
                        case 2
                            obj.selCounter=max(0,obj.selCounter+.1*((1-obj.selThreshold)-currEst)/(1-obj.selThreshold));
                    end
                else
                    obj.selCounter=obj.selCounter+.01;
                end
                if obj.selCounter>=1
                    obj.actualTarget=obj.actualTarget+1;
                    obj.targetPos{obj.actualTarget}=rand(1,2)*2*(1-obj.figureParams.targetRadius)-1+obj.figureParams.targetRadius;
                    obj.actualTargetType=(randn>0)+1;
                    obj.selCounter=0;
                    
                    % Slowly decrease learning rate each time a target is
                    % selected
                    obj.learningRate=.95*obj.learningRate;
                end
            else
                obj.selCounter=obj.selCounter*(1-obj.selDecay);
            end
            
%             % Always assume subject is staring at center of current target
%             obj.gazePosBuffer{obj.actualTarget}=cat(2,obj.gazePosBuffer{obj.actualTarget},coords);
%             obj.gazePosBuffer{obj.actualTarget}(:,1)=[];
%             obj=obj.calibrateEyeTracker;
            
            % Update target pos and color
            xTarget=obj.figureParams.targetShape.X+obj.targetPos{obj.actualTarget}(1);
            yTarget=obj.figureParams.targetShape.Y+obj.targetPos{obj.actualTarget}(2);
            targetColor=obj.figureParams.targetColor{obj.actualTargetType}*(1-obj.selCounter);
            set(obj.figureParams.target,'XData',xTarget,'YData',yTarget,'FaceColor',targetColor);
            drawnow;
            
            % Add relevant info to log
            obj.outputLog.cursorPos=cat(1,obj.outputLog.cursorPos,obj.cursorPos);
            obj.outputLog.actualTarget=cat(1,obj.outputLog.actualTarget,obj.actualTarget);
            obj.outputLog.feats=cat(1,obj.outputLog.feats,BP);
            obj.outputLog.isTraining=cat(1,obj.outputLog.isTraining,obj.isTraining);
            obj.outputLog.time=cat(1,obj.outputLog.time,obj.currTime);
            obj.outputLog.isInTarget=cat(1,obj.outputLog.isInTarget,obj.inTarget);
            obj.outputLog.selCounter=cat(1,obj.outputLog.selCounter,obj.selCounter);
            obj.outputLog.currEst=cat(1,obj.outputLog.currEst,currEst);
            obj.outputLog.actualTargetType=cat(1,obj.outputLog.actualTargetType,obj.actualTargetType);
            if isfield(obj.clsfr,'mat')
                obj.outputLog.coeffsMat=cat(1,obj.outputLog.coeffsMat,obj.clsfr.mat');
            else
                obj.outputLog.coeffsMat=cat(1,obj.outputLog.coeffsMat,zeros(1,length(BP)+1));
            end
            
            % Set next evaluation time for this function
            obj.timeTriggeredEvents{1}.triggersLog=[obj.timeTriggeredEvents{1}.triggersLog,obj.currTime];
            obj.timeTriggeredEvents{1}.nextTrigger=obj.currTime+.05;
        end
        
        function obj=updateBufferData(obj,dataWindow,dataTimeStamp)
            % At the moment, I am only taking a single time point for each
            % dataWindow to estimate mean and sd in the last thirty
            % seconds. This is VERY approximate, better ideas for a
            % different solution are welcome
            if isfield(obj.bufferData,'data')
                obj.bufferData.data=cat(1,obj.bufferData.data,dataWindow(1,:));
                obj.bufferData.timeStamps=cat(1,obj.bufferData.timeStamps,dataTimeStamp);
            else
                obj.bufferData.data=dataWindow(1,:);
                obj.bufferData.timeStamps=dataTimeStamp;
            end
            toBeRemoved=obj.currTime-obj.bufferData.timeStamps>obj.bufferData.bufferLength; % Last value is buffer length, in seconds
            obj.bufferData.data(toBeRemoved,:)=[];
            obj.bufferData.timeStamps(toBeRemoved)=[];
            obj.bufferData.mean=median(obj.bufferData.data,1);
            obj.bufferData.SD=1.4826*median(abs(obj.bufferData.data-repmat(obj.bufferData.mean,size(obj.bufferData.data,1),1)));
%             obj.bufferData.SD=std(obj.bufferData.data,[],1);
        end
        
        function obj=updateClsfr(obj,BP)
            obj.clsfr.feats{obj.actualTarget}=cat(2,1,BP);
            
            % Define link function
            if ~isfield(obj.clsfr,'mat')
                obj.clsfr.mat=zeros(length(obj.clsfr.feats{obj.actualTarget}'),1);
            end
            
            % Compute output of current classifier
            currEst=1./(1+exp(-(obj.clsfr.feats{obj.actualTarget}*obj.clsfr.mat)));
            
            % Update weights of GLM model
            switch obj.actualTargetType
                case 1
                    obj.clsfr.mat=MI_ET.updateWeights(obj.clsfr.mat,BP,currEst,obj.inTarget,obj.learningRate);
                case 2
                    obj.clsfr.mat=MI_ET.updateWeights(obj.clsfr.mat,BP,currEst,1-obj.inTarget,obj.learningRate);
            end
        end
        
        function obj=toggleTraining(obj)
            if evalin('base','exist(''toggleTraining'',''var'')')&&evalin('base','toggleTraining')
                obj.isTraining=~obj.isTraining;
                assignin('base','toggleTraining',0);
                figure(obj.figureParams.handle)
                if obj.isTraining
                    textHandle=text(-.4,.5,'Training on');
                else
                    textHandle=text(-.4,.5,'Training off');
                end
                set(textHandle,'Color','white','FontSize',64);
            	wait(obj,.5);
                delete(textHandle);
            end
            
            % Set next evaluation time for this function
            obj.timeTriggeredEvents{2}.triggersLog=[obj.timeTriggeredEvents{2}.triggersLog,obj.currTime];
            obj.timeTriggeredEvents{2}.nextTrigger=obj.currTime+.5;
        end
                
        function obj=performETcalibration(obj)
            % Generate one target in each position for about 200 steps
            % (each step is 10 ms).
            global udpr
            obj.gazePosBuffer=cell(obj.nTargets,1);
            pause(2);
            for currTarget=1:obj.nTargets
                set(obj.figureParams.target,'XData',obj.targetPos{currTarget}(1)+obj.figureParams.targetShape.X,'YData',obj.targetPos{currTarget}(2)+obj.figureParams.targetShape.Y);
                drawnow;
                obj.gazePosBuffer{currTarget}=[];
                for currRep=1:200
                    coords=str2num(char(udpr.step)'); %#ok<ST2NM>
                    obj.gazePosBuffer{currTarget}=cat(2,obj.gazePosBuffer{currTarget},coords);
                    pause(0.01);
                end
            end
            
            % Move target off-screen
            set(obj.figureParams.target,'XData',100+obj.figureParams.targetShape.X,'YData',obj.targetPos{currTarget}(2)+obj.figureParams.targetShape.Y);
            
            % Perform actual calibration
            obj=obj.calibrateEyeTracker;
        end
       
        function obj=calibrateEyeTracker(obj)
            % Use coordinates of gaze during each target fixation to
            % calibrate transformation
            fixedPoints=cell2mat(obj.targetPos);
            movingPoints=[cellfun(@(x)median(x(1,:)),obj.gazePosBuffer),cellfun(@(x)median(x(2,:)),obj.gazePosBuffer)];
            obj.tform=fitgeotrans(movingPoints,fixedPoints,'Projective');
        end
        
        function prepareSimulinkModel(obj)
            % Check whether simulink model file can be found
            if ~exist(obj.modelName,'file')
                warning('Cannot find model %s.\nPress Enter to continue.\n',obj.modelName);
                input('');
                [fileName,pathName]=uigetfile('*.slx','Select Simulink model to load:');
                obj.modelName=sprintf('%s\\%s',pathName,fileName);
            end
            % Load model
            load_system(obj.modelName);
            
            % Check whether simulation was already running, and, in case,
            % stop it
            if bdIsLoaded(obj.modelName)&&strcmp(get_param(obj.modelName,'SimulationStatus'),'running')
                set_param(obj.modelName,'SimulationCommand','Stop');
            end
            
            % Add event listener to triggered buffer event.
            set_param(obj.modelName,'StartFcn',sprintf('simulinkModelStartFcn(''%s'')',obj.modelName))
            set_param(obj.modelName,'StopTime','inf');
            set_param(obj.modelName,'FixedStep',['1/',num2str(obj.fs)]);
            set_param(obj.modelName,'SimulationCommand','Start');
        end
        
        function wait(obj,pauseLength)
            startTime=get_param(obj.modelName,'SimulationTime');
            while strcmp(get_param(obj.modelName,'SimulationStatus'),'running')&&get_param(obj.modelName,'SimulationTime')<=startTime+pauseLength
                pause(1/(2*obj.fs));
            end
        end
        
        function startCountdown(obj,nSecs)
            % countdown to experiment start
            figure(obj.figureParams.handle)
            for cntDown=nSecs:-1:1
                if ~exist('textHandle','var')
                    textHandle=text(-.05,.5,num2str(cntDown));
                else
                    set(textHandle,'String',num2str(cntDown));
                end
                set(textHandle,'Color','white','FontSize',64);
                pause(1);
            end
            delete(textHandle);
        end
        
        function obj=computeMIclassifierLassoGLM(obj)
            % Perform feature selection
            [obj,feats,lbls]=performFeatureSelection(obj);            
            
            % Train classifier
            fprintf('Training MI classifier. Please be patient, it will take some time...\n');
            [B,stats]=lassoglm(feats,lbls==1,'binomial','Alpha',.5,'CV',4);
            B=B(:,stats.Index1SE);
            Intercept=stats.Intercept(stats.Index1SE);
            
            % Remove features ignored by model
            ignoredFeats=B(:,stats.Index1SE)==0;
            B(ignoredFeats)=[];
            obj.clsfr.mat=[Intercept;B];
            obj.clsfr.featsIdx(ignoredFeats)=[];
        end
        
        function obj=computeMIclassifierSVM(obj)
            % Perform feature selection
            [obj,feats,lbls]=performFeatureSelection(obj);
            
            % Train classifier
            fprintf('Training MI classifier. Please be patient, it will take some time...\n');
            obj.clsfr.svm=fitcsvm(feats,lbls,'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
            obj.clsfr.svm=fitPosterior(obj.clsfr.svm);
            
            % Remove previous coefficients, if they exist
            if isfield(obj.clsfr,'mat')
                obj.clsfr=rmfield(obj.clsfr,'mat');
            end
        end
        
        function [obj,feats,lbls]=performFeatureSelection(obj)
            %             % Recover MI data
            %             relevantData=obj.outputLog.isTraining&obj.outputLog.isInTarget;
            %             allFeats=obj.outputLog.feats(relevantData,:);
            %             lbls=obj.outputLog.actualTargetType(relevantData);
            winStarts=obj.outputLog.time*obj.fs;
            winEnds=winStarts+round(0.4*obj.fs);
            lapData=MI_ET.applyLapFilter(obj.rawData.data);
            relevantData=obj.outputLog.isTraining&obj.outputLog.isInTarget;
            winStarts(winEnds>length(lapData))=[];
            relevantData(winEnds>length(lapData))=[];
            winEnds(winEnds>length(lapData))=[];
            dataWins=zeros(length(winStarts),round(0.4*obj.fs),size(lapData,2));
            for currWin=1:length(winStarts)
                dataWins(currWin,:,:)=lapData(winStarts(currWin)+1:winEnds(currWin),:);
            end
            allFeats=MI_ET.preprocessData(dataWins);
            allFeats=allFeats(relevantData,:);
            lbls=interp1(obj.outputLog.time,obj.outputLog.actualTargetType,(winStarts+winEnds)*.5/obj.fs,'nearest','extrap');
            lbls=lbls(relevantData);
            
            % Make a first selection of relevant features
            classLbls=unique(lbls);
            m=zeros(length(classLbls),size(allFeats,2));
            md=zeros(size(m));
            for currClass=1:length(classLbls)
                % Use median and mad as proxy for mean and sd, to reduce
                % relevance of artifacts
                m(currClass,:)=median(allFeats(lbls==classLbls(currClass),:));
                md(currClass,:)=1.4826*mad(allFeats(lbls==classLbls(currClass),:),1);
            end
            computeWorth=@(m1,m2,md1,md2)abs((m1-m2)./sqrt(md1.^2+md2.^2));
            featWorth=computeWorth(m(1,:),m(2,:),md(1,:),md(2,:));
            
            % Keep features with a worth greater than 0.3 (keep at least
            % 15)
            [sortedWorth,featOrdr]=sort(featWorth,'descend');
            goodFeatsNumber=sum(sortedWorth>.2);
            goodFeatsIdx=featOrdr(1:max(15,goodFeatsNumber));
            feats=allFeats(:,goodFeatsIdx);
            obj.clsfr.featsIdx=goodFeatsIdx;
        end
        
        %% Dependent properties
        function cTime=get.currTime(obj)
            if obj.isExpClosed
                cTime=obj.rawData.Time(end);
            else
                cTime=get_param(obj.modelName,'SimulationTime');
            end
        end        
    end
    methods (Static)
        function [freqFeats,timeFeats]=preprocessData(dataWins)
            % This function takes either one time window as input (during
            % testing) or a vector of them (during training). Reshape
            % single window to make it consistent
            persistent filterData
            if ~isfield(filterData,'B')
                filterData.filterOrder=4;
                [filterData.B,filterData.A]=cheby1(filterData.filterOrder,6,(.2/8)/2); %Assuming here that windows are taken at 8Hz and implementing a 0.2Hz lowpass filter
            end
            
            if length(size(dataWins))==2
                dataWins=reshape(dataWins,1,size(dataWins,1),size(dataWins,2));
            end
            [nWins,~,nChannels]=size(dataWins);
            timeFeats=zeros(size(dataWins));
            freqFeats=zeros(nWins,129,nChannels);
            % Preprocess each input window
            for currWin=1:nWins
                for currCh=1:nChannels
                    relData=squeeze(dataWins(currWin,:,currCh));
                    % Normalize: set first sample to zero, sd to 1
                    relData=(relData-relData(1))/std(relData);
                    % Remove linear trend
                    relData=detrend(relData);
                    timeFeats(currWin,:,currCh)=relData;
                    % Compute log of bandpower
                    freqFeats(currWin,:,currCh)=pyulear(relData.*blackman(length(relData))',16);
                end                
            end
            
            % Consider only frequencies up to ~60Hz
            freqFeats(:,31:end,:)=[];
            
            % Normalize, then extract logs
            freqFeats=freqFeats./repmat(sum(freqFeats,3),1,1,size(freqFeats,3));
            freqFeats=log(freqFeats);
            
            % Reshape data
            freqFeats=reshape(freqFeats,size(freqFeats,1),[]);
            
%             % Lowpass data
%             if size(dataWins,1)>1
%                 freqFeats=filter(filterData.B,filterData.A,freqFeats);
%             else
%                 if ~isfield(filterData,'X')
%                     filterData.X=zeros(filterData.filterOrder+1,size(freqFeats,2));
%                     filterData.Y=zeros(filterData.filterOrder,size(freqFeats,2));
%                 end
%                 filterData.X=[freqFeats;filterData.X];
%                 filterData.X(end,:)=[];
%                 freqFeats=filterData.B*filterData.X-filterData.A(2:end)*filterData.Y;
%                 filterData.Y=[freqFeats;filterData.Y];
%                 filterData.Y(end,:)=[];
%             end
        end
        
        function [outData,fltrWeights]=applyLapFilter(inData)
            try
                load('elMap16.mat')
            catch ME %#ok<NASGU>
                warning('''elMap.mat'' not found. Electrode map required for laplacian filters.');
                outData=[];
                return;
            end
            fltrWeights=zeros(size(inData,2));
            for currEl=1:size(inData,2)
                neighborsMap=zeros(size(elMap16.elMat));
                neighborsMap(elMap16.elMat==currEl)=1;
                neighborsMap=imdilate(neighborsMap,strel('diamond',1));
                neighborsMap(elMap16.elMat==currEl)=0;
                validNeighbors=logical(neighborsMap.*elMap16.elMat);
                fltrWeights(currEl,elMap16.elMat(validNeighbors))=-1/sum(sum(validNeighbors));
                fltrWeights(currEl,currEl)=1;
            end
            outData=inData*fltrWeights';
        end
        
        function closeExp
            % Signals experiment to close
            assignin('base','isExpClosing',1);
        end
        
        function startEyeTracking
            % Losing udpr is bad (cannot close port from Matlab anymore).
            % Declare it as global so that it can be recovered outside of
            % this class, as well
            global udpr
            % Local port numeber is given by GazeTrackEyeXGazeStream
            % Buffer size is pretty much arbitrarily chosen: it should be
            % so that it contains little more than one entry
            if isempty(udpr)
                udpr=dsp.UDPReceiver('LocalIPPort',11000,'ReceiveBufferSize',20);
            end
            
            % Launch GazeTrackEyeXGazeStream in async mode, if not already
            % running (prompts user, I have no idea how to check if
            % external code is running)
            clc
            startET=input('WARNING: select Yes only if GazeStream is not running already.\nDo you want to start eye tracking? Y/N [N]: ','s');
            if isempty(startET)
                startET='n';
            end
            if strcmpi(startET,'y')
                !C:\Code\Sources\GazeTrackEyeXGazeStream\GazeTrackEyeXGazeStream.exe &
            end
        end
        
        function wOut=updateWeights(wIn,feats,E,t,lr)
            % feats new sample
            % E current classifier prediction
            % t true label
            feats=[1,feats];
            wOut=wIn+((lr*(t-E))'*feats)';
        end
    end
end

function simulinkModelStartFcn(modelName) %#ok<DEFNU>
% Start function for Simulink model.
blockName=sprintf('%s/triggeredBuffer/Buffer',modelName);
assignin('base','listener',add_exec_event_listener(blockName,'PostOutputs',@acquireBufferedData));
end

function acquireBufferedData(block,~)
assignin('base','currData',block.OutputPort(1).Data);
assignin('base','currTime',block.SampleTime);
end

function onMouseMove(~,~)
% Makes mouse pointer invisible
if ~strcmp(get(gcf,'Pointer'),'custom')
    set(gcf,'PointerShapeCData',NaN(16));
    set(gcf,'Pointer','custom');
end
end

function KeyPressed(~,eventdata,~)
% This is called each time a keyboard key is pressed while the mouse cursor
% is within the window figure area
if strcmp(eventdata.Key,'escape')
    MI_ET.closeExp;
end
if strcmp(eventdata.Key,'p')
    keyboard;
    %     assignin('base','pauseNextTrial',1)
end
if strcmp(eventdata.Key,'t')
    assignin('base','toggleTraining',1);
end
end

function OnClosing(~,~)
% Overrides normal closing procedure so that regardless of how figure is
% closed logged data is not lost
MI_ET.closeExp;
end