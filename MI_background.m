classdef MI_background
    properties
        fs=512; % WARNING: DO NOT change this. Amplifier acquisition rate can only be changed from its panel in Simulink
        rawData;
        modelName;
        timeTriggeredEvents;
        outputLog;
        bufferData;
        clsfr;
        figHandle;
        UDPchannels;
        selCounter=0; % Current value of selection counter (selection occurs at 1)
        selThreshold=.8; % Classifier output must be at least this high before selCounter is updated
        selDecay=.1; % Decay rate of selCounter when gaze is outside keys (e.g. .1 means 10% decay per iteration)
        selWait=0; % Training will wait this many seconds after gaze is moved into key before considering moro imagery started
    end
    properties (Dependent)
        currTime;
        actualTarget;
        inTarget;
        currKeyID;
    end
    properties (Hidden)
        CDlength;
        isExpClosed=0;
        isTraining=0;
        selStartTime;
        lastKeyID;
    end
    methods
        %% Constructor
        function obj=MI_background(varargin)
            % If an argument is passed, it must be a structure matching
            % clsfr template (i.e. basically, an output of another
            % session). Some parameters (e.g. sampling frequency of
            % amplifier and buffer window length) cannot be changed
            % directly from here, so please do make sure that they're
            % matching with the current settings of relevant Simulink
            % model.
            
            % Set length of initial countdown, in seconds 
            obj.CDlength=15;
            
            % Define expected inputs and output ports for udp communication
            obj.UDPchannels.outPort=7500;
            obj.UDPchannels.inPort=6100;
                       
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
            obj.outputLog.feats=[];
            obj.outputLog.isTraining=[];
            obj.outputLog.isInTarget=[];
            obj.outputLog.selCounter=[];
            obj.outputLog.UDPlog=cell(0);
            obj.outputLog.selStart=[];
            obj.outputLog.selEnd=[];
        end
        
        % Other methods
        function obj=start(obj)
            % Variables on base workspace will be used to trigger closing
            % of experiment
            assignin('base','isExpClosing',0);
            
            % Open udp channels for communication
            obj=openUDPchannels(obj);

            % Opens figure as background. Useful for accepting key presses
            obj=createExpFigure(obj);
            
            % Launch keyboard program
%             !C:\Code\keyboard2\keyboad.exe &
                        
            % Sets name of Simulink model to be used for acquisition
            obj.modelName='SimpleAcquisition_16ch_2014a_RT';
            
            % Prepares Simulink model (i.e. starts recording, basically)
            obj.prepareSimulinkModel;
            
            % Wait for amplifiers to set
            pause(obj.CDlength);
            
            % Generates array of time triggered events
            obj.timeTriggeredEvents{1}=timeTriggeredEvent('expCallback',0);
            obj.timeTriggeredEvents{2}=timeTriggeredEvent('toggleTraining',0);
            
            % Perform bulk of experiment
            obj=manageExperiment(obj);
            
            % Closes exp window and saves data
            obj.closeExp;
        end
        
        function obj=manageExperiment(obj)
            % Generate file name used to save experiment data
            fileName=datestr(now,30);
            while ~evalin('base','isExpClosing')
                pause(0.001);
                for currTTevent=1:length(obj.timeTriggeredEvents);
                    obj=checkAndExecute(obj.timeTriggeredEvents{currTTevent},obj.currTime,obj);
                    pause(0.001);
                end
            end
            obj.isExpClosed=1;
            delete(gcf);
            set_param(obj.modelName,'SimulationCommand','Stop');
            set_param(obj.modelName,'StartFcn','')
            obj.rawData=evalin('base','rawData');
            save(fileName,'obj');
            
            % Release receiver UDP port
            obj.UDPchannels.udpr.isDone;
            
            % Clear variables from base workspace
            evalin('base','clear listener*');
            evalin('base','clear toggleTraining');
        end
        
        function obj=createExpFigure(obj)
            % Set figure properties
            obj.figHandle=figure;
            set(obj.figHandle,'Tag',mfilename,...
                'Toolbar','none',...
                'MenuBar','none',...
                'Units','normalized',...
                'Resize','off',...
                'NumberTitle','off',...
                'Name','',...
                'WindowKeyPressFcn',@KeyPressed,...
                'CloseRequestFcn',@OnClosing,...
                'WindowButtonMotionFcn',@onMouseMove);
            
            % Remove figure axis
            axis([-1,1,-1,1]);
            axis('off')
        end
        
        function obj=expCallback(obj)
            % Recover incoming data and update relevant properties
            obj.lastKeyID=obj.currKeyID;
            obj.UDPchannels.lastInput=char(obj.UDPchannels.udpr.step)';
%             fprintf('%s\n',obj.UDPchannels.lastInput)
            if ~obj.lastKeyID==obj.currKeyID
                obj.selStartTime=obj.currTime;
                obj.selCounter=0;
            end
                        
            % Only execute the following if first EEG data-block has
            % arrived
            if evalin('base','exist(''currData'',''var'')')
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
                    [~,obj.clsfr.lapFltrWeights]=MI_background.applyLapFilter(dataWindow);
                end
                
                % Recover bandpower data from data buffer
                BP=MI_ET.preprocessData(dataWindow*obj.clsfr.lapFltrWeights);
                
                % If only a subset of feature has been chosen, remove the rest
                if isfield(obj.clsfr,'featsIdx')
                    BP=BP(obj.clsfr.featsIdx);
                end
                
                % If training is ongoing, update linear map
                if obj.isTraining
                    if obj.inTarget
                        if obj.currTime-obj.selStartTime>obj.selWait
                            obj=obj.computeclsfr(BP);
                        end
                    else
                        obj=obj.computeclsfr(BP);
                    end
                end
                               
                % If current key has already been selected, reset counter to zero
                if obj.selCounter>1 
                    % Communicate whether threshold has been reached
%                     outString=uint8(sprintf('%d',obj.currKeyID));
                    outString=uint8(sprintf('%d',1));

                    obj.selCounter=0;
                    obj.outputLog.selStart=cat(1,obj.outputLog.selStart,obj.selStartTime);
                    obj.outputLog.selEnd=cat(1,obj.outputLog.selEnd,obj.currTime);
                else
                    outString=uint8('0');
                end
                obj.UDPchannels.udps.step(outString);
%                 fprintf('%s\n',outString)
                
                % If cursor is within target, fills up selection counter
                if obj.inTarget
                    if isfield(obj.clsfr,'mat')
                        feats=cat(2,1,BP);
                        currEst=1./(1+exp(-(feats*obj.clsfr.mat)));
                        obj.selCounter=max(0,obj.selCounter+max(0,currEst-obj.selThreshold));
                    elseif isfield(obj.clsfr,'svm')
                        [~,currEst]=predict(obj.clsfr.svm,BP);
                        currEst=currEst(1);
                        obj.selCounter=max(0,obj.selCounter+max(0,currEst-obj.selThreshold));
                    else
                        currEst=0;
                        obj.selCounter=obj.selCounter+.01;
                    end
                else
                    currEst=0;
                    obj.selCounter=obj.selCounter*(1-obj.selDecay);
                end
%                 fprintf('%0.2f\n',currEst)

%                 % If cursor is within target, fills up selection counter
%                 if obj.inTarget
%                     % Use EEG data when available, otherwise use gaze alone
%                     if isfield(obj.clsfr,'mat')
%                         feats=cat(2,1,BP);
%                         currEst=1./(1+exp(-(feats*obj.clsfr.mat)));
%                         fprintf('%0.2f\n',currEst)
%                         obj.selCounter=max(0,obj.selCounter+max(0,currEst-obj.selThreshold));
%                     else
%                         obj.selCounter=obj.selCounter+.1;
%                     end
%                 else
% %                     obj.selCounter=obj.selCounter*(1-obj.selDecay);
%                 end
                fprintf('%0.2f\n',obj.selCounter)
                
                % Add relevant info to log
                obj.outputLog.feats=cat(1,obj.outputLog.feats,BP);
                obj.outputLog.isTraining=cat(1,obj.outputLog.isTraining,obj.isTraining);
                obj.outputLog.time=cat(1,obj.outputLog.time,obj.currTime);
                obj.outputLog.isInTarget=cat(1,obj.outputLog.isInTarget,obj.inTarget);
                obj.outputLog.selCounter=cat(1,obj.outputLog.selCounter,obj.selCounter);
                obj.outputLog.UDPlog{end+1}=obj.UDPchannels.lastInput;
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
        
        function obj=computeclsfr(obj,BP)
            obj.clsfr.feats{obj.actualTarget}=cat(2,1,BP);
            
            % Define link function
            if ~isfield(obj.clsfr,'mat')
                obj.clsfr.mat=zeros(length(obj.clsfr.feats{obj.actualTarget}'),1);
            end
            
            % Compute output of current classifier
            currEst=1./(1+exp(-(obj.clsfr.feats{obj.actualTarget}*obj.clsfr.mat)));
            
            % Update weights of GLM model
            obj.clsfr.mat=MI_background.updateWeights(obj.clsfr.mat,BP,currEst,obj.inTarget,1e-3);
        end
        
        function obj=toggleTraining(obj)
            if evalin('base','exist(''toggleTraining'',''var'')')&&evalin('base','toggleTraining')
                obj.isTraining=~obj.isTraining;
                assignin('base','toggleTraining',0);
                figure(obj.figHandle)
                if obj.isTraining
                    textHandle=text(-.4,.5,'Training on');
                else
                    textHandle=text(-.4,.5,'Training off');
                end
                set(textHandle,'Color','black','FontSize',28);
            	wait(obj,.5);
                delete(textHandle);
            end
            
            % Set next evaluation time for this function
            obj.timeTriggeredEvents{2}.triggersLog=[obj.timeTriggeredEvents{2}.triggersLog,obj.currTime];
            obj.timeTriggeredEvents{2}.nextTrigger=obj.currTime+.5;
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
        
        function obj=openUDPchannels(obj)
            obj.UDPchannels.udpr=dsp.UDPReceiver('LocalIPPort',obj.UDPchannels.inPort,'ReceiveBufferSize',1);
            obj.UDPchannels.udps=dsp.UDPSender('RemoteIPPort',obj.UDPchannels.outPort);
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
            lbls=zeros(length(obj.outputLog.UDPlog),1);
            for currEl=1:length(obj.outputLog.UDPlog)
                if isempty(obj.outputLog.UDPlog{currEl})
                    lbls(currEl)=5;
                else
                    lbls(currEl)=str2double(obj.outputLog.UDPlog{currEl});
                end
            end
            lbls=double(lbls<5);
            winStarts(winEnds>length(lapData))=[];
            winEnds(winEnds>length(lapData))=[];
            dataWins=zeros(length(winStarts),round(0.4*obj.fs),size(lapData,2));
            for currWin=1:length(winStarts)
                dataWins(currWin,:,:)=lapData(winStarts(currWin)+1:winEnds(currWin),:);
            end
            allFeats=MI_background.preprocData(dataWins);
            lbls=interp1(obj.outputLog.time,lbls,(winStarts+winEnds)*.5/obj.fs,'nearest','extrap');
            
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
        
        function aTarget=get.actualTarget(obj)
                aTarget=double(obj.inTarget)+1;
        end
        
        function inT=get.inTarget(obj) %#ok<MANU>
            inT=1;
        end
        
        function keyID=get.currKeyID(obj)
            if obj.inTarget&&isfield(obj.UDPchannels,'lastInput')&&~isempty(obj.UDPchannels.lastInput)
                keyID=str2double(obj.UDPchannels.lastInput);
            else
                keyID=0;
            end
%             fprintf('%d\n',keyID)
        end
    end
    methods (Static)
        function [freqFeats,timeFeats]=preprocData(dataWins)
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
            
            % Lowpass data
            if size(dataWins,1)>1
                freqFeats=filter(filterData.B,filterData.A,freqFeats);
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
            end
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
                
        function wOut=updateWeights(wIn,feats,E,t,lr)
            % feats new sample
            % E current classifier prediction
            % t true label
            feats=[1,feats];
            wOut=wIn+((lr*(t-E))'*feats)';
        end
        
        function [coords,inKey,keyID]=extractLogs(log)
            coords=zeros(length(log),3);
            inKey=zeros(length(log),1);
            keyID=cell(length(log),1);
            for currLog=1:length(log)
                if isempty(log{currLog})
                    coords(currLog,:)=0;
                    inKey(currLog,:)=0;
                    keyID='';
                else
                    cBracePos=strfind(log{currLog},'}');
                    coords(currLog,:)=str2num(log{currLog}(2:cBracePos-1)); %#ok<ST2NM>
                    inKey(currLog)=numel(strfind(log{currLog},'True'))>0;
                    semiColPos=strfind(log{currLog},';');
                    keyID{currLog}=log{currLog}(semiColPos(end)+1:end);
                end
            end
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
    MI_background.closeExp;
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
MI_background.closeExp;
end