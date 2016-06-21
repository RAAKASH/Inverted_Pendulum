fclose(instrfind); % Comment this out if using coded for the first time , then uncomment it
clear all;
 clc
close all;
 s=serial('COM5','BaudRate',115200);
fopen(s);
readData = fscanf(s)   % new line
readData = fscanf(s)   % Send any character to begin DMP programming and demo: 

writedata = 500; %0x01F4
fwrite(s,writedata); %write data
i = 1; % Index

readData = fscanf(s)  % Enabling DMP
readData = fscanf(s)  % Enabling interrupt detection (Arduino external interrupt 0)...
readData = fscanf(s)  % DMP ready! Waiting for first interrupt...
tic;
t=0.3;

%% Note: it takes approximately 18s to stabilize
   while(toc<50) %read 2 lines of data
    t1 =toc;
    text = fscanf(s);
    
    Data(i) =  str2num(text);
    DATA(i,:) = [t1,Data(i)];
    if(i~=1)
    DATA1(i,2) = (DATA(i,2) + DATA(i-1,2))/2;  % averaging the output
    end
    
    
    % toc
% DATA(i,:)

    i=i+1;
    
%   subplot(1,1,1)
%  grid on;
%   axis([0 500000 -100 50]);
%  hold on;
%  plot(500*(1:(i-1)),Data ,'b-o');
%  title('Yaw');
%   t2 = toc;
%   if(t>(t2-t1))
%  pause(t - (t2-t1));
%   end
   end
   
   %% Plotting
 DATA1(:,1) = DATA(:,1);
 subplot(2,1,1)
 xlabel('Time');
 ylabel('Angle in degrees');
 plot(DATA(:,1),DATA(:,2));
 title('raw output');
 subplot(2,1,2)
 ylabel('Angle in degrees');
 xlabel('Time');
 plot(DATA1(:,1),DATA1(:,2));
 title('Avg output');
 %%
 fclose(s);
 delete(s);