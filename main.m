%%Siddharth Dutta, CMPEN/EE 454
profile on

%load('Subject4-Session3-Take4_mocapJoints.mat');
load('vue2CalibInfo.mat');
load('vue4CalibInfo.mat');
load('Subject4-Session3-Take4_mocapJoints.mat');
s=load('vue4CalibInfo.mat');

%%Creating an empty arry containing zeros for each of the 12 joints
RShoulder = zeros(1,26214);
RElbow = zeros(1,26214);
RWrist = zeros(1,26214);
LShoulder = zeros(1,26214);
LElbow = zeros(1,26214);
LWrist = zeros(1,26214);
RHip = zeros(1,26214);
RKnee = zeros(1,26214);
RAnkle = zeros(1,26214);
LHip = zeros(1,26214);
LKnee = zeros(1,26214);
LAnkle = zeros(1,26214);
err = zeros(1,26214);

for mocapFnum = 1:26214
            x = mocapJoints(mocapFnum,:,1);  % array of 12 X coordinates
            y = mocapJoints(mocapFnum,:,2); % Y coordinates
            z = mocapJoints(mocapFnum,:,3); % Z coordinates
            conf = mocapJoints(mocapFnum,:,4); % confidence values
            %creating a 3x12 matrix for each camera to contain x,y,z
            %coordiates for 12 joints
            cam2 = zeros(3,12);
            cam4 = zeros(3,12);
            %Projecting 3D points to 2D pixel locations         
            for i = 1:12
                jointCor = [x(i);y(i);z(i);1];
                paramValue = vue2.Kmat * vue2.Pmat * jointCor;
                paramValue = paramValue/paramValue(3);
                cam2(:,i) = paramValue;
                paramValue = vue4.Kmat * vue4.Pmat * jointCor;
                paramValue = paramValue/paramValue(3);
                cam4(:,i) = paramValue; 
            end

            p = zeros(3,12);
            %% Triangulation of 2D to 3D            
            %Vue2 position = [-4.450060852085680e+03,5.557920351620290e+03,1.949062726802550e+03]
            %Vue4 position = [4.423562978688680e+03,5.490360440799710e+03,1.889016742748900e+03]
            %Camera postion difference = [8.873623830774366e+03,-67.559910820575170,-60.045984053648226]
            
            for i = 1:12
                cam2proj = vue2.Kmat\cam2(:,i);     
                cam4proj = vue4.Kmat\cam4(:,i);
                A(:,1) = (vue2.Rmat.')* cam2proj; 
                A(:,2) = -((vue4.Rmat.') * cam4proj);
                u1xu2 = cross((vue2.Rmat.')* cam2proj,(vue4.Rmat.')* cam4proj); 
                uhat = u1xu2/abs(u1xu2); % using uhat = (u1 x u2)/abs(u1 x u2)
                A(:,3) = uhat(:,3); 
                a = A\([8.873623830774366e+03;-67.559910820575170;-60.045984053648226]);
                p1 = ([-4.450060852085682e+03;5.557920351620292e+03;1.949062726802550e+03] + (a(1)*(vue2.Rmat.')* cam2proj))/2;  %% AFTER SOLVING FOR UNKNOWS WE USE do P1+P2/2 
                p2 = ([4.423562978688685e+03;5.490360440799717e+03;1.889016742748902e+03] + (a(2)*(vue4.Rmat.')* cam4proj))/2;  %% AS DIRECTED IN FORMULA
                p(:,i) = p1+p2;    
            end

            % The following calcutes the L value or the Euclidean distance
            lnxt = 0;
            L = zeros(1,12);  
            for i = 1:12
                L(:,i) = sqrt((x(i) - p(1,i)).^2 + (y(i) - p(2,i)).^2 + (z(i) - p(3,i)).^2);   
                lnxt = L(i) + lnxt;
            end
            %%Checking for NaN values in L and falgging it since
            %%program goes into an inf loop otherwise 
            flag = 0;
            for x = 1:12
                if isnan(L(:,x)) == 1
                    flag = 1;
                    break
                end
            end
            
            err(mocapFnum) = lnxt/12;
            %%L^2 for the 12 joints
            RShoulder(mocapFnum) = L(1);
            RElbow(mocapFnum) = L(2);
            RWrist(mocapFnum) = L(3);
            LShoulder(mocapFnum) = L(4);
            LElbow(mocapFnum) = L(5);
            LWrist(mocapFnum) = L(6);
            RHip(mocapFnum) = L(7);
            RKnee(mocapFnum) = L(8);
            RAnkle(mocapFnum) = L(9);
            LHip(mocapFnum) = L(10);
            LKnee(mocapFnum) = L(11);
            LAnkle(mocapFnum) = L(12);
end


%Statistics for the 12 joints
%Error calculations 
%Minimum error 
mat = [min(RShoulder(:)),min(RElbow(:)),min(RWrist(:)),min(LShoulder(:)),min(LElbow(:)),min(LWrist(:)),min(RHip(:)),min(RKnee(:)),min(RAnkle(:)),min(LHip(:)),min(LKnee(:)),min(LAnkle(:))];
disp(mat)

%Maximum error
mat2 = [max(RShoulder(:)),max(RElbow(:)),max(RWrist(:)),max(LShoulder(:)),max(LElbow(:)),max(LWrist(:)),max(RHip(:)),max(RKnee(:)),max(RAnkle(:)),max(LHip(:)),max(LKnee(:)),max(LAnkle(:))];
disp("Maximum error of Joints 1-12");
disp(mat2)

%Mean error
mat3 = [mean(RShoulder(:)),mean(RElbow(:)),mean(RWrist(:)),mean(LShoulder(:)),mean(LElbow(:)),mean(LWrist(:)),mean(RHip(:)),mean(RKnee(:)),mean(RAnkle(:)),mean(LHip(:)),mean(LKnee(:)),mean(LAnkle(:))];
disp("Mean error of Joints 1-12");
disp(mat3)

%Median error
mat4 = [median(RShoulder(:)),median(RElbow(:)),median(RWrist(:)),median(LShoulder(:)),median(LElbow(:)),median(LWrist(:)),median(RHip(:)),median(RKnee(:)),median(RAnkle(:)),median(LHip(:)),median(LKnee(:)),median(LAnkle(:))];
disp("Median error of Joints 1-12");
disp(mat4)

%Standard deviation error
mat5 = [std(RShoulder(:)),std(RElbow(:)),std(RWrist(:)),std(LShoulder(:)),std(LElbow(:)),std(LWrist(:)),std(RHip(:)),std(RKnee(:)),std(RAnkle(:)),std(LHip(:)),std(LKnee(:)),std(LAnkle(:))];
disp("Standard deviation of error of Joints 1-12");
disp(mat5)

% setting a specific frame to be plotted for
reqFrameNum = 21000; 

x1 = mocapJoints(reqFrameNum,:,1);  % array of 12 X coordinates
y1 = mocapJoints(reqFrameNum,:,2); % Y coordinates
z1 = mocapJoints(reqFrameNum,:,3); % Z coordinates
conf = mocapJoints(reqFrameNum,:,4); % confidence values
cam2 = zeros(3,12);
cam4 = zeros(3,12);
%%
for i = 1:12
    jointCor = [x1(i);y1(i);z1(i);1];
    paramValue = vue2.Kmat * vue2.Pmat * jointCor;
    paramValue = paramValue/paramValue(3);
    cam2(:,i) = paramValue;
    paramValue = vue4.Kmat * vue4.Pmat * jointCor;
    paramValue = paramValue/paramValue(3);
    cam4(:,i) = paramValue; 
end

%%Plots
%Vue 2
figure()
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
vue2video = VideoReader(filenamevue2mp4);
vue2video.CurrentTime = (reqFrameNum-1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);
image(vid2Frame);

title('Vue 2 PLOT');
hold on

%Plotting joint locations on the frame
plot(cam2(1,1),cam2(2,1), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,2),cam2(2,2), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,3),cam2(2,3), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,4),cam2(2,4), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,5),cam2(2,5), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,6),cam2(2,6), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,7),cam2(2,7), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,8),cam2(2,8), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,9),cam2(2,9), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,10),cam2(2,10), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,11),cam2(2,11), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam2(1,12),cam2(2,12), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot((cam2(1,1)+ cam2(1,4))/2,(cam2(2,1)+ cam2(2,4))/2, 'r+', 'MarkerSize',5, 'LineWidth', 2);%% SHOULDER MID POINT CALC
plot((cam2(1,7)+ cam2(1,10))/2,(cam2(2,7)+ cam2(2,10))/2, 'r+', 'MarkerSize',5, 'LineWidth', 2);%% HIP MIDPOINT CALC

%Joining the dots for pairs 
plot([cam2(1,1) cam2(1,4)], [cam2(2,1) cam2(2,4)], 'c');
plot([cam2(1,1) cam2(1,2)], [cam2(2,1) cam2(2,2)], 'c');
plot([cam2(1,2) cam2(1,3)], [cam2(2,2) cam2(2,3)], 'c');
plot([cam2(1,4) cam2(1,5)], [cam2(2,4) cam2(2,5)], 'c');
plot([cam2(1,5) cam2(1,6)], [cam2(2,5) cam2(2,6)], 'c');
plot([cam2(1,7) cam2(1,10)], [cam2(2,7) cam2(2,10)], 'c');
plot([cam2(1,7) cam2(1,8)], [cam2(2,7) cam2(2,8)], 'c');
plot([cam2(1,8) cam2(1,9)], [cam2(2,8) cam2(2,9)], 'c');
plot([cam2(1,10) cam2(1,11)], [cam2(2,10) cam2(2,11)], 'c');
plot([cam2(1,11) cam2(1,12)], [cam2(2,11) cam2(2,12)], 'c');
plot([cam2(1,1) cam2(1,7)], [cam2(2,1) cam2(2,7)], 'c');
plot([cam2(1,4) cam2(1,10)], [cam2(2,4) cam2(2,10)], 'c');
plot([(cam2(1,1)+cam2(1,4))/2 ; (cam2(1,7)+cam2(1,10))/2], [(cam2(2,1)+cam2(2,4))/2 ; (cam2(2,7)+cam2(2,10))/2], 'c');%%SHOULDER TO HIP LINE JOINED



%%Plot for joint error 

figure();

i = 1:26214;
plot(i, err, 'b', 'MarkerSize',0.1, 'LineWidth', 0.1);
axis([0 27000 0 10.^(-11.5)]);
title('average error/frame number');
xlabel('Frame');
ylabel('Average Error');


%Vue 4
figure()
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';
vue4video = VideoReader(filenamevue4mp4);
vue4video.CurrentTime = (reqFrameNum-1)*(50/100)/vue4video.FrameRate;
vid4Frame = readFrame(vue4video);
image(vid4Frame);
title('Vue4 PLOT');

hold on
%%Plotting joint locations on the frame

plot(cam4(1,1),cam4(2,1), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,2),cam4(2,2), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,3),cam4(2,3), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,4),cam4(2,4), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,5),cam4(2,5), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,6),cam4(2,6), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,7),cam4(2,7), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,8),cam4(2,8), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,9),cam4(2,9), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,10),cam4(2,10), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,11),cam4(2,11), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot(cam4(1,12),cam4(2,12), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot((cam4(1,1)+ cam4(1,4))/2,(cam4(2,1)+ cam4(2,4))/2, 'r+', 'MarkerSize',5, 'LineWidth', 2); %%
plot((cam4(1,7)+ cam4(1,10))/2,(cam4(2,7)+ cam4(2,10))/2, 'r+', 'MarkerSize',5, 'LineWidth', 2); %%

%%Joining the dots for pairs 
 
plot([cam4(1,1) cam4(1,4)], [cam4(2,1) cam4(2,4)], 'c');
plot([cam4(1,1) cam4(1,2)], [cam4(2,1) cam4(2,2)], 'c');
plot([cam4(1,2) cam4(1,3)], [cam4(2,2) cam4(2,3)], 'c');
plot([cam4(1,4) cam4(1,5)], [cam4(2,4) cam4(2,5)], 'c');
plot([cam4(1,5) cam4(1,6)], [cam4(2,5) cam4(2,6)], 'c');
plot([cam4(1,7) cam4(1,10)], [cam4(2,7) cam4(2,10)], 'c');
plot([cam4(1,7) cam4(1,8)], [cam4(2,7) cam4(2,8)], 'c');
plot([cam4(1,8) cam4(1,9)], [cam4(2,8) cam4(2,9)], 'c');
plot([cam4(1,10) cam4(1,11)], [cam4(2,10) cam4(2,11)], 'c');
plot([cam4(1,11) cam4(1,12)], [cam4(2,11) cam4(2,12)], 'c');
plot([cam4(1,1) cam4(1,7)], [cam4(2,1) cam4(2,7)], 'c');
plot([cam4(1,4) cam4(1,10)], [cam4(2,4) cam4(2,10)], 'c');
plot([(cam4(1,1)+cam4(1,4))/2 ; (cam4(1,7)+cam4(1,10))/2], [(cam4(2,1)+cam4(2,4))/2 ; (cam4(2,7)+cam4(2,10))/2], 'c');%%

%3D plot coordinates 
x1 = mocapJoints(reqFrameNum,:,1);  % array of 12 X coordinates
y1 = mocapJoints(reqFrameNum,:,2); % Y coordinates
z1 = mocapJoints(reqFrameNum,:,3); % Z coordinates
conf = mocapJoints(reqFrameNum,:,4); % confidence values

for i = 1:12
    cam2proj = vue2.Kmat\cam2(:,i);     
    cam4proj = vue4.Kmat\cam4(:,i);
    A(:,1) = (vue2.Rmat.')* cam2proj; 
    A(:,2) = -((vue4.Rmat.') * cam4proj);
    u1xu2 = cross((vue2.Rmat.')* cam2proj,(vue4.Rmat.')* cam4proj); 
    uhat = u1xu2/abs(u1xu2); % using uhat = (u1 x u2)/abs(u1 x u2)
    A(:,3) = uhat(:,3); 
    a = A\([8.873623830774366e+03;-67.559910820575170;-60.045984053648226]);
    p1 = ([-4.450060852085682e+03;5.557920351620292e+03;1.949062726802550e+03] + (a(1)*(vue2.Rmat.')* cam2proj))/2;  %% AFTER SOLVING FOR UNKNOWS WE USE do P1+P2/2 
    p2 = ([4.423562978688685e+03;5.490360440799717e+03;1.889016742748902e+03] + (a(2)*(vue4.Rmat.')* cam4proj))/2;  %% AS DIRECTED IN FORMULA
    p(:,i) = p1+p2;    
end


%% 3D plot
figure()

title('3D PLOT');
grid on
hold on
%%Plotting joint locations on the frame

plot3(p(1,1),p(2,1),p(3,1), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,2),p(2,2),p(3,2), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,3),p(2,3),p(3,3), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,4),p(2,4),p(3,4), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,5),p(2,5),p(3,5), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,6),p(2,6),p(3,6), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,7),p(2,7),p(3,7), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,8),p(2,8),p(3,8), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,9),p(2,9),p(3,9), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,10),p(2,10),p(3,10), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,11),p(2,11),p(3,11), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3(p(1,12),p(2,12),p(3,12), 'r+', 'MarkerSize',5, 'LineWidth', 2);
plot3((p(1,1)+ p(1,4))/2,(p(2,1)+ p(2,4))/2, (p(3,1)+ p(3,4))/2,  'r+', 'MarkerSize',5, 'LineWidth', 2); %%
plot3((p(1,7)+ p(1,10))/2,(p(2,7)+ p(2,10))/2, (p(3,7)+ p(3,10))/2, 'r+', 'MarkerSize',5, 'LineWidth', 2); %%

%%Joining the dots for pairs 

plot3([p(1,1) p(1,4)], [p(2,1) p(2,4)], [p(3,1) p(3,4)], 'c');
plot3([p(1,1) p(1,2)], [p(2,1) p(2,2)], [p(3,1) p(3,2)], 'c');
plot3([p(1,2) p(1,3)], [p(2,2) p(2,3)], [p(3,2) p(3,3)], 'c');
plot3([p(1,4) p(1,5)], [p(2,4) p(2,5)], [p(3,4) p(3,5)], 'c');
plot3([p(1,5) p(1,6)], [p(2,5) p(2,6)], [p(3,5) p(3,6)], 'c');
plot3([p(1,7) p(1,10)], [p(2,7) p(2,10)], [p(3,7) p(3,10)], 'c');
plot3([p(1,7) p(1,8)], [p(2,7) p(2,8)], [p(3,7) p(3,8)], 'c');
plot3([p(1,8) p(1,9)], [p(2,8) p(2,9)], [p(3,8) p(3,9)], 'c');
plot3([p(1,10) p(1,11)], [p(2,10) p(2,11)], [p(3,10) p(3,11)], 'c');
plot3([p(1,11) p(1,12)], [p(2,11) p(2,12)], [p(3,11) p(3,12)], 'c');
plot3([p(1,1) p(1,7)], [p(2,1) p(2,7)], [p(3,1) p(3,7)], 'c');
plot3([p(1,4) p(1,10)], [p(2,4) p(2,10)], [p(3,4) p(3,10)], 'c');
plot3([(p(1,1)+p(1,4))/2 ; (p(1,7)+p(1,10))/2], [(p(2,1)+p(2,4))/2 ; (p(2,7)+p(2,10))/2], [(p(3,1)+p(3,4))/2 ; (p(3,7)+p(3,10))/2], 'c');%%


profile viewer