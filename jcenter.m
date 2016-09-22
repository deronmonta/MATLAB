%load rmatrix; %% rmatrix=[rthigh lthigh rshank lshank]; {R1, R2, R3, R4];  {rthigh(med lat) lthigh(med lat) rshank(med lat) lshank(med lat)}; 
close all; clear;

name2=uigetfile('*.trc','select the motion data ');
p=['r+'];    % read and write
eval(['fid2=fopen(' 'name2' ',' 'p' ');']); % use double '' ''  because of in the single ''
for i=1:6
   ii=num2str(i);
   eval(['x' ii '=fgetl(fid2);']);   
end

mov=[fscanf(fid2,'%f',[174,1200])]';     % motion data
fclose(fid2);
data1=mov(:,3:174);
%%% 18 markers

RASIS=data1(:, 72:74)'; LASIS=data1(:, 96:98)'; sacrum=data1(:,27:29)';
Rthigh=data1(:, 75:77)'; Rknee=data1(:,78:80)'; Rshank=data1(:,144:146)'; Rankle=data1(:,84:86)'; Rheel=data1(:,93:95)'; Rtoe=data1(:,87:89)';
Lthigh=data1(:,99:101)'; Lknee=data1(:,102:104)'; Lshank=data1(:,156:158)'; Lankle=data1(:,108:110)'; Lheel=data1(:,117:119)'; Ltoe=data1(:,111:123)';
marker1=data1(:,46:48)'; marker2=data1(:,49:51)'; marker3=data1(:,52:54)';

%%% check major marker position
%figure(1); subplot(121);
%plot(LASIS(1,50:99), LASIS(3,50:99),'b>'); title('left leg'); hold; plot(Lthigh(1,50:99), Lthigh(3,50:99),'r>'); plot(Lknee(1,50:99), Lknee(3,50:99),'g>'); 
%plot(Lshank(1,50:99),Lshank(3,50:99),'r*'); plot(Lankle(1,50:99), Lankle(3,50:99),'g*'); plot(Lheel(1,50:99),Lheel(3,50:99),'bo'); plot(Ltoe(1,50:99), Ltoe(3,50:99),'ro');
%subplot(122);
%plot(RASIS(1,50:99), RASIS(3,50:99),'b>'); title('right leg'); hold; plot(Rthigh(1,50:99), Rthigh(3,50:99),'r>'); plot(Rknee(1,50:99), Rknee(3,50:99),'g>'); 
%plot(Rshank(1,50:99),Rshank(3,50:99),'r*'); plot(Rankle(1,50:99), Rankle(3,50:99),'g*'); plot(Rheel(1,50:99),Rheel(3,50:99),'bo'); plot(Rtoe(1,50:99), Rtoe(3,50:99),'ro');

%%% determine joint center
o_pelvis=0.5*(RASIS+LASIS); %% Pelvis center; 

xp=norm1(RASIS-LASIS);  
zp=norm1(cross(RASIS-sacrum,LASIS-sacrum));
yp=norm1(cross(zp,xp));
Rp=[xp; yp; zp]; %% rotation matrix

%%% Right and left hip joint center
for i=1:length(Rp);
    RL(i)=sqrt(sum((LASIS(:,i)-RASIS(:,i)).^2)); 
    RK=[xp(:,i) yp(:,i) zp(:,i)];  %RK=[R1(1:3,i) R1(4:6,i) R1(7:9,i)];
    rhipJC(:,i)=sacrum(:,i)+RK*[0.344*RL(i); 0.598*RL(i); -0.29*RL(i)];
    lhipJC(:,i)=sacrum(:,i)+RK*[-0.344*RL(i); 0.598*RL(i); -0.29*RL(i)];
end;

load matrix; 

%% Right thigh
zRt=norm1(rhipJC-Rknee);
yRt=norm1(cross(zRt,Rthigh-rhipJC));
xRt=norm1(cross(yRt,zRt));
Rt=[xRt; yRt; zRt]; 
%Left thigh
zLt=norm1(lhipJC-Lknee);
yLt=norm1(cross(zLt,lhipJC-Lthigh));
xLt=norm1(cross(yLt,zLt));
Lt=[xLt; yLt; zLt]; 

for i=1:length(data1);
    rkneeMed(:,i)=[Rt(1:3,i) Rt(4:6,i) Rt(7:9,i)]'*matrix(:,1)+Rknee(:,i); 
    rkneeJC(:,i)=[Rt(1:3,i) Rt(4:6,i) Rt(7:9,i)]'*matrix(:,2)+Rknee(:,i);
    
    lkneeMed(:,i)=[Lt(1:3,i) Lt(4:6,i) Lt(7:9,i)]'*matrix(:,3)+Lknee(:,i); 
    lkneeJC(:,i)=[Lt(1:3,i) Lt(4:6,i) Lt(7:9,i)]'*matrix(:,4)+Lknee(:,i); 
end;
% right shank
zRs=norm1(Rshank-Rankle);
yRs=norm1(cross(zRs,Rshank-rkneeJC));
xRs=norm1(cross(yRs,zRs));
Rs=[xRs; yRs; zRs]; 
% left shank
zLs=norm1(Lshank-Lankle);
yLs=norm1(cross(zLs,lkneeJC-Lshank));
xLs=norm1(cross(yLs,zLs));
Ls=[xLs; yLs; zLs]; 

for i=1:length(data1);
    rankleMed(:,i)=[Rs(1:3,i) Rs(4:6,i) Rs(7:9,i)]'*matrix(:,5)+Rankle(:,i); 
    rankleJC(:,i)=[Rs(1:3,i) Rs(4:6,i) Rs(7:9,i)]'*matrix(:,6)+Rankle(:,i);
    
    lankleMed(:,i)=[Ls(1:3,i) Ls(4:6,i) Ls(7:9,i)]'*matrix(:,7)+Lankle(:,i);
    lankleJC(:,i)=[Ls(1:3,i) Ls(4:6,i) Ls(7:9,i)]'*matrix(:,8)+Lankle(:,i);
end;

figure(2);
plot(o_pelvis(1,:),o_pelvis(3,:),'r'); hold; plot(RASIS(1,:),RASIS(3,:),'o');plot(LASIS(1,:),LASIS(3,:),'go'); 
plot(rhipJC(1,:),rhipJC(3,:),'--'); plot(lhipJC(1,:),lhipJC(3,:),'g--'); plot(rkneeJC(1,:),rkneeJC(3,:),':');plot(lkneeJC(1,:),lkneeJC(3,:),'g:');
plot(rankleJC(1,:),rankleJC(3,:),'-.');plot(lankleJC(1,:),lankleJC(3,:),'g-.');  legend('pelvis','RASIS', 'LASIS','rhipJC','lhipJC','rkneeJC', 'lkneeJC','rankleJC', 'lankleJC');

%r_fp1=[5.6 25.1 -4.5]*10; % Ture center of force plate related to G.C.S.
%r_fp2=[56.5 25.1 -4.5]*10;
figure(1)% Dynamic movement
set(figure(1), 'Color', [0.9 1 1]); 
set(figure(1), 'Position', [30 20 1230 700]);
%plot([r_fp1(3), r_fp1(1)],'b*'); plot([r_fp2(3), r_fp2(1)],'b*'); hold;
for n=1:50;
    rFOOT=[Rtoe(2,3*n) Rtoe(3,3*n)];
    rANKLE=[rankleJC(2,3*n) rankleJC(3,3*n)];
    %rANKLE=[Rheel(2,3*n) Rheel(3,3*n)];
    rKNEE= [rkneeJC(2, 3*n) rkneeJC(3,3*n)];
    rHIP=[rhipJC(2,3*n) rhipJC(3,3*n)];
        
    plot([rFOOT(1) rANKLE(1)], [rFOOT(2) rANKLE(2)],'r'); hold;   %%%% right foot is red
    plot([rANKLE(1) rKNEE(1)], [rANKLE(2) rKNEE(2)],'g') 
    plot([rKNEE(1) rHIP(1)], [rKNEE(2) rHIP(2)]) 
    
    lFOOT=[Ltoe(2,3*n) Ltoe(3,3*n)];
    lANKLE=[lankleJC(2,3*n) lankleJC(3,3*n)];
    %lANKLE=[lheel(1,3*n) lheel(3,3*n)];
    lKNEE= [lkneeJC(2,3*n) lkneeJC(3,3*n)];
    lHIP=[lhipJC(2,3*n) lhipJC(3,3*n)];
        
    plot([lFOOT(1) lANKLE(1)], [lFOOT(2) lANKLE(2)],'k'); %%%% left foot is black
    plot([lANKLE(1) lKNEE(1)], [lANKLE(2) lKNEE(2)],'g') 
    plot([lKNEE(1) lHIP(1)], [lKNEE(2) lHIP(2)]) 
    title('Sagittal plane');axis([650 1300 -400 600])
    M(:,n)=getframe
end;


% segmental centers
rthigh=(0.433*rkneeJC+(1-0.433)*rhipJC); lthigh=(0.433*lkneeJC+(1-0.433)*lhipJC);
rshank=(0.433*rankleJC+(1-0.433)*rkneeJC); lshank=(0.433*lankleJC+(1-0.433)*lkneeJC);
rfoot=(0.429*Rtoe+(1-0.429)*rankleJC); lfoot=(0.429*Ltoe+(1-0.429)*lankleJC);

jointCt=[rhipJC' rkneeJC' rankleJC' lhipJC' lkneeJC' lankleJC'];
segmentCt=[rthigh; rshank; rfoot; lthigh; lshank; lfoot; o_pelvis];


%eval(['save ' name2(length(name2)-7:length(name2)-4) 'jointCt jointCt -ascii -tabs']);
%eval(['save ' name2(length(name2)-7:length(name2)-4) 'segmentCt segmentCt -ascii -tabs']);

save jointCt jointCt -ascii -tabs;
save segmentCt segmentCt -ascii -tabs;

