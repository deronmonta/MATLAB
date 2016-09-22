
vid=videoinput('IMG_5714.MOV',1);
preview(vid);
while 1
im=getsnapshot(vid);
[im_yellow num]=green(im);
[B,L] = bwboundaries(im_yellow,'noholes');
imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
boundary = B{k};
plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
end     
x=boundary(:,1);
y=boundary(:,2);
abc = [x y ones(length(x),1)] \ -(x.^2+y.^2);
a = abc(1); b = abc(2); c = abc(3);
xc = -a/2;
yc = -b/2;
radius  =  sqrt((xc^2+yc^2)-c)
imshow(im);
hold on
plot(yc,xc,'yx','LineWidth',2);
theta = 0:0.01:2*pi;
Xfit = radius*cos(theta) + xc;
Yfit = radius*sin(theta) + yc;
plot(Yfit, Xfit);
message = sprintf('The estimated radius is %2.3f pixels', radius);
text(15,15,message,'Color','y','FontWeight','bold');
end