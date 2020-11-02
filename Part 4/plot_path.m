clear all
clc 
close all

load('WP.mat')

figure(gcf)

hold on

siz=size(WP);

for ii=1:(siz(2)-1)   

plot([WP(2,ii), WP(2,ii+1)], [WP(1,ii), WP(1,ii+1)], 'r-x')

end

%plot(y,x,'linewidth',2); axis('equal')

title('North-East positions (m)');