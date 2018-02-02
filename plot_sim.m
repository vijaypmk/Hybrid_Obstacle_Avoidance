%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: plotSIM
%
% Description: Plots with animation.
%
% Version: 1
% Required files: plotLyapunov.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure(1)
plotLyapunov;
figure(1), hold on, plot(x(:,1),x(:,2),'k')
hold on
% p = plot(x1(1),x2(1),'o','MarkerFaceColor','red');
% p = plot(x1(1),x2(1));
hold off
axis manual
axis([-1 axis_x axis_y-2 axis_y+2])
xlabel('x_1')
ylabel('x_2')

% % for algorithm part of paper
% hold on, plot(x(89,1),x(89,2),'>','Color','r')
% viscircles([x(89,1) x(89,2)],co,'Color','r');
% text(x(89,1)-0.2,x(89,2)-0.15,'2');
% plot(x(224,1),x(224,2),'>','Color','r')
% viscircles([x(224,1) x(224,2)],co,'Color','r');
% text(x(224,1)+0.1,x(224,2)-0.15,'3');

hold on, plot(x1t,x2t,'x')
text(x1t+0.08,x2t-0.1,'x_t');
hold on, plot(x0(1),x0(2),'>','Color','r')
text(x0(1)-0.25,x0(2)+0.15,'x_o');
% text(x0(1)-0.2,x0(2)-0.15,'1');
for i = 1:num_obs
     viscircles([r1(i) r2(i)], bo,'LineStyle','--');
    hold on, plot(r1(i), r2(i), '.','Color','k');
end
viscircles([x0(1) x0(2)],co,'Color','r');

% text(r1(1)-0.23,r2(1)+0.15,'O_1');
% text(r1(2)-0.23,r2(2)+0.15,'O_2');
% text(r1(3)-0.23,r2(3)+0.15,'O_3');
% text(r1(4)-0.23,r2(4)+0.15,'O_4');

% % animation
% h = animatedline;
% % filename = 'testnew51.gif';
% F(150) = struct('cdata',[],'colormap',[]);
% i = 1;
% for k = 20:2:500
% %     p.XData = x1(k);
% %     p.YData = x2(k);
% %     p.XData = x(k,1);
% %     p.YData = x(k,2);
% %     if x(k,1) == x1t && x(k,2) == x2t
% %         break;
% %     end
%     if k > 1500
%         k = k + 100;
%     end
%     addpoints(h, x(k,1), x(k,2));
% %     head = plot(x(k,1), x(k,2));
% %     viscircles([x1(k) x2(k)],co,'Color','r');
%     drawnow
% %     hold on
% %     plotLyapunov;
% %     hold off
%     F(i) = getframe(gcf);
%     i = i + 1;
% %     pause(0.01);
% %     delete(head);
% %     frame = getframe(1);
% %       im = frame2im(frame);
% %       [imind,cm] = rgb2ind(im,256);
% %       if k == 1;
% %           imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
% %       else
% %           imwrite(imind,cm,filename,'gif','WriteMode','append');
% %       end
% end

% video = VideoWriter('traj_4','MPEG-4');
% open(video)
% writeVideo(video, F);
% close(video)
% hold on, plot(0.8445,0,'o')
% hold on, plot(1+0.1,0,'o')
% hold on, plot(1,0.1,'o')
% hold on, plot(1,-0.1,'o')
% text(3,-0.3,' x_t')

% break

% % for paper
% figure(2),clf
% 
% plotLyapunov1
% hold on
% y1 = linspace(-1,1,10);
% y2 = -y1+3.1-0.3;
% plot(y1,y2)
% y1 = linspace(1,4,10);
% y2 = y1+1.1-0.3;
% hold on, plot(y1,y2)
% y1 = linspace(1-0.1,1,10);
% y2 = -y1+3.1-0.3;
% plot(y1,y2)
% y1 = linspace(1,1.1,10);
% y2 = y1+1.1-0.3;
% plot(y1,y2)
% % % axis([-1 4 -2 2])
% % % 
% y1 = linspace(-1,1,10);
% y2 = y1+0.9+0.3;
% plot(y1,y2,':')
% y1 = linspace(1,4,10);
% y2 = -y1+2.9+0.3;
% plot(y1,y2,':')
% y1 = linspace(1-0.1,1,10);
% y2 = y1+0.9+0.3;
% plot(y1,y2)
% y1 = linspace(1,1.1,10);
% y2 = -y1+2.9+0.3;
% plot(y1,y2)
% 
% xlabel('x_1')
% ylabel('x_2')
% 
% hold on, plot(3,0,'x')
% hold on, plot(x1(1),x2(1),'>')
% %hold on, plot(1-0.1,0,'o')
% %hold on, plot(1+0.1,0,'o')
% %hold on, plot(1,0.1,'o')
% %hold on, plot(1,-0.1,'o')
% text(3,-0.2,' x_t')
% 
% text(2.4,1,'W_2')
% text(-0.6,1,'W_2')
% 
% text(-0.6,3,'W_1')
% text(2.4,3,'W_1')
% 
% text(-0.2,-0.25,'c')
% text(0.7,-0.3,'r')

% V plot
% plotLyapunov2;