function plot_3d_obj(obj,init,tar)
    scatter3(init(1),init(2),init(3),20,'filled','MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0]); hold on;
    scatter3(tar(1),tar(2),tar(3),20,'filled','MarkerEdgeColor',[0 1 0],'MarkerFaceColor',[0 1 0]);
    rotm = eul2rotm([obj(9), obj(8), -obj(7)]);
    oPt = obj(1:3)';
    xPt = (obj(1:3)' + (rotm * [3,0,0]')') ;
    yPt = (obj(1:3)' + (rotm * [0,3,0]')');
    zPt = (obj(1:3)' + (rotm * [0,0,3]')');
    line([oPt(1) xPt(1)],[oPt(2) xPt(2)],[oPt(3) xPt(3)],'Color','red','LineWidth',3);
    line([oPt(1) yPt(1)],[oPt(2) yPt(2)],[oPt(3) yPt(3)],'Color','green','LineWidth',3);
    line([oPt(1) zPt(1)],[oPt(2) zPt(2)],[oPt(3) zPt(3)],'Color','blue','LineWidth',3);
    axis equal;
    xlim([-20,20]); xlabel('x');
    ylim([-20,20]); ylabel('y');
    zlim([0,10]); zlabel('z');
    hold off;
end
