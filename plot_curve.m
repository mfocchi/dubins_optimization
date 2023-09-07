function  plot_curve(solution, solution_constr, p0, pf)

%initial
plot(p0(1), p0(2) , 'Marker', '.', 'Color','g', 'MarkerSize',60) ; hold on;
% final point
plot(pf(1), pf(2) , 'Marker', '.', 'Color','r', 'MarkerSize',60) ;


min_x = min(min(solution.p(1,:)), pf(1))-3 ;
max_x = max(max(solution.p(1,:)), pf(1))+3 ;
min_y = min(min(solution.p(2,:)), pf(2))-3 ;
max_y = max(max(solution.p(2,:)), pf(2)) +3 ;

set(gca,'XLim',[min_x max_x])
set(gca,'YLim',[min_y max_y])

color_input = 'r'; 

% actual traj
plot(solution.p(1,:), solution.p(2,:), 'Color', color_input ) ;

% discrete traj
plot(solution_constr.p(1,:), solution_constr.p(2,:), 'o', 'Color', color_input ) ;



%arrow3d_points(p0,pf + solution.Fleg*force_scale,'color','r');grid on;hold on;
grid on;

xlabel('X');
ylabel('Y');



end