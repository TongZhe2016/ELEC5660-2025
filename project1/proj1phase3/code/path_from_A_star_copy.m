function Optimal_path_copy = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1;
    
    % Main structure in the A* search =====================================================

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    queue = [];  

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording whether a node is expanded (popped from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording the parent of each node
    parents = zeros(MAX_X,MAX_Y,MAX_Z, 3);
    
    
    %Start your code here ==================================================================
    % TODO
end
