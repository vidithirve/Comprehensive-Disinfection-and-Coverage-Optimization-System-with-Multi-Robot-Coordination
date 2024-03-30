% Given map of environment - can be changed
occupancyMap = [
    1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 0 0 0 1 0 0 0 0 0 0 0 0 1;
    1 0 0 0 1 0 0 0 0 1 0 0 0 1;
    1 1 1 0 0 0 0 0 0 1 0 0 0 1;
    1 0 0 0 0 1 0 0 0 1 0 0 0 1;
    1 0 0 0 0 0 0 0 0 0 0 0 0 1;
    1 0 0 0 0 0 0 0 0 0 0 0 0 1;
    1 1 1 1 1 0 0 0 0 0 0 1 0 1;
    1 0 0 0 0 0 0 0 0 0 0 0 0 1;
    1 0 1 0 0 0 0 0 0 0 0 0 0 1;
    1 0 0 0 0 0 0 0 0 0 0 0 0 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1;];

% Initial parameters
numRobots = 5; % Number of robots - can be changed
itNum = 1;

% Initial parameters for sake of plots
individualVisitedNodes = zeros(size(occupancyMap, 1), size(occupancyMap, 2), numRobots);
individualCoveragePercentages = zeros(0, numRobots);  % Rows: iterations, Columns: robots
robotDistances = cell(numRobots, numRobots-1);
trajectories = cell(numRobots, 1);
visitedNodes = zeros(size(occupancyMap));
coveragePercentages = [];

% Initial robot position on available areas of map
[freeRows, freeCols] = find(occupancyMap == 0);
indices = randperm(length(freeRows), numRobots);
robotPositions = [freeRows(indices), freeCols(indices)];

%% Model iterations - stop at 50
while itNum < 50  
    currentPercentages = zeros(1, numRobots);

    for i = 1:numRobots
        % Positions of all the other robots
        otherRobotsPositions = robotPositions;
        otherRobotsPositions(i,:) = [];

        % Moving strategy
        robotPositions(i, :) = moveRobotCoverageAware(occupancyMap, robotPositions(i, :), visitedNodes, otherRobotsPositions);

        % Calculate current coverage percentage
        totalFreeCells = sum(occupancyMap(:) == 0);
        coveredCells = sum(visitedNodes(:) > 0);
        currentCoveragePercentage = (coveredCells / totalFreeCells) * 100;
        individualVisitedNodes(robotPositions(i, 1), robotPositions(i, 2), i) = 1;
        coveredByRobot = sum(individualVisitedNodes(:, :, i), 'all');
        currentPercentages(i) = (coveredByRobot / totalFreeCells) * 100;

        % Update parameters
        individualVisitedNodes(robotPositions(i, 1), robotPositions(i, 2), i) = 1;
        trajectories{i} = [trajectories{i}; robotPositions(i, :)];
        visitedNodes(robotPositions(i, 1), robotPositions(i, 2)) = 1;
    end

    % Update parameters again
    individualCoveragePercentages = [individualCoveragePercentages; currentPercentages];
    coveragePercentages = [coveragePercentages, currentCoveragePercentage];

    % Calculating a storing difference is robot distances
    for i = 1:numRobots
        counter = 1;
        for j = 1:numRobots
            if i ~= j
                distance = norm(robotPositions(i, :) - robotPositions(j, :));
                robotDistances{i, counter} = [robotDistances{i, counter}, distance];
                counter = counter + 1;
            end
        end
    end

    % Calculating a storing difference between last 2 positions
    for i = 1:numRobots
        if size(trajectories{i}, 1) > 1
            dist = norm(trajectories{i}(end, :) - trajectories{i}(end-1, :));
        end
    end

    % Visualize
    visualizeCoverage(occupancyMap, robotPositions, trajectories, visitedNodes);
    itNum= itNum +1;
end

%% Plots
% Coverage Efficiency Over Time plot
figure;
plot(coveragePercentages);
xlabel('Time');
ylabel('Coverage Percentage (%)');
title('Coverage Efficiency Over Time');
subtitle('Fundamental Model');

% Minimum Euclidean Distance Over Time plot
minDisBtnRbts = [robotDistances{1, 1}; robotDistances{1, 2}; robotDistances{1, 3};
    robotDistances{1, 4}; robotDistances{2, 2}; robotDistances{2, 3};
    robotDistances{2, 4}; robotDistances{3, 3}; robotDistances{3, 4};
    robotDistances{4, 4};];
minDisBtnRbts= min(minDisBtnRbts);
figure;
plot(minDisBtnRbts);
ylim([0, inf]);
xlabel('Time');
ylabel('Minimum Distance Inbetween Robots');
title('Minimum Euclidean Distance Over Time');
subtitle('Fundamental Model');

% Individual Robot Coverage Efficiency Over Time plot
figure;
hold on;
robotColors = {'r', 'g', 'b', [0.9290 0.6940 0.1250], 'm'};
for i = 1:numRobots
    colorIndex = mod(i-1, length(robotColors)) + 1;
    robotColor = robotColors{colorIndex};
    plot(individualCoveragePercentages(:, i), 'Color', robotColor);
end
hold off;
xlabel('Time');
ylabel('Coverage Percentage (%)');
title('Individual Robot Coverage Efficiency Over Time');
subtitle('Fundamental Model')
legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5');


function newPosition = moveRobotCoverageAware(occupancyMap, currentPosition, visitedNodes, otherRobotsPositions)
% Possible new positions
currentRow = currentPosition(1);
currentCol = currentPosition(2);
moves = [0, -1; -1, 0; 0, 1; 1, 0];
possiblePositions = [currentRow + moves(:, 1), currentCol + moves(:, 2)];

% Filtering out map boundaries, obstacles and visited nodes
validMapPositions = possiblePositions(:, 1) >= 1 & possiblePositions(:, 1) <= size(occupancyMap, 1) & ...
    possiblePositions(:, 2) >= 1 & possiblePositions(:, 2) <= size(occupancyMap, 2);
validPositions = validMapPositions & (occupancyMap(sub2ind(size(occupancyMap), possiblePositions(:, 1), possiblePositions(:, 2))) == 0) & ...
    (visitedNodes(sub2ind(size(visitedNodes), possiblePositions(:, 1), possiblePositions(:, 2))) == 0);

% Logic for if possible moves have been covered or not
if ~any(validPositions)
    % Only filer out walls, obstables, and other robots
    validPositions = validMapPositions & (occupancyMap(sub2ind(size(occupancyMap), possiblePositions(:, 1), possiblePositions(:, 2))) == 0);
    validPositions = validPositions & ~ismember(possiblePositions, otherRobotsPositions, 'rows');
    validMoves = possiblePositions(validPositions, :);

    % Finding nearest unexplored cell and its index (row and column)
    unexploredCells = (~visitedNodes) & (~occupancyMap);
    [unexploredRows, unexploredCols] = find(unexploredCells == 1);
    distancesPt1 = (unexploredRows - ones(size(unexploredRows))*currentRow).^2 + (unexploredCols - ones(size(unexploredCols))*currentCol).^2;
    distances = sqrt(distancesPt1);
    [~, index] = min(distances);
    chosenUnexploredRow = unexploredRows(index);
    chosenUnexploredCol = unexploredCols(index);

    if isempty(chosenUnexploredRow)
        newPosition = currentPosition;
    else
        % Moving to possible move nearest to chosen unexplored cell
        distance2 = sqrt((validMoves(:, 1) - chosenUnexploredRow).^2 + (validMoves(:, 2) - chosenUnexploredCol).^2);
        [~, index2] = min(distance2);
        newPosition = validMoves(index2, :);
    end

else
    % Moving to next valid move in the order of moves
    validMoves = possiblePositions(validPositions, :);
    distances = sqrt((validMoves(:, 1) - currentRow).^2 + (validMoves(:, 2) - currentCol).^2);
    [~, index] = min(distances);
    newPosition = validMoves(index, :);
end

end

function visualizeCoverage(occupancyMap, robotPositions, trajectories, visitedNodes)
% Subplot 1
clf;
subplot(2, 2, [1, 3]);
imagesc(occupancyMap);
hold on;
robotColors = {'r', 'g', 'b', [0.9290 0.6940 0.1250], 'm'}; % mix of standard colors and RGB
for i = 1:size(robotPositions, 1)
    colorIndex = mod(i-1, length(robotColors)) + 1;
    robotColor = robotColors{colorIndex};
    plot(robotPositions(i, 2), robotPositions(i, 1), '.', 'MarkerSize', 30, 'Color', robotColor);
    plot(trajectories{i}(:, 2), trajectories{i}(:, 1), '-', 'LineWidth', 5, 'Color', robotColor);
end
axis equal;
title('Multi-Robot Coverage');
subtitle('Market-Potential Model');

% Subplot 2
subplot(2, 2, 2);
imagesc(visitedNodes);
colormap([1 1 1; 0 0 0]);
axis equal;
title('Visited Nodes');
subtitle('Market-Potential Model');
drawnow;
end