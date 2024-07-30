% Load video
input_video = "..\Downloads\06192movie5.mp4"; % input video path
videoFile = input_video;
video = VideoReader(videoFile);

% Define the full path to the output processed video file
outputVideoFile = ['..\Desktop\processed_' input_video];
  % Update with your actual path
videoWriter = VideoWriter(outputVideoFile, 'MPEG-4');
videoWriter.FrameRate = video.FrameRate;  % Maintain the same frame rate as the input video
open(videoWriter);

% Read the first frame and convert to grayscale
frame = readFrame(video);
grayFrame = rgb2gray(frame);

% Create a mask to filter out the blue particles
hsvFrame = rgb2hsv(frame);
hue = hsvFrame(:,:,1);
saturation = hsvFrame(:,:,2);
value = hsvFrame(:,:,3);
mask = (hue > 0.55 & hue < 0.75 & saturation > 0.4 & value > 0.2); % Mask for blue particles
grayFrame(mask) = 0;

% Detect large particles in the first frame using imfindcircles
[centers, radii] = imfindcircles(grayFrame, [6 30], 'ObjectPolarity', 'bright', 'EdgeThreshold', 0.1, 'Sensitivity', 0.99);

% Display the original image with detected particles
if isempty(centers)
    error('No particles detected in the first frame.');
end
points = centers;

% Initialize the point tracker
pointTracker = vision.PointTracker('MaxBidirectionalError', 3);
initialize(pointTracker, points, grayFrame);

% Initialize Kalman filters for each detected particle
numParticles = size(points, 1);
kalmanFilters = cell(numParticles, 1);
for i = 1:numParticles
    kalmanFilters{i} = configureKalmanFilter('ConstantVelocity', points(i, :), [1 1]*1e5, [25, 10], 25);
end

% Initialize arrays to store particle positions and kymograph data
particlePositions = cell(numParticles, 1);
for i = 1:numParticles
    particlePositions{i} = [];
end
frameWidth = video.Width;
numFrames = floor(video.Duration * video.FrameRate);
kymograph = zeros(numFrames, frameWidth);

% Particle tracking variables
nextLabel = 1; % Next available label number
labels = (nextLabel:nextLabel + numParticles - 1)'; % Initialize labels for points
nextLabel = nextLabel + numParticles;
isInitialized = false; % Flag to check if tracker is initialized

% Existing code to load video and initialize objects...

% Process each frame
frameIndex = 1;
while hasFrame(video)
    frame = readFrame(video);
    grayFrame = rgb2gray(frame);
    
    % Apply the blue particle mask
    hsvFrame = rgb2hsv(frame);
    hue = hsvFrame(:,:,1);
    saturation = hsvFrame(:,:,2);
    value = hsvFrame(:,:,3);
    mask = (hue > 0.55 & hue < 0.75 & saturation > 0.4 & value > 0.2); % Mask for blue particles
    grayFrame(mask) = 0;
    
    % Predict the next positions using the Kalman filters
    predictedPoints = [];
    for i = 1:numParticles
        predictedPoints = [predictedPoints; predict(kalmanFilters{i})];
    end
    
    % Detect particles in the current frame using the threshold method
    binaryFrame = grayFrame > 150;  % Threshold value
    cleanedFrame = imopen(binaryFrame, strel('disk', 1)); % Remove small objects
    cleanedFrame = imclose(cleanedFrame, strel('disk', 3)); % Fill small holes
    blobAnalysis = vision.BlobAnalysis('MinimumBlobArea', 20, 'MaximumBlobArea', 2000); % Adjust based on cargo size
    [~, centroids, bboxes] = step(blobAnalysis, cleanedFrame);
    
    if ~isInitialized
        % Initialize the point tracker with detected points
        if ~isempty(centroids)
            points = centroids;
            labels = (nextLabel:(nextLabel + size(centroids, 1) - 1))';
            nextLabel = nextLabel + size(centroids, 1);
            initialize(pointTracker, points, frame);
            isInitialized = true;
        end
    else
        if ~isempty(points)
            % Track the points
            [trackedPoints, validity] = step(pointTracker, frame);
            points = trackedPoints(validity, :);
            if numel(labels) >= numel(validity)
                labels = labels(validity);
            else
                labels = labels(validity(1:numel(labels)));
            end
        else
            points = [];
        end
        
        % Add new points if needed
        if ~isempty(centroids)
            distances = pdist2(points, centroids);
            for i = 1:size(centroids, 1)
                if isempty(distances) || min(distances(:, i)) > 20 % Distance threshold for adding new points
                    points = [points; centroids(i, :)];
                    labels = [labels; nextLabel];
                    nextLabel = nextLabel + 1;
                end
            end
            if ~isempty(points)
                setPoints(pointTracker, points);
            end
        end
    end
    
    % Correct the Kalman filters with the detected points
    if ~isempty(points)
        for i = 1:min(numParticles, size(points, 1))
            correct(kalmanFilters{i}, points(i, :));
        end
    else
        points = predictedPoints;
    end
    
    % Store valid particle positions
    for i = 1:numParticles
        if i <= size(points, 1)
            particlePositions{i} = [particlePositions{i}; points(i, :)];
        else
            particlePositions{i} = [particlePositions{i}; NaN NaN];
        end
    end
    
    frameIndex = frameIndex + 1;
end

% Close the VideoWriter object
close(videoWriter);

% Calculate instantaneous speeds for each particle
allSpeeds = cell(numParticles, 1);
for i = 1:numParticles
    positions = particlePositions{i};
    speeds = [];
    for j = 2:size(positions, 1)
        if ~isnan(positions(j, 1)) && ~isnan(positions(j-1, 1))
            deltaPos = positions(j, :) - positions(j-1, :);
            speed = norm(deltaPos) * video.FrameRate;  % Speed in pixels per second
            speeds = [speeds; speed];
        else
            speeds = [speeds; NaN];
        end
    end
    allSpeeds{i} = speeds;
end

% Compute average speeds for each particle
averageSpeeds = zeros(numParticles, 1);
for i = 1:numParticles
    averageSpeeds(i) = nanmean(allSpeeds{i});
end

% Calculate the overall average speed of all particles
overallAverageSpeed = nanmean(averageSpeeds);

% Display the average speeds and the overall average speed
disp('Average speeds of each particle:');
disp(averageSpeeds);
disp('Overall average speed of all particles:');
disp(overallAverageSpeed);

% Display completion message
disp('Tracking complete.');

% Use implay to play the output processed video (existing code)
implay(outputVideoFile);
