% Load video
videoFile = '/Users/jinho/Downloads/OPALS/1.mp4'; 
video = VideoReader(videoFile);

% Parameters (final vid 1)
minBlobArea = 20;  % minimum blob area
maxBlobArea = 300; % maximum blob area
velocityThreshold = 1.3; % pixels per frame
pauseThreshold = 5; % frames, adjust as needed
brightnessThreshold = 0.1; % brightness threshold (0-1 range)
redThreshold = 0.1; % Threshold for red channel intensity (0-1 range)
maxLinkingDistance = 100; % Maximum distance for linking tracks between frames

% Initialize variables
frameCount = 0;
allTracks = struct('positions', {}, 'frames', {}, 'velocities', {}, 'pauseTimes', {});
kymograph = zeros(video.NumFrames, video.Width);

% Process each frame
while hasFrame(video)
    frame = readFrame(video);
    frameCount = frameCount + 1;
    
    % Extract red channel and normalize
    redChannel = im2double(frame(:,:,1));
    
    % Create a mask for bright red objects
    redMask = redChannel > redThreshold & ...
              redChannel > im2double(frame(:,:,2)) & ...
              redChannel > im2double(frame(:,:,3));
    
    % Apply brightness threshold
    brightMask = rgb2gray(frame) > brightnessThreshold * 255;
    
    % Combine masks
    binaryImage = redMask & brightMask;
    
    % Remove small objects and fill holes
    binaryImage = bwareaopen(binaryImage, minBlobArea);
    binaryImage = imfill(binaryImage, 'holes');
    
    % Detect blobs
    [labeledImage, numBlobs] = bwlabel(binaryImage);
    blobMeasurements = regionprops(labeledImage, 'Centroid', 'Area');
    
    % Filter blobs by area
    validBlobs = find([blobMeasurements.Area] >= minBlobArea & [blobMeasurements.Area] <= maxBlobArea);
    
    % Update tracks
    for i = 1:length(validBlobs)
        centroid = blobMeasurements(validBlobs(i)).Centroid;
        
        % Find the closest existing track or create a new one
        minDistance = Inf;
        closestTrack = 0;
        for j = 1:length(allTracks)
            if ~isempty(allTracks(j).positions)
                distance = norm(centroid - allTracks(j).positions(end,:));
                if distance < minDistance && distance <= maxLinkingDistance
                    minDistance = distance;
                    closestTrack = j;
                end
            end
        end
        
        if closestTrack == 0
            % Create new track
            newTrack = length(allTracks) + 1;
            allTracks(newTrack).positions = centroid;
            allTracks(newTrack).frames = frameCount;
            allTracks(newTrack).velocities = [];
            allTracks(newTrack).pauseTimes = [];
        else
            % Update existing track
            allTracks(closestTrack).positions(end+1,:) = centroid;
            allTracks(closestTrack).frames(end+1) = frameCount;
            
            % Calculate velocity
            if length(allTracks(closestTrack).frames) > 1
                displacement = norm(allTracks(closestTrack).positions(end,:) - allTracks(closestTrack).positions(end-1,:));
                timeStep = allTracks(closestTrack).frames(end) - allTracks(closestTrack).frames(end-1);
                if timeStep > 0
                    velocity = displacement / timeStep;
                    allTracks(closestTrack).velocities(end+1) = velocity;
                    
                    % Update kymograph only if velocity is above threshold
                    if velocity >= velocityThreshold
                        kymograph(frameCount, round(centroid(1))) = 255;
                    end
                else
                    % Skip velocity calculation if timeStep is zero
                    disp(['Warning: Zero time step detected for track ', num2str(closestTrack), ' at frame ', num2str(frameCount)]);
                end
                
                % Check for pauses
                if exist('velocity', 'var') && velocity < velocityThreshold
                    if isempty(allTracks(closestTrack).pauseTimes)
                        allTracks(closestTrack).pauseTimes = [frameCount, frameCount];
                    elseif allTracks(closestTrack).pauseTimes(end,2) ~= frameCount - 1
                        allTracks(closestTrack).pauseTimes(end+1,:) = [frameCount, frameCount];
                    else
                        allTracks(closestTrack).pauseTimes(end,2) = frameCount;
                    end
                end
            end
        end
    end
    
    % Visualize tracking (optional)
    imshow(frame);
    hold on;
    for i = 1:length(allTracks)
        if ~isempty(allTracks(i).positions)
            plot(allTracks(i).positions(:,1), allTracks(i).positions(:,2), '-o');
        end
    end
    hold off;
    drawnow;
end

% Calculate and display results
totalValidVelocities = [];
totalTracks = 0;
totalPauseTime = 0;
totalPauses = 0;

for i = 1:length(allTracks)
    if ~isempty(allTracks(i).velocities)
        validVelocities = allTracks(i).velocities(isfinite(allTracks(i).velocities));
        
        % Only include velocities above the threshold
        aboveThresholdVelocities = validVelocities(validVelocities >= velocityThreshold);
        totalValidVelocities = [totalValidVelocities, aboveThresholdVelocities];
        
        if ~isempty(validVelocities)
            avgVelocity = mean(validVelocities);
            avgVelocityAboveThreshold = mean(aboveThresholdVelocities);
        else
            avgVelocity = 0;
            avgVelocityAboveThreshold = 0;
        end
        
        if ~isempty(allTracks(i).pauseTimes)
            trackPauseTime = sum(allTracks(i).pauseTimes(:,2) - allTracks(i).pauseTimes(:,1) + 1);
            numPauses = size(allTracks(i).pauseTimes, 1);
            totalPauseTime = totalPauseTime + trackPauseTime;
            totalPauses = totalPauses + numPauses;
        else
            trackPauseTime = 0;
            numPauses = 0;
        end
        
        fprintf('Track %d:\n', i);
        fprintf('  Average velocity (all): %.2f pixels/frame\n', avgVelocity);
        fprintf('  Average velocity (above threshold): %.2f pixels/frame\n', avgVelocityAboveThreshold);
        fprintf('  Total pause time: %d frames\n', trackPauseTime);
        fprintf('  Number of pauses: %d\n', numPauses);
        fprintf('  Track duration: %d frames\n', allTracks(i).frames(end) - allTracks(i).frames(1) + 1);
        fprintf('  Number of positions: %d\n\n', length(allTracks(i).positions));
        
        totalTracks = totalTracks + 1;
    end
end

% Calculate overall statistics
if ~isempty(totalValidVelocities)
    overallAvgVelocity = mean(totalValidVelocities);
    fprintf('Overall Statistics:\n');
    fprintf('  Total number of tracks: %d\n', totalTracks);
    fprintf('  Overall average velocity (above threshold): %.2f pixels/frame\n', overallAvgVelocity);
    fprintf('  Total pause time across all tracks: %d frames\n', totalPauseTime);
    fprintf('  Total number of pauses across all tracks: %d\n', totalPauses);
    if totalTracks > 0
        fprintf('  Average pauses per track: %.2f\n', totalPauses / totalTracks);
        fprintf('  Average pause time per track: %.2f frames\n', totalPauseTime / totalTracks);
    end
else
    fprintf('No valid velocity data available.\n');
end
    
% Display kymograph
figure;
imshow(kymograph, []);
title('Kymograph');
xlabel('Position along axon');
ylabel('Frame number');