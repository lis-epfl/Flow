load('flow81_cam.mat');
frames = [];
for i = 1:length(CamTime),
    if(CamTime(i)>10000),
        frames = [frames, CamFrame(i)];
    end
end

diff = [];
for k = 1:length(frames)-1,
    diff = [diff; frames(k+1)-frames(k)];
end