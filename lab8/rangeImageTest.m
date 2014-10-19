%% Load data
load('rangeData', 'front','robotLeft', 'back', 'robotRight');

rFront = rangeImage(front, 1, true);
rLeft = rangeImage(robotLeft, 1, true);
rRight = rangeImage(robotRight,1,true);
rBack = rangeImage(back, 1, true);


%% Test rangeImage

[err, num, th] = rBack.findLineCandidate(3, 2);

%%
test = [0 286.5 143.2 95.54 71.68 57.37]; 
rTest = rangeImage(test, 1, true);
[err, num, th] = rTest.findLineCandidate(4, 230);