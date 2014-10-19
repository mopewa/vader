%% Load data
load('rangeData', 'front','robotLeft', 'back', 'robotRight');

rFront = rangeImage(front, 1, false);
rLeft = rangeImage(robotLeft, 1, false);
rRight = rangeImage(robotRight,1,false);
rBack = rangeImage(back, 1, false);


%% Test rangeImage

[err, num, th] = rFront.findLineCandidate(3, 5);

%%
test = [0 286.5 143.2 95.54 71.68 57.37];
rTest = rangeImage(test, 1, false);
[err, num, th] = rTest.findLineCandidate(4, 225);