%% Load data
load('rangeData', 'front','robotLeft', 'back', 'robotRight');

%% Test rangeImage
r.plotRvsTh(5)
figure(2);
r.plotXvsY(5)

%%
r.plotXvsY(2);
r.plotRvsTh(2);

%%
[err, num, th] = r.findLineCandidate(3, 2);