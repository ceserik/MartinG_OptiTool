% 
% for i = 1:length(results.data.data.data4WD)
%     expName = sprintf('%s_%s_S-%s_D_%s_GEPD_%d', results.data.data.data4WD(i).car.PowertrainType, results.data.data.data4WD(i).car.discipline  , results.data.data.data4WD(i).optparams.steeredAxle, results.data.data.data4WD(i).optparams.drivenAxle, results.data.data.data4WD(i).car.gepdToggle);
%     videoName = fullfile('results', strcat(expName, '.avi'));
%     raceCar_visu(results.data.data.data4WD(i), 'FileName', videoName, 'ColoredTrackLine', true, 'ShowTireForce', true);
% end
% 
% for i = 1:length(results.data.data.data4WDTV)
%     expName = sprintf('%s_%s_S-%s_D_%s_GEPD_%d', results.data.data.data4WDTV(i).car.PowertrainType, results.data.data.data4WDTV(i).car.discipline  , results.data.data.data4WDTV(i).optparams.steeredAxle, results.data.data.data4WDTV(i).optparams.drivenAxle, results.data.data.data4WDTV(i).car.gepdToggle);
%     videoName = fullfile('results', strcat(expName, '.avi'));
%     raceCar_visu(results.data.data.data4WDTV(i), 'FileName', videoName, 'ColoredTrackLine', true, 'ShowTireForce', true);
% end
% 
% for i = 1:length(results.data.data.data4WDRS)
%     expName = sprintf('%s_%s_S-%s_D_%s_GEPD_%d', results.data.data.data4WDRS(i).car.PowertrainType, results.data.data.data4WDRS(i).car.discipline  , results.data.data.data4WDRS(i).optparams.steeredAxle, results.data.data.data4WDRS(i).optparams.drivenAxle, results.data.data.data4WDRS(i).car.gepdToggle);
%     videoName = fullfile('results', strcat(expName, '.avi'));
%     raceCar_visu(results.data.data.data4WDRS(i), 'FileName', videoName, 'ColoredTrackLine', true, 'ShowTireForce', true);
% end

for i = 1:length(results.data.data.data2WD)
    expName = sprintf('%s_%s_S-%s_D_%s_GEPD_%d', results.data.data.data2WD(i).car.PowertrainType, results.data.data.data2WD(i).car.discipline  , results.data.data.data2WD(i).optparams.steeredAxle, results.data.data.data2WD(i).optparams.drivenAxle, results.data.data.data2WD(i).car.gepdToggle);
    videoName = fullfile('results', strcat(expName, '.avi'));
    raceCar_visu(results.data.data.data2WD(i), 'FileName', videoName, 'ColoredTrackLine', true, 'ShowTireForce', true);
end