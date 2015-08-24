function newDataParser5()

addpath('../../../../Tools/libicp/matlab');

close all;
clc;

lonStand = -122.05417410240001;
latStand = 37.3958123816;

newDataAllL = cell(0);
newDataAllR = cell(0);
ndPaintFlagAllL = cell(0);
ndPaintFlagAllR = cell(0);

for fileIdx = [1];
    fileIdx
%     fid = fopen(['newData0' num2str(fileIdx) '.txt'], 'r');
    fid = fopen(['newData - Copy.txt'], 'r');
    str = fgetl(fid);
    dataIdx = 0;
    
    newDataCellL = cell(0);
    newDataCellR = cell(0);
    ndPaintFlagCellL = cell(0);
    ndPaintFlagCellR = cell(0);
    
    while ischar(str) % before end of file
        %% Get data from file

        eval(['latL = [' str '];']);
        str = fgetl(fid);
        eval(['latR = [' str '];']);
        str = fgetl(fid);
        eval(['lonL = [' str '];']);
        str = fgetl(fid);
        eval(['lonR = [' str '];']);
        str = fgetl(fid);
        eval(['linePaintFlagL = [' str '];']);
        str = fgetl(fid);
        eval(['linePaintFlagR = [' str '];']);
        str = fgetl(fid);
        str = fgetl(fid);
        dataIdx = dataIdx + 1

        %% convert to relative location in meters
        latLRel = latL;
        lonLRel = lonL;
        latRRel = latR;
        lonRRel = lonR;

        %% fit
        newDataL = [lonLRel; latLRel];
        newDataR = [lonRRel; latRRel];

        newDataCellL{end+1} = newDataL;
        newDataCellR{end+1} = newDataR;
        ndPaintFlagCellL{end+1} = linePaintFlagL;
        ndPaintFlagCellR{end+1} = linePaintFlagR;

    end
    
    fclose(fid);
    
    newDataAllL{end+1} = newDataCellL;
    newDataAllR{end+1} = newDataCellR;
    ndPaintFlagAllL{end+1} = ndPaintFlagCellL;
    ndPaintFlagAllR{end+1} = ndPaintFlagCellR;
end
%%
figure(2);
hold on;
plotStyle = {'rs', 'gs-', 'bs-', 'ms-', 'ks-', 'cs-'};
shift = {[0; 0], [0; 0], [0; 0], [0; 0], [0; 0]};
for fileIdx = 1:length(newDataAllL)
    for sectionIdx = 1:length(newDataAllL{fileIdx})
%         plot(newDataAllL{fileIdx}{sectionIdx}(1,ndPaintFlagAllL{fileIdx}{sectionIdx} == 1)+shift{fileIdx}(1), newDataAllL{fileIdx}{sectionIdx}(2,ndPaintFlagAllL{fileIdx}{sectionIdx} == 1)+shift{fileIdx}(2), plotStyle{fileIdx});
%         plot(newDataAllR{fileIdx}{sectionIdx}(1,ndPaintFlagAllR{fileIdx}{sectionIdx} == 1)+shift{fileIdx}(1), newDataAllR{fileIdx}{sectionIdx}(2,ndPaintFlagAllR{fileIdx}{sectionIdx} == 1)+shift{fileIdx}(2), plotStyle{fileIdx});
        plot(newDataAllL{fileIdx}{sectionIdx}(1,:)+shift{fileIdx}(1), newDataAllL{fileIdx}{sectionIdx}(2,:)+shift{fileIdx}(2), plotStyle{fileIdx});
        plot(newDataAllR{fileIdx}{sectionIdx}(1,:)+shift{fileIdx}(1), newDataAllR{fileIdx}{sectionIdx}(2,:)+shift{fileIdx}(2), plotStyle{fileIdx});

    end
end

end

%%
function [x, y] = calcRelativeLocation(lonStand, latStand, lon, lat)
    latDegree = latStand * pi / 180;
    x = (lon - lonStand) * (111413*cos(latDegree) - 94*cos(3*latDegree));
    y = (lat - latStand) * 111320.0;
end

