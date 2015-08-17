function newDataParser2()

addpath('../../../../Tools/libicp/matlab');

close all;
clc;

lonStand = -83.21325826306641;
latStand = 42.29687171068353;

dbData = cell(1);
dbPaintFlag = cell(1);

dataIdx = 0;
fid = fopen('newData2p3_C_bad.txt', 'r');
str = fgetl(fid);
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
    
%     [lonLRel, latLRel] = calcRelativeLocation(lonStand, latStand, lonL, latL);
%     [lonRRel, latRRel] = calcRelativeLocation(lonStand, latStand, lonR, latR);
    
    %% fit
    newDataL = [lonLRel; latLRel];
    newDataR = [lonRRel; latRRel];

    if(size(newDataL, 2) < 100)
        % ignore the new data if too short
        continue;
    end

    newDataCell = cell(1);
    newDataCell{1} = newDataL;
    newDataCell{2} = newDataR;
    ndPaintFlagCell = cell(1);
    ndPaintFlagCell{1} = linePaintFlagL;
    ndPaintFlagCell{2} = linePaintFlagR;
    
    ndPaintCell = cell(1);
    ndArr = [];
    ndPaintFlagArr = [];
    ndPaintArr = [];
    for ndLineIdx = 1:length(newDataCell)
        ndPaintCell{ndLineIdx} = newDataCell{ndLineIdx}(:, ndPaintFlagCell{ndLineIdx} == 1);
        ndPaintArr = [ndPaintArr, ndPaintCell{ndLineIdx}];
        ndArr = [ndArr, newDataCell{ndLineIdx}];
        ndPaintFlagArr = [ndPaintFlagArr, ndPaintFlagCell{ndLineIdx}];
    end
    
    ndLandMarkerArr = getLandMark(newDataCell, ndPaintFlagCell);
    
    if(size(ndLandMarkerArr, 2) < 10)
        % ignore the new data if too short
        continue;
    end
    
    if isempty(dbData{1})
        dbData{1}{1} = newDataL;
        dbData{1}{2} = newDataR;
        dbPaintFlag{1}{1} = linePaintFlagL;
        dbPaintFlag{1}{2} = linePaintFlagR;
    else
        dbDataIdx = 1;

        while dbDataIdx <= length(dbData)
            dbDataCell = dbData{dbDataIdx};
            dbPaintFlagCell = dbPaintFlag{dbDataIdx};
            dbPaintCell = cell(1);
            dbArr = [];
            dbPaintFlagArr = [];
            dbPaintArr = [];
            for dbLineIdx = 1:length(dbDataCell)
                dbPaintCell{dbLineIdx} = dbDataCell{dbLineIdx}(:, dbPaintFlagCell{dbLineIdx} == 1);
                dbPaintArr = [dbPaintArr, dbPaintCell{dbLineIdx}];
                dbArr = [dbArr, dbDataCell{dbLineIdx}];
                dbPaintFlagArr = [dbPaintFlagArr, dbPaintFlagCell{dbLineIdx}];
            end

            dbLandMarkerArr = getLandMark(dbDataCell, dbPaintFlagCell);
            
            [Tr_fit, activeIdx, matchIdx, closeIdx, minDist, meanDist] = icpMex(dbLandMarkerArr,ndLandMarkerArr,eye(3),14,'point_to_point',200);
            if(length(activeIdx) >= 4)
                Tr_fit
                [Tr_fit, activeIdx, matchIdx, closeIdx, minDist, meanDist] = icpMex(dbPaintArr,ndPaintArr,Tr_fit,1,'point_to_point',200);
                Tr_fit

                if(length(activeIdx) < 20)
                    dbDataIdx = dbDataIdx + 1;
                    continue;
                else
                    % merge
                    newDataMoveCoeff = size(dbPaintArr,2) / (size(ndPaintArr,2) + size(dbPaintArr,2));
                    ndPaint_fitCell = cell(1);
                    for ndLineIdx = 1:length(newDataCell)
                        ndPaint_fitCell{ndLineIdx} = newDataCell{ndLineIdx} + newDataMoveCoeff.*(Tr_fit(1:2,3)*ones(1,size(newDataCell{ndLineIdx},2))); % Tr_fit(1:2,1:2)
                    end

                    dbPaint_fitCell = cell(1);
                    for dbLineIdx = 1:length(dbDataCell)
                        dbPaint_fitCell{dbLineIdx} = dbDataCell{dbLineIdx} - (1-newDataMoveCoeff).*(Tr_fit(1:2,3)*ones(1,size(dbDataCell{dbLineIdx},2)));
                    end

                    [mergedDataCell, mergedPaintFlagCell, mergeSuccess] = merge(ndPaint_fitCell, ndPaintFlagCell, dbPaint_fitCell, dbPaintFlagCell);

                    figure(1);
                    hold off;
                    plot(ndLandMarkerArr(1,:), ndLandMarkerArr(2,:), 'rx');
                    hold on;
                    plot(dbLandMarkerArr(1,:), dbLandMarkerArr(2,:), 'bx');
                    for ndLineIdx = 1:length(newDataCell)
                        plot(newDataCell{ndLineIdx}(1,ndPaintFlagCell{ndLineIdx}==1), newDataCell{ndLineIdx}(2,ndPaintFlagCell{ndLineIdx}==1), 'rs');
                        plot(newDataCell{ndLineIdx}(1,ndPaintFlagCell{ndLineIdx}==0), newDataCell{ndLineIdx}(2,ndPaintFlagCell{ndLineIdx}==0), 'r.');
                    end
                    for dbLineIdx = 1:length(dbDataCell)
                        plot(dbDataCell{dbLineIdx}(1,dbPaintFlagCell{dbLineIdx}==1), dbDataCell{dbLineIdx}(2,dbPaintFlagCell{dbLineIdx}==1), 'bs');
                        plot(dbDataCell{dbLineIdx}(1,dbPaintFlagCell{dbLineIdx}==0), dbDataCell{dbLineIdx}(2,dbPaintFlagCell{dbLineIdx}==0), 'b.');
                    end

                    for mergedLineIdx = 1:length(mergedDataCell)
                        plot(mergedDataCell{mergedLineIdx}(1,mergedPaintFlagCell{mergedLineIdx} == 1), mergedDataCell{mergedLineIdx}(2,mergedPaintFlagCell{mergedLineIdx} == 1), 'gs');
                        plot(mergedDataCell{mergedLineIdx}(1,mergedPaintFlagCell{mergedLineIdx} == 0), mergedDataCell{mergedLineIdx}(2,mergedPaintFlagCell{mergedLineIdx} == 0), 'g.');
                    end

%                     if dataIdx == 50
%                         stop = 1;
%                     end
                    
                    newDataCell = mergedDataCell;
                    ndPaintFlagCell = mergedPaintFlagCell;

                    if mergeSuccess == 1
                        dbData(dbDataIdx) = [];
                        dbPaintFlag(dbDataIdx) = [];
                    end

                    dbDataIdx = dbDataIdx + 1;
                end
            else
                dbDataIdx = dbDataIdx + 1;
                continue;
            end
        end

        % push the data
        dbData{end+1} = newDataCell;
        dbPaintFlag{end+1} = ndPaintFlagCell;
    end
end

%%
figure(2);
hold on;
for dataIdx = 1:length(dbData)
    for lineIdx = 1:length(dbData{dataIdx})
        plot(dbData{dataIdx}{lineIdx}(1,dbPaintFlag{dataIdx}{lineIdx} == 1), dbData{dataIdx}{lineIdx}(2,dbPaintFlag{dataIdx}{lineIdx} == 1), 'gs');
        plot(dbData{dataIdx}{lineIdx}(1,dbPaintFlag{dataIdx}{lineIdx} == 0), dbData{dataIdx}{lineIdx}(2,dbPaintFlag{dataIdx}{lineIdx} == 0), 'g.');
    end
end

end

%%
function [x, y] = calcRelativeLocation(lonStand, latStand, lon, lat)
    latDegree = latStand * pi / 180;
    x = (lon - lonStand) * (111413*cos(latDegree) - 94*cos(3*latDegree));
    y = (lat - latStand) * 111320.0;
end

%%
function [mergedDataCell, paintFlagCell, successFlag] = merge(ndPaint_fitCell, ndPaintFlagCell, dbPaint_fitCell, dbPaintFlagCell)
    numNdLine = length(ndPaint_fitCell);
    numDbLine = length(dbPaint_fitCell);
    successFlag = 1;
    
    % match each line
    Tr_fit = cell(numNdLine, numDbLine);
    activeIdx = Tr_fit;
    matchIdx = Tr_fit;
    closeIdx = Tr_fit;
    minDist = Tr_fit;
    meanDist = Tr_fit;
    for ndLineIdx = 1:numNdLine
        for dbLineIdx = 1:numDbLine
            [Tr_fit{ndLineIdx, dbLineIdx}, activeIdx{ndLineIdx, dbLineIdx}, matchIdx{ndLineIdx, dbLineIdx}, closeIdx{ndLineIdx, dbLineIdx}, minDist{ndLineIdx, dbLineIdx}, meanDist{ndLineIdx, dbLineIdx}]...
                = icpMex(dbPaint_fitCell{dbLineIdx},ndPaint_fitCell{ndLineIdx},eye(3),100,'point_to_point',0);
        end
    end

    % find how to match line groups
    meanDistArr = 100000000*ones(1, min(numNdLine, 3) + min(numDbLine, 3) - 1);
    meanDistAddCount = zeros(1, min(numNdLine, 3) + min(numDbLine, 3) - 1);
    for ndLineIdx = 1:min(numNdLine, 3)
        for dbLineIdx = 1:min(numDbLine, 3)
            meanDistIdx = dbLineIdx - ndLineIdx + min(numNdLine, 3);
            meanDistArr(meanDistIdx) = min(meanDistArr(meanDistIdx), meanDist{ndLineIdx, dbLineIdx});
            meanDistAddCount(meanDistIdx) = meanDistAddCount(meanDistIdx) + 1;
        end
    end
    [meanDist, idx] = min(meanDistArr);
    
    if numNdLine >= 3 && numDbLine >= 3
        idx = 3;
    end
    
    % output
    mergedDataCell = cell(0);
    paintFlagCell = cell(0);
    ndUnmergedCell = cell(0);
    ndUnmergedPaintFlagCell = cell(0);
    
    ndLineIdx = 1;
    dbLineIdx = 1;
    if idx > min(numNdLine, 3)
        % db first
        for dbLineIdx = 1:(idx - min(numNdLine, 3))
            mergedDataCell{end+1} = dbPaint_fitCell{dbLineIdx};
            paintFlagCell{end+1} = dbPaintFlagCell{dbLineIdx};
        end
        dbLineIdx = dbLineIdx + 1;
    elseif idx < min(numNdLine, 3)
        % new data first
        for ndLineIdx = 1:(min(numNdLine, 3) - idx)
            mergedDataCell{end+1} = ndPaint_fitCell{ndLineIdx};
            paintFlagCell{end+1} = ndPaintFlagCell{ndLineIdx};
        end
        ndLineIdx = ndLineIdx + 1;
    end
    
    for count = 1:meanDistAddCount(idx)
        if isempty(closeIdx{ndLineIdx, dbLineIdx})
            ndUnmergedCell{end+1} = ndPaint_fitCell{ndLineIdx};
            ndUnmergedPaintFlagCell{end+1} = ndPaintFlagCell{ndLineIdx};
            
            mergedDataCell{end+1} = dbPaint_fitCell{dbLineIdx};
            paintFlagCell{end+1} = dbPaintFlagCell{dbLineIdx};
            
            dbLineIdx = dbLineIdx + 1;
            ndLineIdx = ndLineIdx + 1;
            continue;
        end
        
        [mergedData, paintFlag] = mergeLines(ndPaint_fitCell{ndLineIdx}, ndPaintFlagCell{ndLineIdx}, ...
            dbPaint_fitCell{dbLineIdx}, dbPaintFlagCell{dbLineIdx}, ...
            activeIdx{ndLineIdx, dbLineIdx}, matchIdx{ndLineIdx, dbLineIdx}, closeIdx{ndLineIdx, dbLineIdx});
        
        mergedDataCell{end+1} = mergedData;
        paintFlagCell{end+1} = paintFlag;
        
        dbLineIdx = dbLineIdx + 1;
        ndLineIdx = ndLineIdx + 1;
    end
    
    if dbLineIdx <= min(numDbLine, 3)
        % db last
        while dbLineIdx <= min(numDbLine, 3)
            mergedDataCell{end+1} = dbPaint_fitCell{dbLineIdx};
            paintFlagCell{end+1} = dbPaintFlagCell{dbLineIdx};
            
            dbLineIdx = dbLineIdx + 1;
        end
    elseif ndLineIdx <= min(numNdLine, 3)
        % new data last
        while ndLineIdx <= min(numNdLine, 3)
            mergedDataCell{end+1} = ndPaint_fitCell{ndLineIdx};
            paintFlagCell{end+1} = ndPaintFlagCell{ndLineIdx};
            
            ndLineIdx = ndLineIdx + 1;
        end
    end
    
    DIST_THRESH = 1;
    while dbLineIdx <= numDbLine
%         if ~isempty(closeIdx{1, dbLineIdx}) && meanDist(1, dbLineIdx) < DIST_THRESH
%             [mergedData, paintFlag] = mergeLines(ndPaint_fitCell{ndLineIdx}, ndPaintFlagCell{ndLineIdx}, ...
%             dbPaint_fitCell{dbLineIdx}, dbPaintFlagCell{dbLineIdx}, ...
%             activeIdx{ndLineIdx, dbLineIdx}, matchIdx{ndLineIdx, dbLineIdx}, closeIdx{ndLineIdx, dbLineIdx});
%         
%             mergedDataCell{end+1} = mergedData;
%             paintFlagCell{end+1} = paintFlag;
%         else
            mergedDataCell{end+1} = dbPaint_fitCell{dbLineIdx};
            paintFlagCell{end+1} = dbPaintFlagCell{dbLineIdx};
%         end

        dbLineIdx = dbLineIdx + 1;
    end
    
    while ndLineIdx <= numNdLine
        mergedDataCell{end+1} = ndPaint_fitCell{ndLineIdx};
        paintFlagCell{end+1} = ndPaintFlagCell{ndLineIdx};

        ndLineIdx = ndLineIdx + 1;
    end
    
    for unmergedIdx = 1:length(ndUnmergedCell)
        mergedDataCell{end+1} = ndUnmergedCell{unmergedIdx};
        paintFlagCell{end+1} = ndUnmergedPaintFlagCell{unmergedIdx};
    end
end

%%
function [outData, paintFlag] = mergeLines(newData, ndPaintFlag, dbData, dbPaintFlag, activeIdx, matchIdx, closeIdx)
    startThresh = 7;
    endThresh   = length(dbPaintFlag) - 7;
    
    indexBegin = 1;
    indexEnd   = length(matchIdx);
    while matchIdx(indexBegin == -1)
        indexBegin = indexBegin + 1;
    end
    while matchIdx(indexEnd == -1)
        indexEnd = indexEnd - 1;
    end
    
    if closeIdx(indexBegin) < startThresh
        %start with new data
        outDataStart = newData(:, 1:activeIdx(indexBegin));
        paintFlagStart = ndPaintFlag(1:activeIdx(indexBegin));
    else
        %start with DB data
        outDataStart = dbData(:, 1:closeIdx(indexBegin));
        paintFlagStart = dbPaintFlag(1:closeIdx(indexBegin));
    end
    
    outDataMid   = (newData(:,activeIdx(indexBegin:indexEnd)+1)+ dbData(:,closeIdx(indexBegin:indexEnd)+1))./2;
    paintFlagMid = ndPaintFlag(activeIdx(indexBegin:indexEnd)+1) | dbPaintFlag(closeIdx(indexBegin:indexEnd)+1);
    
    if closeIdx(indexEnd)+1 > endThresh
        %tail from new data
        outDataTail = newData(:,(activeIdx(indexEnd)+1):length(ndPaintFlag));
        paintFlagTail = ndPaintFlag((activeIdx(indexEnd)+1):length(ndPaintFlag));
    else
        outDataTail = dbData(:,(closeIdx(indexEnd)+1):length(dbPaintFlag));
        paintFlagTail = dbPaintFlag((closeIdx(indexEnd)+1):length(dbPaintFlag));
    end
    
    outData = [outDataStart, outDataMid, outDataTail];
    paintFlag = [paintFlagStart, paintFlagMid, paintFlagTail];
end

%%
function landMarker = getLandMark(newDataCell, linePaintFlagCell)
    landMarker = [];
    lmInterval = 15;
    count = lmInterval;
    for cellIdx = 1:length(newDataCell)
        linePaintFlag = linePaintFlagCell{cellIdx};
        landMarkFlag = zeros(size(linePaintFlag));
        for idx = 2:length(landMarkFlag)
            count = count + 1;
            if (linePaintFlag(idx-1) == 0) && (linePaintFlag(idx) == 1)
                if count > lmInterval
                    landMarkFlag(idx) = 1;
                end
                count = 0;
            elseif (linePaintFlag(idx-1) == 1) && (linePaintFlag(idx) == 0)
                if count > lmInterval
                    landMarkFlag(idx-1) = 1;
                end
                count = 0;
            end
        end
        landMarker = [landMarker newDataCell{cellIdx}(:, landMarkFlag==1)];
    end

    
end
