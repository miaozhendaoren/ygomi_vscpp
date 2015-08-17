function newDataParser()

addpath('../../../../Tools/libicp/matlab');

close all;
clc;

fid = fopen('newData2.txt', 'r');

lonStand = -83.21325826306641;
latStand = 42.29687171068353;

dbData = cell(1);
dbPaintFlag = cell(1);

str = fgetl(fid);
while ischar(str) % before end of file
    %% Get data from file
    while ~isempty(str)
        eval(str);
        str = fgetl(fid);
    end

    %% convert to relative location in meters
    latLRel = latL;
    lonLRel = lonL;
    latRRel = latR;
    lonRRel = lonR;
    
    for idx = 1:length(latL)
        [lonLRel(idx), latLRel(idx)] = calcRelativeLocation(lonStand, latStand, lonL(idx), latL(idx));
        [lonRRel(idx), latRRel(idx)] = calcRelativeLocation(lonStand, latStand, lonR(idx), latR(idx));
    end
    
    %% fit
    newDataL = [lonLRel; latLRel];
    newDataR = [lonRRel; latRRel];
    
    if isempty(dbData{1})
        dbData{1}{1} = newDataL;
        dbData{1}{2} = newDataR;
        dbPaintFlag{1}{1} = linePaintFlagL;
        dbPaintFlag{1}{2} = linePaintFlagR;
    else
        dbDataIdx = 1;
        
        newDataCell = cell(1);
        newDataCell{1} = newDataL;
        newDataCell{2} = newDataR;
        ndPaintFlagCell = cell(1);
        ndPaintFlagCell{1} = linePaintFlagL;
        ndPaintFlagCell{2} = linePaintFlagR;
        
        while dbDataIdx <= length(dbData)
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
            
            ndLandMarkerArr = getLandMark(ndArr, ndPaintFlagArr);
            dbLandMarkerArr = getLandMark(dbArr, dbPaintFlagArr);
            
            [Tr_fit, activeIdx, matchIdx, closeIdx, minDist, meanDist] = icpMex(dbLandMarkerArr,ndLandMarkerArr,eye(3),10,'point_to_point',200);
            if(length(activeIdx) >= 6)
                Tr_fit
                [Tr_fit, activeIdx, matchIdx, closeIdx, minDist, meanDist] = icpMex(dbPaintArr,ndPaintArr,Tr_fit,7,'point_to_point',200);
                Tr_fit
                
                if(length(activeIdx) < 10)
                    dbDataIdx = dbDataIdx + 1;
                    continue;
                else
                    % merge
                    newDataMoveCoeff = 0.7;
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
                    plot(ndPaintArr(1,:), ndPaintArr(2,:), 'rs');
                    hold on;
                    plot(ndLandMarkerArr(1,:), ndLandMarkerArr(2,:), 'rx');
                    plot(dbPaintArr(1,:), dbPaintArr(2,:), 'bs');
                    plot(dbLandMarkerArr(1,:), dbLandMarkerArr(2,:), 'bx');
                    
                    for mergedLineIdx = 1:length(mergedDataCell)
                        plot(mergedDataCell{mergedLineIdx}(1,mergedPaintFlagCell{mergedLineIdx} == 1), mergedDataCell{mergedLineIdx}(2,mergedPaintFlagCell{mergedLineIdx} == 1), 'gs');
                        plot(mergedDataCell{mergedLineIdx}(1,mergedPaintFlagCell{mergedLineIdx} == 0), mergedDataCell{mergedLineIdx}(2,mergedPaintFlagCell{mergedLineIdx} == 0), 'g.');
                    end
                    
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

%     for idx = 1:length(latL)
%         if linePaintFlagL(idx) == 0
%             plot(lonLRel(idx), latLRel(idx), 'r.');
%         else
%             plot(lonLRel(idx), latLRel(idx), 'rs');
%         end
% 
%         if linePaintFlagR(idx) == 0
%             plot(lonRRel(idx), latRRel(idx), 'b.');
%         else
%             plot(lonRRel(idx), latRRel(idx), 'bs');
%         end
%     end
    
    %%
    str = fgetl(fid);
end

fclose(fid);

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
                = icpMex(dbPaint_fitCell{dbLineIdx},ndPaint_fitCell{ndLineIdx},eye(3),50,'point_to_point',0);
        end
    end

    % find how to match line groups
    meanDistArr = zeros(1, numNdLine + numDbLine - 1);
    meanDistAddCount = meanDistArr;
    for ndLineIdx = 1:numNdLine
        for dbLineIdx = 1:numDbLine
            meanDistIdx = dbLineIdx - ndLineIdx + numNdLine;
            meanDistArr(meanDistIdx) = meanDistArr(meanDistIdx) + meanDist{ndLineIdx, dbLineIdx};
            meanDistAddCount(meanDistIdx) = meanDistAddCount(meanDistIdx) + 1;
        end
    end
    meanDistAvg = meanDistArr ./ meanDistAddCount;
    [meanDist, idx] = min(meanDistAvg);
    
    % output
    mergedDataCell = cell(0);
    paintFlagCell = cell(0);
    ndLineIdx = 1;
    dbLineIdx = 1;
    if idx > numNdLine
        % db first
        for dbLineIdx = 1:(idx - numNdLine)
            mergedDataCell{end+1} = dbPaint_fitCell{dbLineIdx};
            paintFlagCell{end+1} = dbPaintFlagCell{dbLineIdx};
        end
        dbLineIdx = dbLineIdx + 1;
    elseif idx < numNdLine
        % new data first
        for ndLineIdx = 1:(numNdLine - idx)
            mergedDataCell{end+1} = ndPaint_fitCell{ndLineIdx};
            paintFlagCell{end+1} = ndPaintFlagCell{ndLineIdx};
        end
        ndLineIdx = ndLineIdx + 1;
    end
    
    for count = 1:meanDistAddCount(idx)
        if isempty(closeIdx{ndLineIdx, dbLineIdx})
            % ignore new data if do not match well
            mergedDataCell = ndPaint_fitCell;
            paintFlagCell = ndPaintFlagCell;
            successFlag = 0;
            return;
        end
        
        [mergedData, paintFlag] = mergeLines(ndPaint_fitCell{ndLineIdx}, ndPaintFlagCell{ndLineIdx}, ...
            dbPaint_fitCell{dbLineIdx}, dbPaintFlagCell{dbLineIdx}, ...
            activeIdx{ndLineIdx, dbLineIdx}, matchIdx{ndLineIdx, dbLineIdx}, closeIdx{ndLineIdx, dbLineIdx});
        
        mergedDataCell{end+1} = mergedData;
        paintFlagCell{end+1} = paintFlag;
        
        dbLineIdx = dbLineIdx + 1;
        ndLineIdx = ndLineIdx + 1;
    end
    
    if dbLineIdx <= numDbLine
        % db last
        while dbLineIdx <= numDbLine
            mergedDataCell{end+1} = dbPaint_fitCell{dbLineIdx};
            paintFlagCell{end+1} = dbPaintFlagCell{dbLineIdx};
            
            dbLineIdx = dbLineIdx + 1;
        end
    elseif ndLineIdx <= numNdLine
        % new data last
        while ndLineIdx <= numNdLine
            mergedDataCell{end+1} = ndPaint_fitCell{ndLineIdx};
            paintFlagCell{end+1} = ndPaintFlagCell{ndLineIdx};
            
            ndLineIdx = ndLineIdx + 1;
        end
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
function landMarker = getLandMark(newData, linePaintFlag)
    landMarkFlag = zeros(size(linePaintFlag));
    for idx = 2:length(linePaintFlag)
        if (linePaintFlag(idx-1) == 0) && (linePaintFlag(idx) == 1)
            landMarkFlag(idx) = 1;
        elseif (linePaintFlag(idx-1) == 1) && (linePaintFlag(idx) == 0)
            landMarkFlag(idx-1) = 1;
        end
    end
    
    landMarker = newData(:, landMarkFlag==1);
end

