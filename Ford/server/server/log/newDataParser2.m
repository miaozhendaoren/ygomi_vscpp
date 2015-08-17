function newDataParser2()

addpath('../../../../Tools/libicp/matlab');

close all;
clc;

lonStand = -83.21325826306641;
latStand = 42.29687171068353;

% get line of file
fileCount = 1;
for fileIdx = [0: 3]
    fid = fopen(['./GPS/GPS_' num2str(fileIdx) '.txt'], 'r');
    row=0;
    while ~feof(fid)
        row=row+sum(fread(fid,10000,'*char')==char(10));
    end
    fclose(fid);

    % get data
    fid = fopen(['./GPS/GPS_' num2str(fileIdx) '.txt'], 'r');
    Data = zeros(6, row);
    lineIdx = 1;
    while ~feof(fid)
        fileLine = fscanf(fid, '%f %f %f %f %d %d', 6);
        if ~isempty(fileLine)
            Data(:, lineIdx) = fileLine;
            lineIdx = lineIdx + 1;
        else
            break;
        end
    end
    fclose(fid);

    eval(['latL{' num2str(fileCount) '} = Data(1, :);']);
    eval(['lonL{' num2str(fileCount) '} = Data(2, :);']);
    eval(['latR{' num2str(fileCount) '} = Data(3, :);']);
    eval(['lonR{' num2str(fileCount) '} = Data(4, :);']);
    eval(['linePaintFlagL{' num2str(fileCount) '} = Data(5, :);']);
    eval(['linePaintFlagR{' num2str(fileCount) '} = Data(6, :);']);
    
    fileCount = fileCount + 1;
end

for fileIdx = 1:length(lonL)
    dummyIdx = find(lonL{fileIdx} == 0);
    lonL{fileIdx}(:, dummyIdx) = [];
    latL{fileIdx}(:, dummyIdx) = [];
    lonR{fileIdx}(:, dummyIdx) = [];
    latR{fileIdx}(:, dummyIdx) = [];
    linePaintFlagL{fileIdx}(dummyIdx) = [];
    linePaintFlagR{fileIdx}(dummyIdx) = [];
    
    dummyIdx = find(latL{fileIdx} == -1);
    lonL{fileIdx}(:, dummyIdx) = [];
    latL{fileIdx}(:, dummyIdx) = [];
    lonR{fileIdx}(:, dummyIdx) = [];
    latR{fileIdx}(:, dummyIdx) = [];
    linePaintFlagL{fileIdx}(dummyIdx) = [];
    linePaintFlagR{fileIdx}(dummyIdx) = [];
    
    dummyIdx = find(lonR{fileIdx} == 0);
    lonL{fileIdx}(:, dummyIdx) = [];
    latL{fileIdx}(:, dummyIdx) = [];
    lonR{fileIdx}(:, dummyIdx) = [];
    latR{fileIdx}(:, dummyIdx) = [];
    linePaintFlagL{fileIdx}(dummyIdx) = [];
    linePaintFlagR{fileIdx}(dummyIdx) = [];
    
    dummyIdx = find(latR{fileIdx} == -1);
    lonL{fileIdx}(:, dummyIdx) = [];
    latL{fileIdx}(:, dummyIdx) = [];
    lonR{fileIdx}(:, dummyIdx) = [];
    latR{fileIdx}(:, dummyIdx) = [];
    linePaintFlagL{fileIdx}(dummyIdx) = [];
    linePaintFlagR{fileIdx}(dummyIdx) = [];
end

for fileIdx = 1:length(lonL)
%     eval(['[lonLRel{' num2str(fileIdx) '}, latLRel{' num2str(fileIdx) '}] = calcRelativeLocation(lonStand, latStand, lonL{' num2str(fileIdx) '}, latL{' num2str(fileIdx) '});']);
%     eval(['[lonRRel{' num2str(fileIdx) '}, latRRel{' num2str(fileIdx) '}] = calcRelativeLocation(lonStand, latStand, lonR{' num2str(fileIdx) '}, latR{' num2str(fileIdx) '});']);
    eval(['lonLRel{' num2str(fileIdx) '} = lonL{' num2str(fileIdx) '};']);
    eval(['lonRRel{' num2str(fileIdx) '} = lonR{' num2str(fileIdx) '};']);
    eval(['latLRel{' num2str(fileIdx) '} = latL{' num2str(fileIdx) '};']);
    eval(['latRRel{' num2str(fileIdx) '} = latR{' num2str(fileIdx) '};']);

end

figure(1);
hold on;
% for fileIdx = 1:length(lonLRel)
fileIdx = 1;
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), 'rs');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), 'rs');
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), 'r.');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), 'r.');
fileIdx = 2;
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), 'bs');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), 'bs');
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), 'b.');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), 'b.');
fileIdx = 3;
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), 'gs');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), 'gs');
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), 'g.');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), 'g.');
fileIdx = 4;
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 1), 'ms');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 1), 'ms');
    plot(lonLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), latLRel{fileIdx}(linePaintFlagL{fileIdx} == 0), 'm.');
    plot(lonRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), latRRel{fileIdx}(linePaintFlagR{fileIdx} == 0), 'm.');

% end

dbData = cell(1);
dbPaintFlag = cell(1);
fileIdx = 1;
dataLen = 2000;
dataOffset = 1;

while 1
    if (dataOffset+dataLen) >= length(lonLRel{fileIdx})
        dataEnd = length(lonLRel{fileIdx});
        if fileIdx < length(lonLRel)
            fileIdxStep = 1;
        else
            fileIdxStep = 1-length(lonLRel);
        end
        dataStart = dataOffset;
        dataOffset = 1;
    else
        dataEnd = dataOffset+dataLen;
        fileIdxStep = 0;
        dataStart = dataOffset;
        dataOffset = dataEnd + 1;
    end
    
    ndLonLRel = lonLRel{fileIdx}(:, dataStart:dataEnd);
    ndLatLRel = latLRel{fileIdx}(:, dataStart:dataEnd);
    ndLonRRel = lonRRel{fileIdx}(:, dataStart:dataEnd);
    ndLatRRel = latRRel{fileIdx}(:, dataStart:dataEnd);
    ndLinePaintFlagL = linePaintFlagL{fileIdx}(dataStart:dataEnd);
    ndLinePaintFlagR = linePaintFlagR{fileIdx}(dataStart:dataEnd);
    
    fileIdx = fileIdx + fileIdxStep;

    %% fit
    newDataL = [ndLonLRel; ndLatLRel];
    newDataR = [ndLonRRel; ndLatRRel];

    if isempty(dbData{1})
        dbData{1}{1} = newDataL;
        dbData{1}{2} = newDataR;
        dbPaintFlag{1}{1} = ndLinePaintFlagL;
        dbPaintFlag{1}{2} = ndLinePaintFlagR;
    else
        dbDataIdx = 1;

        newDataCell = cell(1);
        newDataCell{1} = newDataL;
        newDataCell{2} = newDataR;
        ndPaintFlagCell = cell(1);
        ndPaintFlagCell{1} = ndLinePaintFlagL;
        ndPaintFlagCell{2} = ndLinePaintFlagR;

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
                [Tr_fit, activeIdx, matchIdx, closeIdx, minDist, meanDist] = icpMex(dbPaintArr,ndPaintArr,Tr_fit,5,'point_to_point',200);
                Tr_fit

                if(length(activeIdx) < 20)
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
    lmInterval = 10;
    count = lmInterval;
    for idx = 2:length(linePaintFlag)
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

    landMarker = newData(:, landMarkFlag==1);
end
