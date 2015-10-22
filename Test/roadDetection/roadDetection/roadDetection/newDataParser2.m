function newDataParser()

% addpath('C:\Users\feng.liang\Desktop\GPS\');

close all;
clc;

% lonStand = -83.213250649943689;
% latStand = 42.296855933108084;
% lonStand = 52.35299955;
% latStand = 10.693509536666665;
% get line of file
for fileIdx = [0]
    fid = fopen(['dataStruct_',num2str(fileIdx),'.txt'], 'r');
    row=0;
    while ~feof(fid)
        row=row+sum(fread(fid,10000,'*char')==char(10));
    end
    fclose(fid);

    % get data
    fid = fopen(['dataStruct_',num2str(fileIdx),'.txt'], 'r');
    Data = zeros(19, row);
    lineIdx = 1;
    while ~feof(fid)
        fileLine = fscanf(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 19);
        if ~isempty(fileLine)
            Data(:, lineIdx) = fileLine;
            lineIdx = lineIdx + 1;
        else
            break;
        end
    end
    fclose(fid);

    eval(['latL' num2str(fileIdx) ' = Data(1, :);']);
    eval(['lonL' num2str(fileIdx) ' = Data(2, :);']);
    eval(['latR' num2str(fileIdx) ' = Data(12, :);']);
    eval(['lonR' num2str(fileIdx) ' = Data(13, :);']);
    eval(['linePaintFlagL' num2str(fileIdx) ' = Data(3, :);']);
    eval(['linePaintFlagR' num2str(fileIdx) ' = Data(14, :);']);
    
    eval(['latLRel' num2str(fileIdx) ' = latL' num2str(fileIdx) ';']);
    eval(['lonLRel' num2str(fileIdx) ' = lonL' num2str(fileIdx) ';']);
    eval(['latRRel' num2str(fileIdx) ' = latR' num2str(fileIdx) ';']);
    eval(['lonRRel' num2str(fileIdx) ' = lonR' num2str(fileIdx) ';']);
    
%     eval(['[lonLRel' num2str(fileIdx) ', latLRel' num2str(fileIdx) '] = calcRelativeLocation(lonStand, latStand, lonL' num2str(fileIdx) ', latL' num2str(fileIdx) ');']);
%     eval(['[lonRRel' num2str(fileIdx) ', latRRel' num2str(fileIdx) '] = calcRelativeLocation(lonStand, latStand, lonR' num2str(fileIdx) ', latR' num2str(fileIdx) ');']);
%     eval(['lonLRel' num2str(fileIdx) ' = lonL' num2str(fileIdx) ';']);
%     eval(['latLRel' num2str(fileIdx) ' = latL' num2str(fileIdx) ';']);
%     eval(['lonRRel' num2str(fileIdx) ' = lonR' num2str(fileIdx) ';']);
%     eval(['latRRel' num2str(fileIdx) ' = latR' num2str(fileIdx) ';']);
end

figure(1);
hold on;
plot(lonLRel0(linePaintFlagL0 == 1), latLRel0(linePaintFlagL0 == 1), 'r.');
plot(lonRRel0(linePaintFlagR0 == 1), latRRel0(linePaintFlagR0 == 1), 'r.');
% plot(lonLRel1(linePaintFlagL1 == 1), latLRel1(linePaintFlagL1 == 1), 'g.');
% plot(lonRRel1(linePaintFlagR1 == 1), latRRel1(linePaintFlagR1 == 1), 'g.');
% plot(lonLRel2(linePaintFlagL2 == 1), latLRel2(linePaintFlagL2 == 1), 'b.');
% plot(lonRRel2(linePaintFlagR2 == 1), latRRel2(linePaintFlagR2 == 1), 'b.');
% plot(lonLRel3(linePaintFlagL3 == 1), latLRel3(linePaintFlagL3 == 1), 'm.');
% plot(lonRRel3(linePaintFlagR3 == 1), latRRel3(linePaintFlagR3 == 1), 'm.');
% plot(lonLRel4(linePaintFlagL4 == 1), latLRel4(linePaintFlagL4 == 1), 'b.');
% plot(lonRRel4(linePaintFlagR4 == 1), latRRel4(linePaintFlagR4 == 1), 'b.');
% 
% plot(lonLRel5(linePaintFlagL5 == 1), latLRel5(linePaintFlagL5 == 1), 'g.');
% plot(lonRRel5(linePaintFlagR5 == 1), latRRel5(linePaintFlagR5 == 1), 'g.');
% plot(lonLRel6(linePaintFlagL6 == 1), latLRel6(linePaintFlagL6 == 1), 'r.');
% plot(lonRRel6(linePaintFlagR6 == 1), latRRel6(linePaintFlagR6 == 1), 'r.');
% plot(lonLRel7(linePaintFlagL7 == 1), latLRel7(linePaintFlagL7 == 1), 'g.');
% plot(lonRRel7(linePaintFlagR7 == 1), latRRel7(linePaintFlagR7 == 1), 'g.');
% plot(lonLRel8(linePaintFlagL8 == 1), latLRel8(linePaintFlagL8 == 1), 'g.');
% plot(lonRRel8(linePaintFlagR8 == 1), latRRel8(linePaintFlagR8 == 1), 'g.');
% plot(lonLRel9(linePaintFlagL9 == 1), latLRel9(linePaintFlagL9 == 1), 'g.');
% plot(lonRRel9(linePaintFlagR9 == 1), latRRel9(linePaintFlagR9 == 1), 'g.');
% % 
% % 
% plot(lonLRel10(linePaintFlagL10 == 1), latLRel10(linePaintFlagL10 == 1), 'b.');
% plot(lonRRel10(linePaintFlagR10 == 1), latRRel10(linePaintFlagR10 == 1), 'b.');
% plot(lonLRel11(linePaintFlagL11 == 1), latLRel11(linePaintFlagL11 == 1), 'b.');
% plot(lonRRel11(linePaintFlagR11 == 1), latRRel11(linePaintFlagR11 == 1), 'b.');
% plot(lonLRel12(linePaintFlagL12 == 1), latLRel12(linePaintFlagL12 == 1), 'b.');
% plot(lonRRel12(linePaintFlagR12 == 1), latRRel12(linePaintFlagR12 == 1), 'b.');
% plot(lonLRel13(linePaintFlagL13 == 1), latLRel13(linePaintFlagL13 == 1), 'b.');
% plot(lonRRel13(linePaintFlagR13 == 1), latRRel13(linePaintFlagR13 == 1), 'b.');
% plot(lonLRel14(linePaintFlagL14 == 1), latLRel14(linePaintFlagL14 == 1), 'b.');
% plot(lonRRel14(linePaintFlagR14 == 1), latRRel14(linePaintFlagR14 == 1), 'b.');

% 
% % plot(lonLRel18(linePaintFlagL18 == 1), latLRel18(linePaintFlagL18 == 1), 'b.');
% % plot(lonRRel18(linePaintFlagR18 == 1), latRRel18(linePaintFlagR18 == 1), 'b.');
% plot(lonLRel19(linePaintFlagL19 == 1), latLRel19(linePaintFlagL19 == 1), 'm.');
% plot(lonRRel19(linePaintFlagR19 == 1), latRRel19(linePaintFlagR19 == 1), 'm.');


dbData = cell(1);


while 1 % before end of file
    %% Get data from file
    

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
    newData = [lonLRel(linePaintFlagL == 1), lonRRel(linePaintFlagR == 1); ...
               latLRel(linePaintFlagL == 1), latRRel(linePaintFlagR == 1)];
    
    if isempty(dbData{1})
        dbData{1} = newData;
    else
        dbDataIdx = 1;
        while dbDataIdx <= length(dbData)
        
            [Tr_fit, activeIdx, matchIdx, closeIdx, minDist] = icpMex(dbData{dbDataIdx},newData,eye(3),7,'point_to_point');

            if(length(activeIdx) < 30)
                dbDataIdx = dbDataIdx + 1;
                continue;
            else
                % merge
                newDataMoveCoeff = 0.7;
                newData_fit = newData + newDataMoveCoeff.*(Tr_fit(1:2,3)*ones(1,size(newData,2))); % Tr_fit(1:2,1:2)
                
                dbData_fit = dbData{dbDataIdx} - (1-newDataMoveCoeff).*(Tr_fit(1:2,3)*ones(1,size(dbData{dbDataIdx},2)));

                mergedData = merge(newData_fit, dbData_fit);
                
                figure(1);
                hold off;
                plot(newData(1,:), newData(2,:), 'rs');
                hold on;
                plot(dbData{dbDataIdx}(1,:), dbData{dbDataIdx}(2,:), 'bs');
                plot(mergedData(1,:), mergedData(2,:), 'gs');
                
                newData = mergedData;
                dbData(dbDataIdx, :) = [];
                
                dbDataIdx = dbDataIdx + 1;
            end
            
        end
        
        % push the data as is
        dbData = [dbData; newData];
        
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

end

%%
function [x, y] = calcRelativeLocation(lonStand, latStand, lon, lat)
    latDegree = latStand * pi / 180;
    x = (lon - lonStand) * (111413*cos(latDegree) - 94*cos(3*latDegree));
    y = (lat - latStand) * 111320.0;
end

%%
function mergedData = merge(newData_fit, dbData_fit)
    mergedData = [newData_fit, dbData_fit];
end
