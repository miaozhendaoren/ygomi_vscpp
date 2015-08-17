clc;
clear all;
close all;

dis = 0.00004;
disDash = 0.000030;
datafromFile = 0; %0: from gpx, 1: from txt
outputFormat = 1; %0: to dhd, 1: to kml

if datafromFile == 1
    fid = fopen('NEMA_GPS_data.txt', 'rt');
    date = fscanf(fid,'%f,%f,%f',[3,inf]);
    fclose(fid);
else
    fid = fopen('Loop2-removeOverlap.gpx', 'rt');

    date = zeros();
    dataIndex = 1;

    while ~feof(fid) 
        tline=fgetl(fid);
        if(strfind(tline, '<trkpt'))
            latIndex = strfind(tline, 'lat');
            lonIndex = strfind(tline, 'lon');
            if latIndex < lonIndex
                quotation = strfind(tline, '"');
                lat = tline(quotation(1)+1 : quotation(2)-1);
                lon = tline(quotation(3)+1 : quotation(4)-1);

                tline=fgetl(fid);
                quotation = strfind(tline, 'ele');
                ati = tline(quotation(1)+4 : quotation(2)-3);

                date(1, dataIndex) = str2double(lat);
                date(2, dataIndex) = str2double(lon);
                date(3, dataIndex) = str2double(ati);
                dataIndex = dataIndex + 1;
            else
                quotation = strfind(tline, '"');
                lon = tline(quotation(1)+1 : quotation(2)-1);
                lat = tline(quotation(3)+1 : quotation(4)-1);

                tline=fgetl(fid);
                quotation = strfind(tline, 'ele');
                ati = tline(quotation(1)+4 : quotation(2)-3);

                date(1, dataIndex) = str2double(lat);
                date(2, dataIndex) = str2double(lon);
                date(3, dataIndex) = str2double(ati);
                dataIndex = dataIndex + 1;
            end
        end
    end
    fclose(fid);
end

% altitude = date(3,1);
altitude = 0;
date(3, :) = altitude;

planData = [date(2,:) date(1,:)];

len = length(date(1,:));

LEFT = zeros();
RIGHT = zeros();

segmentIndex = 1;
segmentNum = 1;
middleIndex = 1;
leftIndex = 1;
rightIndex = 1;
marginIndex = 1;
marginDashIndex = 1;

planData = reshape(planData, len, 2);
% LEFT = {};
% RIGHT = {};

segment(1, 1) = 1;

kkkk = zeros();

%Loop2
segmentNum = 7;
segment(1, 1) = 1;
segment(1, 2) = 6;
segment(1, 3) = 49;
segment(1, 4) = 57;
segment(1, 5) = 80;
segment(1, 6) = 100;
segment(1, 7) = 104;
segment(1, 8) = 128;
% %Loop4
% segmentNum = 7;
% segment(1, 1) = 1;
% segment(1, 2) = 4;
% segment(1, 3) = 41;
% segment(1, 4) = 48;
% segment(1, 5) = 68;
% segment(1, 6) = 91;
% segment(1, 7) = 108;

preslopeFlag = 0;
slopeRange = 0.57;
i = 1;
dataLen = size(planData, 1);
while i <= dataLen
% for i = 1 : len-2
    dataIndex = mod(i-1, size(planData, 1));
    if dataIndex == 0
        dataIndex = dataLen;        
    end
    A = planData(dataIndex,:);
    
    dataIndex = mod(i, size(planData, 1));
    if dataIndex == 0
        dataIndex = dataLen;        
    end
    B = planData(dataIndex,:);
    
    dataIndex = mod(i+1, size(planData, 1));
    if dataIndex == 0
        dataIndex = dataLen;        
    end
    C = planData(dataIndex,:);
    
    dataIndex = mod(i+2, size(planData, 1));
    if dataIndex == 0
        dataIndex = dataLen;        
    end
    D = planData(dataIndex,:);
%     A = planData(mod(i-1, size(planData, 1))+1,:);
%     B = planData(mod(i,   size(planData, 1))+1,:);
%     C = planData(mod(i+1, size(planData, 1))+1,:);
%     D = planData(mod(i+2, size(planData, 1))+1,:);
%     %D = planData(mod(i+2, size(planData, 1))+1,:);
    
    k1 = -(A(1) - B(1))/(A(2) - B(2));
    k2 = -(B(1) - C(1))/(B(2) - C(2));
    k3 = -(C(1) - D(1))/(C(2) - D(2));
    %k3 = -(B(1) - C(1))/(B(2) - C(2));
    
    k = (k1+k2+k3)/3; 
    if inf == k1 || -inf == k1 || isnan(k1)
        k = (k2+k3)/2; 
    end
    if inf == k2 || -inf == k2 || isnan(k2)
        k = (k1+k3)/2; 
    end
    if inf == k3 || -inf == k3 || isnan(k3)
        k = (k1+k2)/2; 
    end
    
    if (inf == k1 && inf == k2) || (-inf == k1 && -inf == k2) || (isnan(k1) && isnan(k2))
        k = k3;
    end
    if (inf == k1 && inf == k3) || (-inf == k1 && -inf == k3) || (isnan(k1) && isnan(k3))
        k = k2;
    end
    if (inf == k2 && inf == k3) || (-inf == k2 && -inf == k3) || (isnan(k2) && isnan(k3))
        k = k1; 
    end
    
    C = [
         (k^2*B(1)+B(1)+(dis^2+k^2*dis^2)^(1/2))/(1+k^2), ...
         k*(k^2*B(1)+B(1)+(dis^2+k^2*dis^2)^(1/2))/(1+k^2)-B(1)*k+B(2)];
 
    D =[ 
         (k^2*B(1)+B(1)-(dis^2+k^2*dis^2)^(1/2))/(1+k^2), ...
         k*(k^2*B(1)+B(1)-(dis^2+k^2*dis^2)^(1/2))/(1+k^2)-B(1)*k+B(2)];

     if (i>=segment(1, 4)) && (i<=segment(1, 7))
         Cdash = [
             (k^2*B(1)+B(1)+(disDash^2+k^2*disDash^2)^(1/2))/(1+k^2), ...
             k*(k^2*B(1)+B(1)+(disDash^2+k^2*disDash^2)^(1/2))/(1+k^2)-B(1)*k+B(2)];

         Ddash =[ 
             (k^2*B(1)+B(1)-(disDash^2+k^2*disDash^2)^(1/2))/(1+k^2), ...
             k*(k^2*B(1)+B(1)-(disDash^2+k^2*disDash^2)^(1/2))/(1+k^2)-B(1)*k+B(2)];
     end
     
%     if isnan(C(1)) 
%         k = 0;
%         
%         planData(i,:) = [];
%         date(:,i) = [];
%         
%         continue;
%     end
%     if k <= 0
    if A(1) < B(1)        
        if A(2) < B(2)
        LEFT(2, marginIndex) = C(1);
        LEFT(1, marginIndex) = C(2);
        RIGHT(2, marginIndex) = D(1);
        RIGHT(1, marginIndex) = D(2);
            if (i>=segment(1, 4)) && (i<=segment(1, 7))
            LEFTdash(2, marginDashIndex) = Cdash(1);
            LEFTdash(1, marginDashIndex) = Cdash(2);
            RIGHTdash(2, marginDashIndex) = Ddash(1);
            RIGHTdash(1, marginDashIndex) = Ddash(2);
            end
        else
        LEFT(2, marginIndex) = D(1);
        LEFT(1, marginIndex) = D(2);
        RIGHT(2, marginIndex) = C(1);
        RIGHT(1, marginIndex) = C(2);
            if (i>=segment(1, 4)) && (i<=segment(1, 7))
            LEFTdash(2, marginDashIndex) = Ddash(1);
            LEFTdash(1, marginDashIndex) = Ddash(2);
            RIGHTdash(2, marginDashIndex) = Cdash(1);
            RIGHTdash(1, marginDashIndex) = Cdash(2);
            end
        end
    else
        if A(2) > B(2)
        LEFT(2, marginIndex) = D(1);
        LEFT(1, marginIndex) = D(2);
        RIGHT(2, marginIndex) = C(1);
        RIGHT(1, marginIndex) = C(2);
            if (i>=segment(1, 4)) && (i<=segment(1, 7))
            LEFTdash(2, marginDashIndex) = Ddash(1);
            LEFTdash(1, marginDashIndex) = Ddash(2);
            RIGHTdash(2, marginDashIndex) = Cdash(1);
            RIGHTdash(1, marginDashIndex) = Cdash(2);
            end
        else
        LEFT(2, marginIndex) = C(1);
        LEFT(1, marginIndex) = C(2);
        RIGHT(2, marginIndex) = D(1);
        RIGHT(1, marginIndex) = D(2);
            if (i>=segment(1, 4)) && (i<=segment(1, 7))
            LEFTdash(2, marginDashIndex) = Cdash(1);
            LEFTdash(1, marginDashIndex) = Cdash(2);
            RIGHTdash(2, marginDashIndex) = Ddash(1);
            RIGHTdash(1, marginDashIndex) = Ddash(2);
            end
        end
    end
    
    if ((k1<0&&k1~=-inf) && (k2>=0||k2==-inf||k2==inf)) && (A(1) > B(1))
        temp1 = LEFT(2, marginIndex);
        temp2 = LEFT(1, marginIndex);
        
        LEFT(2, marginIndex) = RIGHT(2, marginIndex);
        LEFT(1, marginIndex) = RIGHT(1, marginIndex);
        
        RIGHT(2, marginIndex) = temp1;
        RIGHT(1, marginIndex) = temp2;
        
        if (i>=segment(1, 4)) && (i<=segment(1, 7))
        temp1 = LEFTdash(2, marginDashIndex);
        temp2 = LEFTdash(1, marginDashIndex);
        
        LEFTdash(2, marginDashIndex) = RIGHTdash(2, marginDashIndex);
        LEFTdash(1, marginDashIndex) = RIGHTdash(1, marginDashIndex);
        
        RIGHTdash(2, marginDashIndex) = temp1;
        RIGHTdash(1, marginDashIndex) = temp2;
        end
    end
    
    %%
%     if  mod(i, 8) == 0
%         segmentNum = segmentNum + 1;
%         segment(1, segmentNum) = i;
%     end
    if (i>=segment(1, 4)) && (i<=segment(1, 7))
        marginDashIndex = marginDashIndex + 1;
    end
    marginIndex = marginIndex + 1;
    i = i + 1;
end

LEFT(:,marginIndex-1) = LEFT(:,marginIndex-2);
RIGHT(:,marginIndex-1) = RIGHT(:,marginIndex-2);

totalNumPoint = i - 1;

plot(date(2,:), date(1,:), '-bo');
hold on;
string = num2str(1 : length(date(2,:)));
for i = 1:length(date(2,:))
    text(date(2,i), date(1,i), num2str(i));
end

plot(LEFT(2,:), LEFT(1,:), '-ro');
hold on;
plot(RIGHT(2,:), RIGHT(1,:), '-go');

hold on;
plot(LEFTdash(2,:), LEFTdash(1,:), '--m');
hold on;
plot(RIGHTdash(2,:), RIGHTdash(1,:), '--m');

segPort = zeros(segmentNum, 2);
for segIdx = 1:segmentNum
    segPortPoint(1) = segment(1, mod(segIdx+1-2, segmentNum)+1);
    segPortPoint(2) = segment(1, mod(segIdx+2-2, segmentNum)+1);
    segPort(segIdx, :) = segPortPoint;
end

%% Output
if outputFormat == 0 % dhd
    %%
    fid = fopen('road_data_segment.bin', 'wb');
    fwrite(fid, hex2dec('0001'), 'int16');%header typeId
    fwrite(fid, 2+segmentNum*4, 'uint32');

    fwrite(fid, hex2dec('0002'), 'int16');%Segment typeId
    fwrite(fid, 21, 'uint32');% length of Segment
    for segmentIndex = 1:segmentNum
        SegmentID = segmentIndex;
        SegmentVersion = 1;
        SegmentLength = 1;
        SegmentType = 0;
        NumPort  = 2;

        % Segment
        fwrite(fid, hex2dec('0701'), 'int16');
        fwrite(fid, SegmentID, 'uint32');
        fwrite(fid, hex2dec('0702'), 'int16');
        fwrite(fid, SegmentVersion, 'uint32');
        fwrite(fid, hex2dec('0703'), 'int16');
        fwrite(fid, SegmentLength, 'uint32');
        fwrite(fid, hex2dec('0704'), 'int16');
        fwrite(fid, SegmentType, 'uint8');
        fwrite(fid, hex2dec('0705'), 'int16');
        fwrite(fid, NumPort, 'uint32');
        for i = 1 : NumPort
            fwrite(fid, hex2dec('0051'), 'int16');
            fwrite(fid, date(1, segPort(segmentIndex, i)), 'double');
            fwrite(fid, hex2dec('0052'), 'int16');
            fwrite(fid, date(2, segPort(segmentIndex, i)), 'double');
            fwrite(fid, hex2dec('0053'), 'int16');
            fwrite(fid, date(3, segPort(segmentIndex, i)), 'double');
            %plot(date(2,segPort(segmentIndex, i)), date(1,segPort(segmentIndex, i)), '-b*');
        end

        fwrite(fid, hex2dec('0706'), 'int16');
        fwrite(fid, NumPort, 'uint32');
    %     for i = 1 : NumPort
            if segmentIndex == 1
                fwrite(fid, segmentNum, 'int32');
                fwrite(fid, segmentIndex+1, 'int32');
            elseif segmentIndex == segmentNum
                fwrite(fid, segmentIndex-1, 'int32');
                fwrite(fid, 1, 'int32');
            else
                fwrite(fid, segmentIndex-1, 'int32');
                fwrite(fid, segmentIndex+1, 'int32');
            end        
    %     end
    end

    %% Vector
    fwrite(fid, hex2dec('0003'), 'int16');%Vector typeId
    fwrite(fid, 41+len*30, 'uint32');% length of Vector
    for segmentIndex = 1:segmentNum
        SegmentID = segmentIndex;
        segIdxTmp = mod(segmentIndex+i-2, segmentNum)+1;

        NumVector = 3;

        fwrite(fid, hex2dec('1001'), 'int16');
        fwrite(fid, SegmentID, 'int32');
        fwrite(fid, hex2dec('1002'), 'int16');
        fwrite(fid, NumVector, 'int32');%NumVector
        % Vector
        VectorID  = 1;
        Width = 0.2;
        LineStyle = 1;
        SegVersion = 1;
        len = mod(segPort(segmentIndex, 2) - segPort(segmentIndex, 1), totalNumPoint) + 1;
        fwrite(fid, hex2dec('0101'), 'int16');
        fwrite(fid, VectorID, 'uint32');
        fwrite(fid, hex2dec('0102'), 'int16');
        fwrite(fid, Width, 'float');%f\n', );
        fwrite(fid, hex2dec('0103'), 'int16');%d\n', LineStyle);
        fwrite(fid, LineStyle, 'uint8');%d\n', LineStyle);
        fwrite(fid, hex2dec('0104'), 'int16');%d\n', SegVersion);
        fwrite(fid, SegVersion, 'uint32');
        fwrite(fid, hex2dec('0105'), 'int16');%d\n', len);
        fwrite(fid, len, 'int32');%d\n', len);

        if segPort(segmentIndex, 1) <= segPort(segmentIndex, 2)
            pointIdx = segPort(segmentIndex, 1) : segPort(segmentIndex, 2);
        else
            pointIdx = [segPort(segmentIndex, 1) : totalNumPoint, 1 : segPort(segmentIndex, 2)];
        end

        for i = pointIdx
            fwrite(fid, hex2dec('0051'), 'int16');
            fwrite(fid, date(1,i), 'double');
            fwrite(fid, hex2dec('0052'), 'int16');
            fwrite(fid, date(2,i), 'double');
            fwrite(fid, hex2dec('0053'), 'int16');
            fwrite(fid, date(3,i), 'double');
        end

        %%    
        VectorID = 2;
        Width = 0.2;
        LineStyle = 2;
        SegVersion = 1;
        len = mod(segPort(segmentIndex, 2) - segPort(segmentIndex, 1), totalNumPoint) + 1;
        % Vector
        fwrite(fid, hex2dec('0101'), 'int16');
        fwrite(fid, VectorID, 'uint32');
        fwrite(fid, hex2dec('0102'), 'int16');
        fwrite(fid, Width, 'float');%f\n', );
        fwrite(fid, hex2dec('0103'), 'int16');%d\n', LineStyle);
        fwrite(fid, LineStyle, 'uint8');%d\n', LineStyle);
        fwrite(fid, hex2dec('0104'), 'int16');%d\n', SegVersion);
        fwrite(fid, SegVersion, 'uint32');
        fwrite(fid, hex2dec('0105'), 'int16');%d\n', len);
        fwrite(fid, len, 'int32');%d\n', len);

        if segPort(segmentIndex, 1) <= segPort(segmentIndex, 2)
            pointIdx = segPort(segmentIndex, 1) : segPort(segmentIndex, 2);
        else
            pointIdx = [segPort(segmentIndex, 1) : totalNumPoint, 1 : segPort(segmentIndex, 2)];
        end

        for i = pointIdx
            fwrite(fid, hex2dec('0051'), 'int16');
            fwrite(fid, LEFT(1,i), 'double');
            fwrite(fid, hex2dec('0052'), 'int16');
            fwrite(fid, LEFT(2,i), 'double');
            fwrite(fid, hex2dec('0053'), 'int16');
            fwrite(fid, altitude, 'double');
        end

        %%    
        VectorID = 3;
        Width = 0.2;
        LineStyle = 2;
        SegVersion = 1;
        len = mod(segPort(segmentIndex, 2) - segPort(segmentIndex, 1), totalNumPoint) + 1;
        % Vector
        fwrite(fid, hex2dec('0101'), 'int16');
        fwrite(fid, VectorID, 'uint32');
        fwrite(fid, hex2dec('0102'), 'int16');
        fwrite(fid, Width, 'float');%f\n', );
        fwrite(fid, hex2dec('0103'), 'int16');%d\n', LineStyle);
        fwrite(fid, LineStyle, 'uint8');%d\n', LineStyle);
        fwrite(fid, hex2dec('0104'), 'int16');%d\n', SegVersion);
        fwrite(fid, SegVersion, 'uint32');
        fwrite(fid, hex2dec('0105'), 'int16');%d\n', len);
        fwrite(fid, len, 'int32');%d\n', len);

        if segPort(segmentIndex, 1) <= segPort(segmentIndex, 2)
            pointIdx = segPort(segmentIndex, 1) : segPort(segmentIndex, 2);
        else
            pointIdx = [segPort(segmentIndex, 1) : totalNumPoint, 1 : segPort(segmentIndex, 2)];
        end

        for i = pointIdx
            fwrite(fid, hex2dec('0051'), 'int16');
            fwrite(fid, RIGHT(1,i), 'double');
            fwrite(fid, hex2dec('0052'), 'int16');
            fwrite(fid, RIGHT(2,i), 'double');
            fwrite(fid, hex2dec('0053'), 'int16');
            fwrite(fid, altitude, 'double');
        end
    end

    fclose(fid);
elseif outputFormat == 1 % kml
    addpath('./xml_io_tools_2010_11_05');
    
    [kml treeName] = xml_read ('test_file.xml');
    %gen_object_display(kml);
    
    Placemark = [];

    for segmentIndex = 1:segmentNum
        
        vector1 = [];
        vector2 = [];
        vector3 = [];
        vector4 = [];
        vector5 = [];
        dashVectorFlag = 0;
        
        if segPort(segmentIndex, 1) <= segPort(segmentIndex, 2)
            pointIdx = segPort(segmentIndex, 1) : segPort(segmentIndex, 2);
        else
            pointIdx = [segPort(segmentIndex, 1) : totalNumPoint, 1 : segPort(segmentIndex, 2)];
        end
        
        minI = min(pointIdx);
        maxI = max(pointIdx);

        for i = pointIdx
            vector1 = [vector1 num2str(date(2,i), 30) ','];
            vector1 = [vector1 num2str(date(1,i), 30) ','];
            vector1 = [vector1 num2str(date(3,i), 30) ' '];
            
            vector2 = [vector2 num2str(LEFT(2,i), 30) ','];
            vector2 = [vector2 num2str(LEFT(1,i), 30) ','];
            vector2 = [vector2 num2str(altitude, 30) ' '];
            
            vector3 = [vector3 num2str(RIGHT(2,i), 30) ','];
            vector3 = [vector3 num2str(RIGHT(1,i), 30) ','];
            vector3 = [vector3 num2str(altitude, 30) ' '];
            
            if (minI>=segment(1, 4)) && (maxI<=segment(1, 7))
                dashVectorFlag = 1;
                dashDataIndex = i - segPort(4, 1) + 1;
                vector4 = [vector4 num2str(LEFTdash(2,dashDataIndex), 30) ','];
                vector4 = [vector4 num2str(LEFTdash(1,dashDataIndex), 30) ','];
                vector4 = [vector4 num2str(altitude, 30) ' '];

                vector5 = [vector5 num2str(RIGHTdash(2,dashDataIndex), 30) ','];
                vector5 = [vector5 num2str(RIGHTdash(1,dashDataIndex), 30) ','];
                vector5 = [vector5 num2str(altitude, 30) ' '];
            end
        end

        % vector 1
        LineStringElement = [];
        LineStringElement.tessellate = 1;
        LineStringElement.coordinates = vector1;

        PlacemarkElement = [];
        PlacemarkElement.name = ['seg' num2str(segmentIndex) '_vector1_style0'];
        PlacemarkElement.styleUrl = '#m_ylw-pushpin';
        PlacemarkElement.LineString = LineStringElement;
        
        Placemark = [Placemark PlacemarkElement];
        
        % vector 2
        LineStringElement = [];
        LineStringElement.tessellate = 1;
        LineStringElement.coordinates = vector2;

        PlacemarkElement = [];
        PlacemarkElement.name = ['seg' num2str(segmentIndex) '_vector2_style5'];
        PlacemarkElement.styleUrl = '#m_ylw-pushpin';
        PlacemarkElement.LineString = LineStringElement;
        
        Placemark = [Placemark PlacemarkElement];
        
        % vector 3
        LineStringElement = [];
        LineStringElement.tessellate = 1;
        LineStringElement.coordinates = vector3;

        PlacemarkElement = [];
        PlacemarkElement.name = ['seg' num2str(segmentIndex) '_vector3_style5'];
        PlacemarkElement.styleUrl = '#m_ylw-pushpin';
        PlacemarkElement.LineString = LineStringElement;
        
        Placemark = [Placemark PlacemarkElement];
        
        if dashVectorFlag == 1
            % vector 4
            LineStringElement = [];
            LineStringElement.tessellate = 1;
            LineStringElement.coordinates = vector4;

            PlacemarkElement = [];
            PlacemarkElement.name = ['seg' num2str(segmentIndex) '_vector4_style1'];
            PlacemarkElement.styleUrl = '#m_ylw-pushpin';
            PlacemarkElement.LineString = LineStringElement;

            Placemark = [Placemark PlacemarkElement];
            
            % vector 5
            LineStringElement = [];
            LineStringElement.tessellate = 1;
            LineStringElement.coordinates = vector5;

            PlacemarkElement = [];
            PlacemarkElement.name = ['seg' num2str(segmentIndex) '_vector5_style1'];
            PlacemarkElement.styleUrl = '#m_ylw-pushpin';
            PlacemarkElement.LineString = LineStringElement;

            Placemark = [Placemark PlacemarkElement];
        end
    end
    
    kml.Document.Folder.Placemark = Placemark;
    
    wPref.StructItem = false;
    xml_write('out_path.kml', kml, 'kml', wPref);
end

