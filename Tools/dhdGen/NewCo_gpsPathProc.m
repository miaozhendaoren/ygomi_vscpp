clc;
clear all;
close all;

%%
err = 0.000035;

roadDate = NewCo_readKML_All('out_path.kml');

segmentNum = length(roadDate(:, 1));

%%
fid = fopen('road_data_segment.dhd', 'wb');
fwrite(fid, hex2dec('0001'), 'int16');%header typeId
fwrite(fid, 2+segmentNum*4, 'uint32');

fwrite(fid, hex2dec('0002'), 'int16');%Segment typeId
fwrite(fid, 21, 'uint32');% length of Segment

%% segment
for segmentIndex = 1 : segmentNum    
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
    for i = [1, length(roadDate{SegmentID, 1}(1,:))]
        fwrite(fid, hex2dec('0051'), 'int16');
        fwrite(fid, roadDate{SegmentID, 1}(1,i), 'double');
        fwrite(fid, hex2dec('0052'), 'int16');
        fwrite(fid, roadDate{SegmentID, 1}(2,i), 'double');
        fwrite(fid, hex2dec('0053'), 'int16');
        fwrite(fid, roadDate{SegmentID, 1}(3,i), 'double');
    end
    
    fwrite(fid, hex2dec('0706'), 'int16');
    fwrite(fid, NumPort, 'uint32');
    
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
end

%% Vector
fwrite(fid, hex2dec('0003'), 'int16');%Vector typeId
fwrite(fid, 41+segmentNum*30, 'uint32');% length of Vector
for segmentIndex = 1:segmentNum
    SegmentID = segmentIndex;
    
    nonZeros = cellfun(@(x)~isempty(x),roadDate(SegmentID, :),'UniformOutput',true);
    vectorNum = sum(nonZeros(:));

    fwrite(fid, hex2dec('1001'), 'int16');
    fwrite(fid, SegmentID, 'int32');
    fwrite(fid, hex2dec('1002'), 'int16');
    fwrite(fid, vectorNum, 'int32');%vectorNum
    % Vector
    for vectorIndex = 1 : vectorNum    
        VectorID  = vectorIndex;
        Width = 0.2;
        LineStyle = roadDate{SegmentID, VectorID}(4,1);
        SegVersion = 1;
        len = length(roadDate{SegmentID, VectorID}(4,:));
        
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

%         if segPort(segmentIndex, 1) <= segPort(segmentIndex, 2)
%             pointIdx = segPort(segmentIndex, 1) : segPort(segmentIndex, 2);
%         else
%             pointIdx = [segPort(segmentIndex, 1) : totalNumPoint, 1 : segPort(segmentIndex, 2)];
%         end

        for i = 1:len
            fwrite(fid, hex2dec('0051'), 'int16');
            fwrite(fid, roadDate{SegmentID, VectorID}(1,i), 'double');
            fwrite(fid, hex2dec('0052'), 'int16');
            fwrite(fid, roadDate{SegmentID, VectorID}(2,i), 'double');
            fwrite(fid, hex2dec('0053'), 'int16');
            fwrite(fid, roadDate{SegmentID, VectorID}(3,i), 'double');            
        end
        
        plot(roadDate{SegmentID, VectorID}(2,:), roadDate{SegmentID, VectorID}(1,:), '-go');
        hold on;
    end
    
    %%
    lat1 = roadDate{SegmentID, 1}(1, 1);
    lat2 = roadDate{SegmentID, 1}(1, end);
    lon1 = roadDate{SegmentID, 1}(2, 1);
    lon2 = roadDate{SegmentID, 1}(2, end);
    
    plot(lon1, lat1, lon2, lat2, 'b*');
    
    if lat1 > lat2
        lat1 = lat1 + err;
        lat2 = lat2 - err;
    else
        lat1 = lat1 - err;
        lat2 = lat2 + err;
        
        temp = lat1;
        lat1 = lat2;
        lat2 = temp;
    end
    
    if lon1 > lon2
        lon1 = lon1 + err;
        lon2 = lon2 - err;
    else
        lon1 = lon1 - err;
        lon2 = lon2 + err;
        
        temp = lon1;
        lon1 = lon2;
        lon2 = temp;
    end
    
    hold on;
    plot([lon1 lon1], [lat1 lat2], 'r-');
    plot([lon2 lon2], [lat1 lat2], 'r-');
    plot([lon1 lon2], [lat1 lat1], 'r-');
    plot([lon1 lon2], [lat2 lat2], 'r-');
end

fclose(fid);
