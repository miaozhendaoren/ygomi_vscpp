function roadData = NewCo_readKML_All(fileName)
    fid = fopen(fileName, 'rt');
    while ~feof(fid) 
        tline=fgetl(fid);
        segIndices = strfind(tline, '<name>seg');
        if(segIndices ~= 0)
            vecIndices = strfind(tline, '_vector');
            segIndex = str2double(tline(segIndices + 9 : vecIndices-1));            
            styleIndices = strfind(tline, '_style');
            vectorIndex = str2double(tline(vecIndices + 7 : styleIndices-1));
            endIndices = strfind(tline, '</name>');
            styleIndex = str2double(tline(styleIndices + 6 : endIndices-1));
        end
        coordinates = strfind(tline, '<coordinates>');
        if(coordinates ~= 0)
            if(strfind(tline, '</coordinates>'))
            else
                tline=fgetl(fid);
            end
            GPScoordinate = regexp(tline,'\d*\.?\d*','match');
                                
            gpsLen = length(GPScoordinate);
            
            if gpsLen > 0
                dataIndex = 1;
                data = zeros(4, gpsLen/3);
                for i = 1:3:gpsLen
                    data(2, dataIndex) = str2double(GPScoordinate(i));
                    data(1, dataIndex) = str2double(GPScoordinate(i+1));
                    data(3, dataIndex) = str2double(GPScoordinate(i+2));
                    data(4, dataIndex) = styleIndex;
                    dataIndex = dataIndex + 1;
                end
                roadData{segIndex, vectorIndex} = data;
            end
        end
    end
    fclose(fid);
end
