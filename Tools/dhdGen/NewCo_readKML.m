function data = NewCo_readKML(fileName)
    dataIndex = 1;
    fid = fopen(fileName, 'rt');
    while ~feof(fid) 
        tline=fgetl(fid);
        if(strfind(tline, '<coordinates>'))
            tline=fgetl(fid);
            GPScoordinate = regexp(tline,'\d*\.?\d*','match');
        end
    end
    fclose(fid);
    
    gpsLen = length(GPScoordinate);
    for i = 1:3:gpsLen
        data(2, dataIndex) = str2double(GPScoordinate(i));
        data(1, dataIndex) = str2double(GPScoordinate(i+1));
        data(3, dataIndex) = str2double(GPScoordinate(i+2));
        dataIndex = dataIndex + 1;
    end
end
